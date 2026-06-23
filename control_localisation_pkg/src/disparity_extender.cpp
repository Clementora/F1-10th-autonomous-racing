#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::placeholders;
using namespace std::chrono_literals;

class DisparityExtender : public rclcpp::Node {
public:
    DisparityExtender() : Node("disparity_extender_node") {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("scan_topic", "/yellow_car/scan");
        this->declare_parameter<std::string>("drive_topic", "/yellow_car/cmd_ackermann");
        this->declare_parameter<double>("car_width", 0.30);       // Width of the car in meters
        this->declare_parameter<double>("tolerance", 0.10);       // Safety margin in meters
        this->declare_parameter<double>("disparity_threshold", 0.20); // Meters
        this->declare_parameter<double>("max_steering_angle", 0.4189); // ~24 degrees in radians
        this->declare_parameter<double>("max_speed", 1.5);      // Default constant speed

        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        car_width_ = this->get_parameter("car_width").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();

        // Publishers & Subscribers (Using SensorDataQoS for reliable high-rate LIDAR streaming)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, //rclcpp::SensorDataQoS(), 
            std::bind(&DisparityExtender::scan_callback, this, _1));

        pose_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, 
                                                                          std::bind(&DisparityExtender::callback_odom, this, _1));
        
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            drive_topic, 10);

        main_timer_ = create_wall_timer(0.1s, std::bind(&DisparityExtender::main_loop, this));

        RCLCPP_INFO(this->get_logger(), "Disparity Extender Node Initialized.");
    }

private:

    // Parameters
    double car_width_;
    double tolerance_;
    double disparity_threshold_;
    double max_steering_angle_;
    double max_speed_;
    double steering_vel = 2.0;

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    nav_msgs::msg::Odometry odom_msg_;

    float target_angle_; // angle that has the longest obstacle free distance
    float curr_drive_speed_; // calculated drive speed
    float forward_distance_;
    float curr_yaw_;

    bool has_odom_ = false;
    bool has_obstacles_ = false;

    const double MIN_SAFE_DIST_ = 0.5;   // Below this, stop completely (meters)
    const double MID_SAFE_DIST_ = 2.0;   // Breakpoint between segments (meters)
    const double MAX_SAFE_DIST_ = 5.0;   // Above this, go max speed (meters)

    const double MID_SPEED_ = 1.5;   // Target speed at the midpoint (m/s)

    void callback_odom(nav_msgs::msg::Odometry::SharedPtr msg){
        odom_msg_ = *msg;
        // double curr_x = odom_msg_.pose.pose.position.x;
        // double curr_y = odom_msg_.pose.pose.position.y;
        // curr_pos_ = Eigen::Vector2d(curr_x, curr_y);
        
        double q_z = odom_msg_.pose.pose.orientation.z;
        double q_w = odom_msg_.pose.pose.orientation.w;
        
        curr_yaw_ = 2.0 * std::atan2(q_z, q_w);
        has_odom_ = true;   
    }

    void main_loop(){

        if(!has_odom_ || !has_obstacles_){
            return;
        }

        auto speed_msg = ackermann_msgs::msg::AckermannDrive();
        speed_msg.steering_angle = target_angle_;
        // msg.steering_angle_velocity = steering_vel;
        speed_msg.speed = calculate_dynamic_speed(forward_distance_);

        drive_pub_->publish(speed_msg);
        

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. Clone raw LIDAR data to create a mutable filtered list
        std::vector<float> filtered_ranges = msg->ranges;
        std::vector<float> ranges = msg->ranges;
        size_t num_samples = filtered_ranges.size();
        RCLCPP_INFO(this->get_logger(), "Subscribing to lidar topic");

        has_obstacles_ = true;

        for (size_t i = 1; i<num_samples; ++i){

            if(std::isnan(ranges[i]) || std::isinf(ranges[i])){
                ranges[i] = msg->range_max;
                filtered_ranges[i] = msg->range_max;
            }
        }

        // 2 & 3. Identify and extend disparities across the array
        for (size_t i = 1; i < num_samples; ++i) {
            // Skip invalid measurements
            // if (std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i]) ||
            //     std::isnan(msg->ranges[i - 1]) || std::isinf(msg->ranges[i - 1])) {
            //     continue;
            // }

            float diff = std::abs(ranges[i] - ranges[i - 1]);
            
            if (diff > disparity_threshold_) {
                float closer_dist = std::min(ranges[i], ranges[i - 1]);
                
                // Prevent division by zero/very small distances
                closer_dist = std::max(closer_dist, msg->range_min);

                // Calculate number of samples needed to cover half the car width + tolerance
                float half_width_total = (car_width_ / 2.0f) + tolerance_;
                float angle_to_cover = half_width_total / closer_dist; 
                int samples_to_cover = static_cast<int>(std::ceil(angle_to_cover / msg->angle_increment));

                if (ranges[i] > ranges[i - 1]) {
                    // Step Up Disparity: i-1 is closer (obstacle), i is farther (background)
                    // Overwrite starting at i, moving forward (rightward in array traversal)
                    int start_idx = static_cast<int>(i);
                    int end_idx = std::min(static_cast<int>(num_samples), start_idx + samples_to_cover);
                    
                    for (int j = start_idx; j < end_idx; ++j) {
                        filtered_ranges[j] = std::min(filtered_ranges[j], closer_dist);
                    }
                } else {
                    // Step Down Disparity: i is closer (obstacle), i-1 is farther (background)
                    // Overwrite starting at i-1, moving backward (leftward in array traversal)
                    int start_idx = static_cast<int>(i - 1);
                    int end_idx = std::max(0, start_idx - samples_to_cover);
                    
                    for (int j = start_idx; j > end_idx; --j) {
                        filtered_ranges[j] = std::min(filtered_ranges[j], closer_dist);
                    }
                }
            }
        }

        // 4. Find the farthest reachable distance within the forward-facing [-90, +90] degree arc
        float max_dist = -1.0f;
        int best_idx = -1;

        for (size_t i = 1; i < num_samples; ++i) {
            if (std::isnan(filtered_ranges[i]) || std::isinf(filtered_ranges[i])) {
                RCLCPP_INFO(this->get_logger(), "Infinite value detected");
                continue;
            }

            if(i*msg->angle_increment >= M_PI/2.0f && i*msg->angle_increment <= 3*M_PI/2.0f){ // limit search to -pi/2 to pi/2
                continue;
            }
            
            if (filtered_ranges[i] > max_dist) {
                max_dist = filtered_ranges[i];
                best_idx = i;
            }
        }

        // 5. Compute targeting direction

        if (best_idx != -1) {
            float target_angle = 0 + (best_idx * msg->angle_increment);
            
            // Clamp steering angle to car's hardware limits
            // target_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, static_cast<double>(target_angle)));
            
            target_angle_ = std::atan2(std::sin(target_angle), std::cos(target_angle)); // keep angle between -pi and pi
            forward_distance_ = filtered_ranges[best_idx];
            RCLCPP_INFO(this->get_logger(), "chosen angle: %f, chosen distance: %f", target_angle, filtered_ranges[best_idx]);
        } else {
            // Safety fallback: if no path is found, stop the vehicle
            target_angle_ = 0.0f;
            RCLCPP_WARN(this->get_logger(), "No safe paths detected! Emergency braking triggered.");
        }
    }

    double calculate_dynamic_speed(double forward_distance) {
        // Case 1: Obstacle is way too close. Stop!
        if (forward_distance <= MIN_SAFE_DIST_) {
            return 0.0;
        }
        
        // Case 2: Horizon is wide open. Full speed ahead!
        if (forward_distance >= MAX_SAFE_DIST_) {
            return max_speed_;
        }
        
        // Case 3: Segment 1 (Between Min and Mid Distance)
        if (forward_distance < MID_SAFE_DIST_) {
            double ratio = (forward_distance - MIN_SAFE_DIST_) / (MID_SAFE_DIST_ - MIN_SAFE_DIST_);
            return 0.0 + ratio * (MID_SPEED_ - 0.0);
        }
        
        // Case 4: Segment 2 (Between Mid and Max Distance)
        // (As the text points out: allows the car to carry speed through broad turns and drift smoothly!)
        double ratio = (forward_distance - MID_SAFE_DIST_) / (MAX_SAFE_DIST_ - MID_SAFE_DIST_);
        return MID_SPEED_ + ratio * (max_speed_ - MID_SPEED_);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityExtender>());
    rclcpp::shutdown();
    return 0;
}