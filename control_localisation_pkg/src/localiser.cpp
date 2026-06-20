#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp> // Changed from PoseStamped
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

using std::placeholders::_1;

namespace customLocaliser
{

class Localiser : public rclcpp::Node
{
public:
    Localiser(rclcpp::NodeOptions options) : Node("localiser", options)
    {
        my_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Localiser::odom_callback, this, _1));
            
        // Changed to Odometry publisher
        my_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/localisation", 10);
    }   
  
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr my_subscription;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr my_publisher;

    // URDF Constants
    const double WHEEL_RADIUS = 0.04;  
    const double WHEEL_BASE = 0.257;   
    
    // Odometry tracking
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0, v_ = 0.0;
    bool initialized_ = false;
    double prev_rl_pos_ = 0.0, prev_rr_pos_ = 0.0;
    rclcpp::Time last_time_;
    
    void odom_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        int rl_idx = -1, rr_idx = -1, fl_steer_idx = -1, fr_steer_idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "rear_left_wheel_joint") rl_idx = i;
            else if (msg->name[i] == "rear_right_wheel_joint") rr_idx = i;
            else if (msg->name[i] == "front_left_steering_joint") fl_steer_idx = i;
            else if (msg->name[i] == "front_right_steering_joint") fr_steer_idx = i;
        }

        if (rl_idx == -1 || rr_idx == -1 || fl_steer_idx == -1 || fr_steer_idx == -1) return;

        double current_rl = msg->position[rl_idx];
        double current_rr = msg->position[rr_idx];
        double delta_steer = (msg->position[fl_steer_idx] + msg->position[fr_steer_idx]) / 2.0;
        rclcpp::Time current_time = msg->header.stamp;

        if (!initialized_) {
            prev_rl_pos_ = current_rl;
            prev_rr_pos_ = current_rr;
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) dt = 0.01; 

        double d_left = (current_rl - prev_rl_pos_) * WHEEL_RADIUS;
        double d_right = (current_rr - prev_rr_pos_) * WHEEL_RADIUS;
        double d_center = (d_left + d_right) / 2.0;

        v_ = d_center / dt;
        prev_rl_pos_ = current_rl;
        prev_rr_pos_ = current_rr;
        last_time_ = current_time;

        double d_theta = (d_center * std::tan(delta_steer)) / WHEEL_BASE;
        x_ += d_center * std::cos(theta_ + d_theta / 2.0);
        y_ += d_center * std::sin(theta_ + d_theta / 2.0);
        theta_ += d_theta;
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

        // --- NEW ODOMETRY MESSAGE SETUP ---
        nav_msgs::msg::Odometry out_msg;
        out_msg.header.stamp = current_time;
        out_msg.header.frame_id = "odom";       // The fixed starting point
        out_msg.child_frame_id = "base_link";   // The car itself

        // Position
        out_msg.pose.pose.position.x = x_;
        out_msg.pose.pose.position.y = y_;
        out_msg.pose.pose.position.z = 0.0;
        out_msg.pose.pose.orientation.x = 0.0;
        out_msg.pose.pose.orientation.y = 0.0;
        out_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        out_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

        // Velocity
        out_msg.twist.twist.linear.x = v_;
        out_msg.twist.twist.angular.z = d_theta / dt;

        // Base Covariance (Tell the EKF we trust the wheels, but they aren't perfect)
        out_msg.pose.covariance[0] = 0.01;   // X
        out_msg.pose.covariance[7] = 0.01;   // Y
        out_msg.pose.covariance[35] = 0.05;  // Yaw
        
        out_msg.twist.covariance[0] = 0.01;  // Linear velocity
        out_msg.twist.covariance[35] = 0.05; // Angular velocity

        my_publisher->publish(out_msg);
    }
};

} // namespace customLocaliser

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<customLocaliser::Localiser>(options));
    rclcpp::shutdown();
    return 0;
}