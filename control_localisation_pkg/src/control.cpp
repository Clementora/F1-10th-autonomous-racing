#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;

namespace customController
{

class GapFollower : public rclcpp::Node
{
public:
    GapFollower(rclcpp::NodeOptions options) : Node("gap_follower_controller", options)
    {
        laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10, std::bind(&GapFollower::gap_follower_callback, this, _1));
            
        cmd_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/yellow_car/cmd_ackermann", 10);
    }   
  
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_publisher;

    // Control Constants
    const double MAX_STEER = 0.35;

    // --- TRACK-OPTIMIZED GAP FOLLOWER ---
    void gap_follower_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int fov_cells = M_PI / std::abs(msg->angle_increment); 
        int center_idx = msg->ranges.size() / 2;
        int start_idx = std::max(0, center_idx - fov_cells / 2);
        int end_idx = std::min((int)msg->ranges.size() - 1, center_idx + fov_cells / 2);

        std::vector<double> proc_ranges;
        double min_dist = 100.0;
        int min_idx = -1;
        
        // FIX 1: Max Lookahead to prevent Tunnel Vision
        double MAX_LOOKAHEAD = 3.5; 

        // 1. Data Sanitization & Lookahead Capping
        for (int i = start_idx; i <= end_idx; ++i) {
            double r = msg->ranges[i];
            
            if (std::isnan(r) || std::isinf(r) || r < 0.15) {
                r = MAX_LOOKAHEAD; 
            } else if (r > MAX_LOOKAHEAD) {
                r = MAX_LOOKAHEAD; // Flatten distant gaps
            }
            
            proc_ranges.push_back(r);
            int local_idx = proc_ranges.size() - 1;

            if (r < min_dist) {
                min_dist = r;
                min_idx = local_idx;
            }
        }

        // 2. Dynamic Safety Bubble (Scales based on how close the wall is)
        int bubble_radius = 0;
        if (min_dist > 0.05) {
            double safety_clearance = 0.45; // 45cm padding around the closest wall
            // Trigonometry to calculate exactly how many rays cover 45cm at this distance
            double angle_covered = std::asin(std::min(1.0, safety_clearance / min_dist));
            bubble_radius = static_cast<int>(angle_covered / std::abs(msg->angle_increment));
        }
        
        int bubble_start = std::max(0, min_idx - bubble_radius);
        int bubble_end = std::min((int)proc_ranges.size() - 1, min_idx + bubble_radius);
        
        for (int i = bubble_start; i <= bubble_end; ++i) {
            proc_ranges[i] = 0.0;
        }

        // 3. Find the Max Gap
        int current_start = -1, max_start = -1;
        int current_length = 0, max_length = 0;
        
        for (size_t i = 0; i < proc_ranges.size(); ++i) {
            if (proc_ranges[i] > 0.1) { 
                if (current_length == 0) current_start = i;
                current_length++;
            } else {
                if (current_length > max_length) {
                    max_length = current_length;
                    max_start = current_start;
                }
                current_length = 0;
            }
        }
        if (current_length > max_length) {
            max_start = current_start;
            max_length = current_length;
        }

        // 4. Calculate Steering Angle
        double delta_target = 0.0;

        if (max_length > 0 && max_start != -1) {
            // Find the maximum depth inside our chosen gap
            double max_depth = 0.0;
            for(int i = max_start; i < max_start + max_length; ++i) {
                if (proc_ranges[i] > max_depth) {
                    max_depth = proc_ranges[i];
                }
            }
            
            // FIX 2: Averaged Targeting
            long long sum_idx = 0;
            int count = 0;
            for(int i = max_start; i < max_start + max_length; ++i) {
                if (proc_ranges[i] >= max_depth - 0.2) { 
                    sum_idx += i;
                    count++;
                }
            }
            
            int best_idx = (count > 0) ? (sum_idx / count) : (max_start + max_length / 2);
            
            int original_idx = start_idx + best_idx;
            double target_angle = msg->angle_min + original_idx * msg->angle_increment;
            delta_target = std::max(-MAX_STEER, std::min(MAX_STEER, target_angle));
        }

        // 5. Dynamic Speed Control
        double speed = 1.0; 
        if (std::abs(delta_target) < 0.1) speed = 2.5; 
        else if (std::abs(delta_target) < 0.25) speed = 1.5; 
        else speed = 0.8; 
        
        ackermann_msgs::msg::AckermannDrive cmd_msg;
        cmd_msg.speed = speed; 
        cmd_msg.steering_angle = delta_target;
        cmd_publisher->publish(cmd_msg);
    }
};

} // namespace customController

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<customController::GapFollower>(options));
    rclcpp::shutdown();
    return 0;
}
