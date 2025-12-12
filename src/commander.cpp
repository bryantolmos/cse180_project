#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class WarehouseCommander : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseCommander() : Node("warehouse_commander")
    {
        // 1. Publishers & Action Clients
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 2. Sensors Subscribers (Best Effort QoS for sensor data is often safer)
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", sensor_qos, std::bind(&WarehouseCommander::scan_callback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(), std::bind(&WarehouseCommander::map_callback, this, std::placeholders::_1)); // Map is latched

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", sensor_qos, std::bind(&WarehouseCommander::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Mission Commander Initialized.");
    }

    // --- CALLBACKS ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_scan_ = msg;
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_map_ = msg;
        RCLCPP_INFO(this->get_logger(), "Map received.");
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_ = msg;
    }

    // --- HELPER: Get Robot Yaw from Pose ---
    double get_yaw_from_pose(const geometry_msgs::msg::Pose &pose) {
        tf2::Quaternion q(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // --- REAL CHECK FUNCTION ---
    // Checks if a human exists at (target_x, target_y) within a tolerance radius
    bool check_human_missing(double target_x, double target_y) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!latest_scan_ || !current_pose_) {
            RCLCPP_WARN(this->get_logger(), "No sensor data to check human!");
            return false; // Assume present if sensors fail
        }

        double robot_x = current_pose_->pose.pose.position.x;
        double robot_y = current_pose_->pose.pose.position.y;
        double robot_yaw = get_yaw_from_pose(current_pose_->pose.pose);

        // Vector to Human
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double dist_to_human = std::sqrt(dx*dx + dy*dy);
        double angle_to_human = std::atan2(dy, dx); // World frame angle

        // Convert world angle to robot local scan frame
        double angle_in_scan = angle_to_human - robot_yaw;
        
        // Normalize to [-PI, PI]
        while (angle_in_scan > M_PI) angle_in_scan -= 2.0 * M_PI;
        while (angle_in_scan < -M_PI) angle_in_scan += 2.0 * M_PI;

        // Check rays in a small cone pointing at the human (+/- 10 degrees)
        int center_idx = (angle_in_scan - latest_scan_->angle_min) / latest_scan_->angle_increment;
        int width = (10.0 * (M_PI/180.0)) / latest_scan_->angle_increment; // 10 deg cone

        int hits = 0;
        int misses = 0;

        for (int i = center_idx - width; i <= center_idx + width; i++) {
            if (i < 0 || i >= (int)latest_scan_->ranges.size()) continue;

            float r = latest_scan_->ranges[i];
            
            // Logic: If the laser stops NEAR the human (within 0.5m), they are there.
            // If the laser goes WAY PAST the human, they are missing.
            if (std::abs(r - dist_to_human) < 0.5) {
                hits++;
            } else if (r > dist_to_human + 0.5) {
                misses++;
            }
        }

        // If we see mostly empty space behind where the human should be, they are missing.
        if (misses > hits) {
            RCLCPP_WARN(this->get_logger(), "Human at (%.2f, %.2f) is MISSING!", target_x, target_y);
            return true; 
        } else {
            RCLCPP_INFO(this->get_logger(), "Human at (%.2f, %.2f) is present.", target_x, target_y);
            return false;
        }
    }

    // --- REAL SEARCH FUNCTION ---
    // Scans current view for obstacles that exist in Reality but NOT in the Map
    void search_for_new_human_location() {
        std::this_thread::sleep_for(1s); // let scan stabilize
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!latest_scan_ || !current_pose_ || !latest_map_) {
            RCLCPP_WARN(this->get_logger(), "Missing data for search.");
            return;
        }

        double robot_x = current_pose_->pose.pose.position.x;
        double robot_y = current_pose_->pose.pose.position.y;
        double robot_yaw = get_yaw_from_pose(current_pose_->pose.pose);

        RCLCPP_INFO(this->get_logger(), "Scanning for new human location...");

        for (size_t i = 0; i < latest_scan_->ranges.size(); i+=5) { // Step by 5 to save CPU
            float r = latest_scan_->ranges[i];
            if (r > latest_scan_->range_max || r < latest_scan_->range_min) continue;

            // 1. Calculate Hit Point in World Coords
            double angle = latest_scan_->angle_min + (i * latest_scan_->angle_increment) + robot_yaw;
            double wx = robot_x + r * std::cos(angle);
            double wy = robot_y + r * std::sin(angle);

            // 2. Convert World Coords to Map Grid
            double map_origin_x = latest_map_->info.origin.position.x;
            double map_origin_y = latest_map_->info.origin.position.y;
            double res = latest_map_->info.resolution;
            int w = latest_map_->info.width;

            int mx = (int)((wx - map_origin_x) / res);
            int my = (int)((wy - map_origin_y) / res);

            // Bounds check
            if (mx < 0 || mx >= w || my < 0 || my >= (int)latest_map_->info.height) continue;

            // 3. Check Map Value
            int map_val = latest_map_->data[my * w + mx];

            // 4. DISCREPANCY DETECTED: Map says FREE (0), Laser says OBSTACLE
            // We use a threshold (e.g., < 50) because map probability is 0-100
            if (map_val >= 0 && map_val < 50) {
                // We found something where there should be nothing!
                RCLCPP_INFO(this->get_logger(), "FOUND MOVED HUMAN AT: X: %.2f, Y: %.2f", wx, wy);
                
                // For a real robust solution, you would cluster these points to find the center.
                // For now, returning the first solid hit is a good proof of concept.
                return; 
            }
        }
        RCLCPP_INFO(this->get_logger(), "No new humans found in current view.");
    }

    void set_initial_position() {
        // ... (Keep your existing code here) ...
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
        message.header.frame_id = "map";
        message.header.stamp = this->get_clock()->now();
        message.pose.pose.position.x = 2.12;
        message.pose.pose.position.y = -21.3;
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.57); 
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();
        message.pose.covariance[0] = 0.01;
        init_pose_pub_->publish(message);
        std::this_thread::sleep_for(1s); // Give AMCL time to update
    }

    void navigate_to(double x, double y, double yaw) {
        // ... (Keep your existing code here) ...
        // Ensure you reuse your existing navigation logic
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
            return;
        }
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        
        // Block until goal is accepted
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return;

        // Block until goal is done
        auto result_future = nav_client_->async_get_result(goal_handle);
        auto result = result_future.get(); // This blocks the thread
    }

    void run_mission()
    {
        // Wait for sensors to warm up
        std::this_thread::sleep_for(2s); 
        
        set_initial_position();
        
        // Wait for Map
        while(rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if(latest_map_) break;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for Map...");
            std::this_thread::sleep_for(1s);
        }

        // Coordinates of where humans SHOULD be
        double h1_x = 0.98, h1_y = 0.16;
        double h2_x = -12.0, h2_y = 17.0;

        // 1. Go to Human 1
        navigate_to(h1_x - 1.5, h1_y, 0.0); // Stop 1.5m before, facing them
        
        if (check_human_missing(h1_x, h1_y)) {
            // Spin around to find them
             RCLCPP_INFO(this->get_logger(), "Scanning for Human 1 new location...");
             // Implement a rotate here or just scan current view
             search_for_new_human_location();
        }

        // 2. Go to Human 2
        navigate_to(h2_x, h2_y - 2.0, 1.57); // Stop below, facing up
        
        if (check_human_missing(h2_x, h2_y)) {
             RCLCPP_INFO(this->get_logger(), "Scanning for Human 2 new location...");
             search_for_new_human_location();
        }

        RCLCPP_INFO(this->get_logger(), "Mission Complete.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // Data Storage (protected by mutex)
    std::mutex data_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseCommander>();
    
    // We run the mission in a separate thread so the callbacks (spin) can keep updating data
    std::thread mission_thread([node](){
        node->run_mission();
    });

    rclcpp::spin(node);
    mission_thread.join();
    rclcpp::shutdown();
    return 0;
}