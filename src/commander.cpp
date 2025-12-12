#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

struct DetectedHuman {
    double x;
    double y;
};

class WarehouseScanner : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseScanner() : Node("warehouse_scanner")
    {
        // 1. Init TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. Communications
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        // Publisher to set initial pose (crucial for AMCL)
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Subscribers
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", sensor_qos, std::bind(&WarehouseScanner::scan_callback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(), std::bind(&WarehouseScanner::map_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", sensor_qos, std::bind(&WarehouseScanner::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Warehouse Scanner Initialized.");
    }

    // --- 1. SET INITIAL POSE (Required for AMCL) ---
    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.pose.position.x = 2.12;
        msg.pose.pose.position.y = -21.3;
        // Yaw 1.57 (90 deg)
        msg.pose.pose.orientation.z = 0.707;
        msg.pose.pose.orientation.w = 0.707;
        msg.pose.covariance[0] = 0.01; // High confidence
        init_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Initial Pose Sent.");
    }

    // --- 2. BUILD COVERAGE PATH (The "Lawnmower") ---
    void build_coverage_path() {
        if (!latest_map_) return;
        
        RCLCPP_INFO(this->get_logger(), "Building Coverage Path...");
        coverage_path_.clear();

        int width = latest_map_->info.width;
        int height = latest_map_->info.height;
        double res = latest_map_->info.resolution;
        double ox = latest_map_->info.origin.position.x;
        double oy = latest_map_->info.origin.position.y;

        // Step size: Check a waypoint every 4.0 meters (Optimized for speed)
        int step_cells = (int)(4.0 / res); 

        // Iterate through map (Rows first, then columns)
        for (int y = 0; y < height; y += step_cells) {
            double wy = oy + y * res;
            
            // OPTIMIZATION: Don't search the back wall area where false positives live
            if (wy <= -24.0) continue;

            for (int x = 0; x < width; x += step_cells) {
                // Check if this spot is FREE space
                int idx = y * width + x;
                if (latest_map_->data[idx] == 0) { // 0 means completely free
                    
                    geometry_msgs::msg::PoseStamped wp;
                    wp.header.frame_id = "map";
                    wp.pose.position.x = ox + x * res;
                    wp.pose.position.y = wy;
                    wp.pose.orientation.w = 1.0;
                    
                    coverage_path_.push_back(wp);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Generated %zu search waypoints.", coverage_path_.size());
    }

// --- 3. EXECUTE SEARCH (Modified to Start at Center) ---
    void run_full_search() {
        // Wait for connections
        std::this_thread::sleep_for(2s);
        set_initial_pose();
        
        // Wait for Map
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (latest_map_) break;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for map...");
            std::this_thread::sleep_for(1s);
        }

        // --- STEP 1: PRIORITY SCAN (Center / Human 1 Location) ---
        // We go here FIRST to check the "Human 1" area immediately.
        RCLCPP_WARN(this->get_logger(), "!!! PRIORITY SCAN: GOING TO CENTER (2.0, 0.0) !!!");
        
        geometry_msgs::msg::PoseStamped center_goal;
        center_goal.header.frame_id = "map";
        center_goal.pose.position.x = 2.0; // Center / Human 1 X
        center_goal.pose.position.y = 0.0; // Center / Human 1 Y
        center_goal.pose.orientation.w = 1.0;
        
        navigate_to_pose(center_goal);
        
        // Wait 5 seconds to ensure we get a good scan of the area
        RCLCPP_INFO(this->get_logger(), "Scanning Center Area...");
        std::this_thread::sleep_for(5s);


        // --- STEP 2: RESUME FULL COVERAGE (Lawnmower) ---
        // Now we continue searching the rest of the warehouse in case we missed them.
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            build_coverage_path();
        }

        for (size_t i = 0; i < coverage_path_.size(); ++i) {
            if (!rclcpp::ok()) break;

            // Stop if we found 2 humans
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (detected_humans_.size() >= 2) {
                    RCLCPP_INFO(this->get_logger(), "Found both humans! Stopping search.");
                    break;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Navigating to Waypoint %zu/%zu...", i+1, coverage_path_.size());
            navigate_to_pose(coverage_path_[i]);
            std::this_thread::sleep_for(2s); 
        }
        
        RCLCPP_INFO(this->get_logger(), "Coverage Search Complete.");
    }

    // --- 4. DETECT GHOSTS (With Stronger Buffer) ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_map_ || !current_pose_) return;

        double rx = current_pose_->pose.pose.position.x;
        double ry = current_pose_->pose.pose.position.y;
        
        tf2::Quaternion q(
            current_pose_->pose.pose.orientation.x,
            current_pose_->pose.pose.orientation.y,
            current_pose_->pose.pose.orientation.z,
            current_pose_->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, robot_yaw;
        m.getRPY(roll, pitch, robot_yaw);

        for (size_t i = 0; i < msg->ranges.size(); i+=5) {
            float r = msg->ranges[i];
            
            // Filter 1: Distance Limit
            if (r > 2.5 || r < msg->range_min) continue;

            double angle = msg->angle_min + (i * msg->angle_increment) + robot_yaw;
            double wx = rx + r * std::cos(angle);
            double wy = ry + r * std::sin(angle);

            // Filter 2: Ignore Start Zone
            if (wy < -17.0) continue;

            double ox = latest_map_->info.origin.position.x;
            double oy = latest_map_->info.origin.position.y;
            double res = latest_map_->info.resolution;
            int width = latest_map_->info.width;
            int height = latest_map_->info.height;

            int mx = (int)((wx - ox) / res);
            int my = (int)((wy - oy) / res);

            // Bounds check with padding for the buffer
            if (mx < 2 || mx >= width - 2 || my < 2 || my >= height - 2) continue;

            // --- STRONG BUFFER: 5x5 NEIGHBOR CHECK ---
            // We check a radius of 2 pixels (approx 10-15cm)
            bool close_to_wall = false;
            int buffer_radius = 4; 

            for (int dy = -buffer_radius; dy <= buffer_radius; dy++) {
                for (int dx = -buffer_radius; dx <= buffer_radius; dx++) {
                    int idx = (my + dy) * width + (mx + dx);
                    
                    // If any neighbor is even slightly occupied (>10), treat as wall
                    if (latest_map_->data[idx] > 10) { 
                        close_to_wall = true;
                        break; 
                    }
                }
                if (close_to_wall) break;
            }

            if (close_to_wall) continue;

            // --- FINAL CHECK: Is it Free Space? ---
            int map_val = latest_map_->data[my * width + mx];
            if (map_val >= 0 && map_val < 5) { 
                
                bool is_new = true;
                for (const auto& human : detected_humans_) {
                    double dist = std::sqrt(std::pow(human.x - wx, 2) + std::pow(human.y - wy, 2));
                    if (dist < 1.0) { 
                        is_new = false;
                        break;
                    }
                }

                if (is_new) {
                    RCLCPP_WARN(this->get_logger(), "!!! FOUND HUMAN AT X: %.2f, Y: %.2f !!!", wx, wy);
                    detected_humans_.push_back({wx, wy});
                }
            }
        }
    }


    // --- Helpers ---
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_map_ = msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_ = msg;
    }

    void navigate_to_pose(const geometry_msgs::msg::PoseStamped &target) {
        if (!nav_client_->wait_for_action_server(1s)) return;
        
        auto goal = NavigateToPose::Goal();
        goal.pose = target;
        goal.pose.header.stamp = this->now();
        
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        auto future = nav_client_->async_send_goal(goal, options);
        
        auto handle = future.get();
        if (!handle) return;
        
        nav_client_->async_get_result(handle).get(); // Block until reached
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex data_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
    
    std::vector<geometry_msgs::msg::PoseStamped> coverage_path_;
    std::vector<DetectedHuman> detected_humans_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseScanner>();
    
    // Thread for navigation logic
    std::thread runner([node](){
        node->run_full_search();
    });

    rclcpp::spin(node);
    runner.join();
    rclcpp::shutdown();
    return 0;
}