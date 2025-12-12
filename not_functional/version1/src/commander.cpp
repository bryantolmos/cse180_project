#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <vector>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;

class WarehouseCommander : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseCommander() : Node("warehouse_commander")
    {
        // Publisher to set the initial pose for AMCL (same as before)
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        // Action Client for Nav2 navigate_to_pose
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Subscriptions to map, laser scan and amcl_pose
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&WarehouseCommander::map_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&WarehouseCommander::scan_callback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&WarehouseCommander::amcl_callback, this, std::placeholders::_1));

        // Marker publisher for visualizing search waypoints and current target in RViz
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Mission Commander Initialized.");
        std::srand(unsigned(std::time(nullptr))); // seed for fallback randomness
    }

    // -----------------------------
    // Map / scan / pose callbacks
    // -----------------------------
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg; // store latest map
        have_map_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = *msg; // store latest scan
        have_scan_ = true;
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg; // store latest estimated robot pose
        have_pose_ = true;
    }

    // -----------------------------
    // Helper: Publish initial pose for AMCL
    // -----------------------------
    void set_initial_position()
    {
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
        message.header.frame_id = "map";
        message.header.stamp = this->get_clock()->now();

        // Provided initial pose values from your note
        message.pose.pose.position.x = 2.12;
        message.pose.pose.position.y = -21.3;

        tf2::Quaternion q;
        q.setRPY(0, 0, 1.57);
        message.pose.pose.orientation = tf2::toMsg(q);

        // small covariance to help localization converge
        message.pose.covariance[0] = 0.01;
        message.pose.covariance[7] = 0.01;
        message.pose.covariance[35] = 0.01;

        // publish a few times to ensure AMCL receives initial pose
        for(int i = 0; i < 3; i++) {
            init_pose_pub_->publish(message);
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_INFO(this->get_logger(), "Initial Pose Set.");
    }

    // -----------------------------
    // Navigation helper (unchanged, with logging)
    // -----------------------------
    void navigate_to(double x, double y, double yaw)
    {
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
        goal_msg.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Navigating to: (%.2f, %.2f)...", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // send goal synchronously (blocking)
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        auto goal_handle = goal_handle_future.get();

        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        auto result_future = nav_client_->async_get_result(goal_handle);
        auto result = result_future.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached destination.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach destination.");
        }
    }

    // -----------------------------
    // Check whether a human is missing (uses /map occupancy grid)
    // -----------------------------
    // Return true if human is missing at the expected location (i.e. map cell is free)
    bool check_human_missing(int human_id, double expected_x, double expected_y)
    {
        RCLCPP_INFO(this->get_logger(), "Checking map occupancy for Human %d at (%.2f, %.2f)...",
                    human_id, expected_x, expected_y);

        // Ensure we have a map to check
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!have_map_) {
                RCLCPP_WARN(this->get_logger(), "No /map available yet; assuming human missing for safety.");
                return true; // if we don't have a map, better to trigger search
            }
        }

        // Convert world coordinates to map cell index
        int idx = world_to_map_index(expected_x, expected_y);
        if (idx < 0) {
            RCLCPP_WARN(this->get_logger(), "Expected coordinates are outside map bounds; assuming missing.");
            return true;
        }

        int8_t val;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            val = map_.data[idx]; // occupancy value [-1 unknown, 0 free, 100 occupied]
        }

        // If map cell is marked occupied (>50), we consider human present at expected location
        if (val > 50) {
            RCLCPP_INFO(this->get_logger(), "Map shows occupancy at human location (value=%d) -> present.", val);
            return false; // not missing
        } else {
            RCLCPP_WARN(this->get_logger(), "Map does not show occupancy at human location (value=%d) -> missing.", val);
            return true; // missing: trigger search
        }
    }

    // -----------------------------
    // Scan for human using laser vs map differencing
    // Called at each waypoint; returns true if human detected
    // -----------------------------
    bool scan_for_human(int human_id)
    {
        RCLCPP_INFO(this->get_logger(), "Performing laser-based scan for Human %d...", human_id);

        // Need scan, map, and robot pose
        {
            std::lock_guard<std::mutex> lock1(scan_mutex_);
            std::lock_guard<std::mutex> lock2(map_mutex_);
            std::lock_guard<std::mutex> lock3(pose_mutex_);

            if (!have_scan_ || !have_map_ || !have_pose_) {
                RCLCPP_WARN(this->get_logger(), "Missing sensors/map/pose — cannot scan reliably.");
                return false; // do not claim detection if data is missing
            }
        }

        // Copy locally for processing to avoid holding locks while computing
        sensor_msgs::msg::LaserScan scan;
        nav_msgs::msg::OccupancyGrid map;
        geometry_msgs::msg::PoseWithCovarianceStamped robot_pose;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan = latest_scan_;
        }
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            map = map_;
        }
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            robot_pose = latest_pose_;
        }

        // Extract robot yaw from AMCL pose
        double robot_x = robot_pose.pose.pose.position.x;
        double robot_y = robot_pose.pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(robot_pose.pose.pose.orientation, q);
        double robot_roll, robot_pitch, robot_yaw;
        tf2::Matrix3x3(q).getRPY(robot_roll, robot_pitch, robot_yaw);

        // We'll count beam endpoints that belong to newly observed obstacles (not in static map).
        int new_obstacle_hits = 0;
        int total_checked = 0;

        // For each laser reading, compute the endpoint in map coordinates and compare with map occupancy
        double angle = scan.angle_min;
        for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
            float r = scan.ranges[i];

            // Ignore invalid or out-of-range readings
            if (std::isnan(r) || r < scan.range_min || r > scan.range_max) continue;

            // Compute beam endpoint in world frame (map)
            double beam_angle_world = robot_yaw + angle;
            double ex = robot_x + r * std::cos(beam_angle_world);
            double ey = robot_y + r * std::sin(beam_angle_world);

            int idx = world_to_map_index_in_map(map, ex, ey);
            if (idx < 0) continue; // outside map

            total_checked++;

            int8_t cell_val = map.data[idx];

            // If map cell is free (<=50 and >=0) but laser sees an obstacle here -> new object
            if (cell_val >= 0 && cell_val <= 50) {
                // mark as new obstacle hit
                new_obstacle_hits++;
            }

            // Heuristic: if enough beams indicate new obstacle, we declare detection
            if (new_obstacle_hits >= detection_beam_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Detected potential moved object (human) — hits=%d, checked=%d",
                            new_obstacle_hits, total_checked);
                return true;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Scan complete: new_obstacle_hits=%d out of %d beams", new_obstacle_hits, total_checked);
        return false;
    }

    // -----------------------------
    // Search for human using lawnmower sweep
    // Moves through waypoints and calls scan_for_human() at each step.
    // Visualizes waypoints in RViz via markers.
    // -----------------------------
    void search_for_human(int human_id, double center_x, double center_y)
    {
        RCLCPP_WARN(this->get_logger(), "Initiating SEARCH for Human %d around (%.2f, %.2f)", human_id, center_x, center_y);

        // configure lawnmower parameters (tweak if needed)
        double search_width = 10.0;   // meters (x dimension)
        double search_height = 10.0;  // meters (y dimension)
        double spacing = 1.5;         // meters between passes
        double yaw = 0.0;             // heading for navigation (facing forward)

        // create waypoints in a lawnmower pattern centered at (center_x, center_y)
        std::vector<std::pair<double,double>> waypoints;

        double start_x = center_x - search_width / 2.0;
        double start_y = center_y - search_height / 2.0;

        bool left_to_right = true;
        for (double y = start_y; y <= start_y + search_height; y += spacing) {
            if (left_to_right) {
                for (double x = start_x; x <= start_x + search_width; x += spacing) {
                    waypoints.emplace_back(x, y);
                }
            } else {
                for (double x = start_x + search_width; x >= start_x; x -= spacing) {
                    waypoints.emplace_back(x, y);
                }
            }
            left_to_right = !left_to_right;
        }

        // Publish visualization markers for all waypoints (so you can see path in RViz)
        publish_waypoint_markers(waypoints, human_id);

        // Iterate waypoints and search
        for (size_t i = 0; i < waypoints.size(); ++i) {
            double wx = waypoints[i].first;
            double wy = waypoints[i].second;

            // Publish a "current target" marker for visibility
            publish_current_target_marker(wx, wy, i);

            // Move to waypoint
            navigate_to(wx, wy, yaw);

            // After arrival (or attempt), perform scan_for_human
            if (scan_for_human(human_id)) {
                RCLCPP_WARN(this->get_logger(), "Human %d FOUND during search at waypoint (%.2f, %.2f)!", human_id, wx, wy);
                // Optionally navigate to the exact detected location (not implemented — we detected via laser)
                return;
            }

            // small delay to allow fresh sensor data
            std::this_thread::sleep_for(400ms);
        }

        RCLCPP_WARN(this->get_logger(), "Search complete: Human %d NOT FOUND in search area.", human_id);
    }

    // -----------------------------
    // Visualization helpers (RViz markers)
    // -----------------------------
    void publish_waypoint_markers(const std::vector<std::pair<double,double>>& waypoints, int human_id)
    {
        // Publish a set of small sphere markers showing each waypoint
        for (size_t i = 0; i < waypoints.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->get_clock()->now();
            m.ns = "search_waypoints_human_" + std::to_string(human_id);
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = waypoints[i].first;
            m.pose.position.y = waypoints[i].second;
            m.pose.position.z = 0.2;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.15;
            m.scale.y = 0.15;
            m.scale.z = 0.15;
            // Color: green-ish for planned waypoints
            m.color.r = 0.0f;
            m.color.g = 0.9f;
            m.color.b = 0.1f;
            m.color.a = 0.8f;
            m.lifetime = rclcpp::Duration::from_seconds(60.0);
            marker_pub_->publish(m);
            // slight sleep so RViz ingests markers in order (not required but keeps RViz responsive)
            std::this_thread::sleep_for(10ms);
        }
    }

    void publish_current_target_marker(double x, double y, int id)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->get_clock()->now();
        m.ns = "search_current_target";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.25;
        m.scale.y = 0.25;
        m.scale.z = 0.1;
        // Color: red-ish for current target
        m.color.r = 0.9f;
        m.color.g = 0.1f;
        m.color.b = 0.1f;
        m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration::from_seconds(10.0);
        marker_pub_->publish(m);
    }

    // -----------------------------
    // Utility: convert world coords to map index using stored map_
    // Returns -1 if outside bounds
    // -----------------------------
    int world_to_map_index(double wx, double wy)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return world_to_map_index_in_map(map_, wx, wy);
    }

    // Non-locking variant (take map as parameter)
    int world_to_map_index_in_map(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy)
    {
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        double resolution = map.info.resolution;

        // compute indices from world coordinates
        int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        int my = static_cast<int>(std::floor((wy - origin_y) / resolution));

        if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) || my >= static_cast<int>(map.info.height)) {
            return -1; // outside bounds
        }

        return my * map.info.width + mx;
    }

    // -----------------------------
    // Main mission flow (uses real check + search)
    // -----------------------------
    void run_mission()
    {
        // 1. Initialize Localization
        set_initial_position();
        std::this_thread::sleep_for(2s);

        // 2. Define where the humans are (values from rviz/map)
        double h1_inspect_x = 0.98;
        double h1_inspect_y = 0.16;

        double h2_inspect_x = -12.0;
        double h2_inspect_y = 17.0;

        // Wait until map and sensors are available (give some time to start)
        RCLCPP_INFO(this->get_logger(), "Waiting for /map, /scan, and /amcl_pose...");
        int tries = 0;
        while (tries < 60 && !(have_map_ && have_scan_ && have_pose_)) {
            std::this_thread::sleep_for(500ms);
            tries++;
        }
        if (!(have_map_ && have_scan_ && have_pose_)) {
            RCLCPP_WARN(this->get_logger(), "Sensors not fully available; mission will still proceed but results may be unreliable.");
        }

        // --- Check Human 1 ---
        navigate_to(h1_inspect_x, h1_inspect_y, 0.0);
        if (check_human_missing(1, h1_inspect_x, h1_inspect_y)) {
            // If missing → search around that area
            search_for_human(1, h1_inspect_x, h1_inspect_y);
        }

        // --- Check Human 2 ---
        navigate_to(h2_inspect_x, h2_inspect_y, 1.57);
        if (check_human_missing(2, h2_inspect_x, h2_inspect_y)) {
            search_for_human(2, h2_inspect_x, h2_inspect_y);
        }

        RCLCPP_INFO(this->get_logger(), "Mission Complete.");
    }

private:
    // Publishers / subscribers / clients
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Latest sensor/topic data with guards
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid map_;
    bool have_map_ = false;

    std::mutex scan_mutex_;
    sensor_msgs::msg::LaserScan latest_scan_;
    bool have_scan_ = false;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
    bool have_pose_ = false;

    // detection tuning param: how many beams indicating new obstacle to trigger detection
    const int detection_beam_threshold_ = 6;

};

// -----------------------------
// Entry point: create node and run mission in separate thread
// -----------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseCommander>();

    // Run mission in separate thread so rclcpp::spin can process callbacks
    std::thread mission_thread([node](){
        node->run_mission();
    });

    rclcpp::spin(node);
    mission_thread.join();
    rclcpp::shutdown();
    return 0;
}
