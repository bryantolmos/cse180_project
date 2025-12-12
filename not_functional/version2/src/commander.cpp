#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <vector>
#include <cstdlib>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

using namespace std::chrono_literals;

class WarehouseCommander : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseCommander() : Node("warehouse_commander")
    {
        // Parameter: scan topic
        this->declare_parameter<std::string>("scan_topic", "/scan");
        std::string scan_topic = this->get_parameter("scan_topic").as_string();

        // Publisher to set the initial AMCL pose
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        // Action Client (Nav2)
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&WarehouseCommander::map_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&WarehouseCommander::scan_callback, this, std::placeholders::_1));

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable(),
            std::bind(&WarehouseCommander::amcl_callback, this, std::placeholders::_1)
);


        // Marker visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Warehouse Commander initialized.");
    }

    // --------------------------------------------------------------------------
    // Callbacks
    // --------------------------------------------------------------------------
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = *msg;
        have_map_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = *msg;
        have_scan_ = true;
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
        have_pose_ = true;
    }

    // --------------------------------------------------------------------------
    // Wait for AMCL node to exist before publishing initial pose
    // --------------------------------------------------------------------------
    bool wait_for_amcl()
    {
    RCLCPP_INFO(this->get_logger(), "Waiting for AMCL to start publishing /amcl_pose...");

    while (rclcpp::ok() && !have_pose_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Still waiting for /amcl_pose...");
        std::this_thread::sleep_for(200ms);
    }

    RCLCPP_INFO(this->get_logger(), "AMCL is publishing. Continuing...");
    return true;
    }


    // --------------------------------------------------------------------------
    // Publish initial pose to AMCL
    // --------------------------------------------------------------------------
    void set_initial_position()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();

        msg.pose.pose.position.x = 2.12;
        msg.pose.pose.position.y = -21.3;

        tf2::Quaternion q;
        q.setRPY(0, 0, 1.57);
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance[0] = 0.01;
        msg.pose.covariance[7] = 0.01;
        msg.pose.covariance[35] = 0.01;

        for (int i = 0; i < 3; i++) {
            init_pose_pub_->publish(msg);
            std::this_thread::sleep_for(200ms);
        }

        RCLCPP_INFO(this->get_logger(), "Initial pose published to AMCL.");
    }

    // --------------------------------------------------------------------------
    // NAVIGATION
    // --------------------------------------------------------------------------
    void navigate_to(double x, double y, double yaw)
    {
        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 navigate_to_pose server not available!");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Navigating to (%.2f, %.2f)", x, y);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        auto handle_future = nav_client_->async_send_goal(goal, options);
        auto handle = handle_future.get();

        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2.");
            return;
        }

        auto result_future = nav_client_->async_get_result(handle);
        auto result = result_future.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached destination.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach destination.");
        }
    }

    // --------------------------------------------------------------------------
    // HUMAN LASER DETECTION (corrected)
    // --------------------------------------------------------------------------
    bool laser_confirms_human_present(double hx, double hy)
    {
        std::lock_guard<std::mutex> pose_lock(pose_mutex_);
        std::lock_guard<std::mutex> scan_lock(scan_mutex_);

        if (!have_scan_ || !have_pose_)
            return false;

        auto scan = latest_scan_;
        auto pose = latest_pose_;

        double rx = pose.pose.pose.position.x;
        double ry = pose.pose.pose.position.y;

        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Check laser hits near expected human location
        for (size_t i = 0; i < scan.ranges.size(); i++) {
            float r = scan.ranges[i];
            if (std::isnan(r) || r < scan.range_min || r > scan.range_max)
                continue;

            double angle = scan.angle_min + i * scan.angle_increment;
            double wx = rx + r * std::cos(yaw + angle);
            double wy = ry + r * std::sin(yaw + angle);

            if (std::hypot(wx - hx, wy - hy) < 0.45) {
                RCLCPP_INFO(this->get_logger(), "Laser CONFIRMS human present.");
                return true;
            }
        }

        RCLCPP_WARN(this->get_logger(), "Laser does NOT see human at expected position.");
        return false;
    }

    // --------------------------------------------------------------------------
    // CHECK IF HUMAN IS MISSING
    // --------------------------------------------------------------------------
    bool human_missing(double hx, double hy)
    {
        return !laser_confirms_human_present(hx, hy);
    }

    // --------------------------------------------------------------------------
    // LAWNMOWER SEARCH FOR HUMAN
    // --------------------------------------------------------------------------
    void search_for_human(int id, double cx, double cy)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Starting SEARCH for Human %d around (%.2f, %.2f)",
                    id, cx, cy);

        double width = 10.0;
        double height = 10.0;
        double spacing = 1.5;

        double start_x = cx - width / 2.0;
        double start_y = cy - height / 2.0;

        bool left_to_right = true;

        std::vector<std::pair<double, double>> waypoints;

        for (double y = start_y; y <= start_y + height; y += spacing) {
            if (left_to_right) {
                for (double x = start_x; x <= start_x + width; x += spacing)
                    waypoints.emplace_back(x, y);
            } else {
                for (double x = start_x + width; x >= start_x; x -= spacing)
                    waypoints.emplace_back(x, y);
            }
            left_to_right = !left_to_right;
        }

        for (auto &wp : waypoints) {
            navigate_to(wp.first, wp.second, 0.0);

            std::this_thread::sleep_for(400ms);

            if (laser_confirms_human_present(cx, cy)) {
                RCLCPP_ERROR(this->get_logger(),
                             "Human %d FOUND during search!", id);
                return;
            }
        }

        RCLCPP_ERROR(this->get_logger(),
                     "Human %d NOT found in search region.", id);
    }

    // --------------------------------------------------------------------------
    // MAIN MISSION FLOW
    // --------------------------------------------------------------------------
    void run_mission()
    {
        RCLCPP_INFO(this->get_logger(), "Mission starting...");

        // allow Gazebo & Nav2 to start
        std::this_thread::sleep_for(2s);

        wait_for_amcl();
        std::this_thread::sleep_for(300ms);

        set_initial_position();
        std::this_thread::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "Waiting for map/scan/pose topics...");

        while (rclcpp::ok() && !(have_map_ && have_scan_ && have_pose_)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Still waiting for required sensors...");
            std::this_thread::sleep_for(200ms);
        }

        RCLCPP_INFO(this->get_logger(), "All topics available. Beginning mission.");

        // HUMAN LOCATIONS
        double h1x = 0.98,  h1y = 0.16;
        double h2x = -12.0, h2y = 17.0;

        // -- HUMAN 1 --
        navigate_to(h1x, h1y, 0.0);
        if (human_missing(h1x, h1y))
            search_for_human(1, h1x, h1y);

        // -- HUMAN 2 --
        navigate_to(h2x, h2y, 1.57);
        if (human_missing(h2x, h2y))
            search_for_human(2, h2x, h2y);

        RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE.");
    }

private:
    // Publishers / clients
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    // Data and mutexes
    std::mutex map_mutex_, scan_mutex_, pose_mutex_;
    nav_msgs::msg::OccupancyGrid map_;
    sensor_msgs::msg::LaserScan latest_scan_;
    geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;

    bool have_map_ = false;
    bool have_scan_ = false;
    bool have_pose_ = false;
};

// --------------------------------------------------------------------------
// entry point
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseCommander>();

    // Start spinning first so callbacks begin immediately
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    // Small delay so subscriptions are ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Now it's safe to run the mission
    node->run_mission();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}


