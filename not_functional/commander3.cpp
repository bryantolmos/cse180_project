#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>
#include <tuple> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h" // Required for tf2::Transform math object

using namespace std::chrono_literals;

// --- UTILITY: CLUSTER STRUCT ---
struct Cluster {
    std::vector<std::pair<double, double>> points;
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;

    void add_point(double x, double y) {
        points.push_back({x, y});
        if(x < min_x) min_x = x;
        if(x > max_x) max_x = x;
        if(y < min_y) min_y = y;
        if(y > max_y) max_y = y;
    }

    double get_width() const {
        return std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));
    }
    
    std::pair<double, double> get_centroid() const {
        double sum_x = 0, sum_y = 0;
        if (points.empty()) return {0.0, 0.0};
        for(auto &p : points) { sum_x += p.first; sum_y += p.second; }
        return {sum_x / points.size(), sum_y / points.size()};
    }
};

class WarehouseCommander : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseCommander() : Node("warehouse_commander")
    {
        // 1. TF2 Setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. Communication Interface
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // For spinning
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 3. Sensors
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", sensor_qos, std::bind(&WarehouseCommander::scan_callback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(), std::bind(&WarehouseCommander::map_callback, this, std::placeholders::_1));

        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", sensor_qos, std::bind(&WarehouseCommander::local_costmap_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", sensor_qos, std::bind(&WarehouseCommander::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Mission Commander Initialized.");
    }

    // --- SENSOR CALLBACKS ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_scan_ = msg;
    }
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_map_ = msg;
    }
    void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_local_costmap_ = msg;
    }
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_ = msg;
    }

    // --- CORE LOGIC 1: TARGET CHECK/ROTATION ---

    // Returns: 
    //   > 0.0: Angle needed to turn (Target is out of FOV).
    //   0.0:   Target is present (in FOV and verified).
    //   -1.0:  Target is confirmed missing.
    double get_target_angle_and_check(double target_x, double target_y) {
        RCLCPP_INFO(this->get_logger(), "Checking for human at (%.2f, %.2f)...", target_x, target_y);
        
        sensor_msgs::msg::LaserScan::SharedPtr current_scan;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_scan = latest_scan_;
        }

        if (!current_scan) {
            RCLCPP_WARN(this->get_logger(), "LaserScan data is missing. Assuming missing to trigger search.");
            return -1.0; 
        }
        
        const std::string laser_frame_id = current_scan->header.frame_id;

        // 1. Get Transform (Map -> Laser Frame)
        geometry_msgs::msg::TransformStamped t_laser_to_map;
        try {
            t_laser_to_map = tf_buffer_->lookupTransform(
                "map", laser_frame_id, rclcpp::Time(0), std::chrono::seconds(1));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: Could not transform target point to laser frame (%s). Assuming missing to trigger search.", ex.what());
            return -1.0; 
        }

        // 2. Get the inverse transform: Map -> Laser Frame
        tf2::Transform tf2_laser_to_map;
        tf2::fromMsg(t_laser_to_map.transform, tf2_laser_to_map); 

        tf2::Transform tf2_map_to_laser = tf2_laser_to_map.inverse(); 

        geometry_msgs::msg::TransformStamped t_map_to_laser;
        t_map_to_laser.header.stamp = t_laser_to_map.header.stamp;
        t_map_to_laser.header.frame_id = "map"; 
        t_map_to_laser.child_frame_id = laser_frame_id; 
        tf2::toMsg(tf2_map_to_laser, t_map_to_laser.transform); 


        // 3. Transform Target Point to Laser Frame
        geometry_msgs::msg::PointStamped target_map, target_local;
        target_map.header.frame_id = "map";
        target_map.point.x = target_x;
        target_map.point.y = target_y;
        target_map.point.z = 0.0;
        
        tf2::doTransform(target_map, target_local, t_map_to_laser);

        // 4. Calculate Angle and Distance in Local Frame
        double dx = target_local.point.x;
        double dy = target_local.point.y;
        double dist_expected = std::sqrt(dx*dx + dy*dy);
        double angle_expected = std::atan2(dy, dx);
        
        // Safety check: if distance is very short (robot inside expected spot), assume present.
        if (dist_expected < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Robot is too close to target, assuming presence.");
            return 0.0;
        }

        // 5. Check FOV and request rotation if needed
        if (angle_expected < current_scan->angle_min || angle_expected > current_scan->angle_max) {
            RCLCPP_WARN(this->get_logger(), "Target is out of FOV. Needs rotation of %.2f rad.", angle_expected);
            return angle_expected; // Return the angle needed to turn
        }
        
        // 6. Run the actual check (Target is in FOV)
        int index = (angle_expected - current_scan->angle_min) / current_scan->angle_increment;
        
        int hits = 0;
        int misses = 0;
        int window = 5; 

        for (int i = index - window; i <= index + window; i++) {
            if (i < 0 || i >= (int)current_scan->ranges.size()) continue;

            float r = current_scan->ranges[i];
            
            if (r > (dist_expected + 0.5)) {
                misses++;
            } 
            else if (std::abs(r - dist_expected) < 0.4) { 
                hits++;
            }
        }

        if (misses > hits) {
            RCLCPP_WARN(this->get_logger(), "Target Empty! Laser sees past expected location.");
            return -1.0; // Human Missing
        }
        
        RCLCPP_INFO(this->get_logger(), "Target Present. Laser hit obstacle at expected dist.");
        return 0.0; // Human Present
    }

    // Private helper function to rotate the robot
    void rotate_by_angle(double angle_rad) {
        if (std::abs(angle_rad) < 0.05) return; 

        RCLCPP_INFO(this->get_logger(), "Rotating by %.2f radians...", angle_rad);

        auto twist = geometry_msgs::msg::Twist();
        const double angular_speed = 0.4; 
        
        twist.angular.z = (angle_rad > 0) ? angular_speed : -angular_speed;
        double duration_sec = std::abs(angle_rad) / angular_speed;
        int iterations = static_cast<int>(duration_sec * 20.0); 

        for (int i = 0; i < iterations; ++i) {
            vel_pub_->publish(twist);
            std::this_thread::sleep_for(50ms); 
        }
        
        twist.angular.z = 0.0;
        vel_pub_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Rotation complete.");
        std::this_thread::sleep_for(500ms); 
    }

    // --- CORE LOGIC 2: NEW SEARCH METHODS ---
    
    // Lawnmower Search Pattern
    void perform_lawnmower_search() {
        RCLCPP_INFO(this->get_logger(), "--- COMMENCING LAWNMOWER SEARCH ---");
        
        // Define Search Box (Adjust these boundaries to fit your map)
        const double X_MIN = -6.0;
        const double X_MAX = 6.0;
        const double Y_MIN = 0.0;
        const double Y_MAX = 6.0;
        const double LANE_WIDTH = 3.0; // Ensure Lidar coverage overlaps

        double current_y = Y_MIN;
        bool sweep_right = true;
        
        // 1. Initial position/yaw (start at corner, facing the direction of sweep)
        navigate_to(X_MIN, Y_MIN, 0.0); // Start at bottom left, facing +X

        while (current_y <= Y_MAX) {
            double target_x = sweep_right ? X_MAX : X_MIN;
            double target_yaw = sweep_right ? 0.0 : M_PI;

            // 2. Sweep across the lane
            RCLCPP_INFO(this->get_logger(), "Searching lane at Y=%.2f. Navigating to (%.2f, %.2f).", current_y, target_x, current_y);
            navigate_to(target_x, current_y, target_yaw); 
            
            // CHECK 1: After sweeping the lane, check for discrepancies
            search_map_discrepancy();

            // Check termination condition early
            if (current_y + LANE_WIDTH > Y_MAX) break;

            // 3. Move to the next lane
            current_y += LANE_WIDTH;
            target_yaw = M_PI_2; // Yaw 90 degrees (+Y direction)

            RCLCPP_INFO(this->get_logger(), "Moving to next lane at Y=%.2f.", current_y);
            // Navigate to the start of the next line, maintaining the current X (X_MAX or X_MIN)
            navigate_to(target_x, current_y, target_yaw);

            // CHECK 2: After changing lanes, check again (new view angle)
            search_map_discrepancy();
            
            // 4. Reverse sweep direction
            sweep_right = !sweep_right;
        }

        RCLCPP_INFO(this->get_logger(), "--- LAWNMOWER SEARCH COMPLETE ---");
    }

    void search_map_discrepancy() {
        nav_msgs::msg::OccupancyGrid local_map;
        nav_msgs::msg::OccupancyGrid global_map;
        
        // Data fetching and setup (omitted for brevity - remains the same)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!latest_local_costmap_ || !latest_map_) return;
            local_map = *latest_local_costmap_;
            global_map = *latest_map_;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "map", local_map.header.frame_id, rclcpp::Time(0), std::chrono::seconds(1));
        } catch (tf2::TransformException &ex) { return; }

        std::vector<Cluster> clusters;
        Cluster current_cluster;

        int w = local_map.info.width;
        int h = local_map.info.height;
        double res = local_map.info.resolution;

        for (int y = 0; y < h; y+=2) { 
            for (int x = 0; x < w; x+=2) {
                // Check for LETHAL OBSTACLE in local costmap
                if (local_map.data[y * w + x] > 90) { 
                    
                    // Transform to World
                    double lx = local_map.info.origin.position.x + (x * res);
                    double ly = local_map.info.origin.position.y + (y * res);
                    geometry_msgs::msg::PointStamped pt_loc, pt_world;
                    pt_loc.point.x = lx; pt_loc.point.y = ly; 
                    pt_loc.header.frame_id = local_map.header.frame_id;
                    tf2::doTransform(pt_loc, pt_world, transform);

                    // Convert World Coords to Global Map Grid
                    int mx = (pt_world.point.x - global_map.info.origin.position.x) / global_map.info.resolution;
                    int my = (pt_world.point.y - global_map.info.origin.position.y) / global_map.info.resolution;

                    if (mx >= 0 && mx < (int)global_map.info.width && 
                        my >= 0 && my < (int)global_map.info.height) {
                        
                        // Discrepancy check: Global Map says FREE (0)
                        if (global_map.data[my*global_map.info.width + mx] == 0) {
                            if (current_cluster.points.empty()) {
                                current_cluster.add_point(pt_world.point.x, pt_world.point.y);
                            } else {
                                auto last = current_cluster.points.back();
                                double d = std::hypot(pt_world.point.x - last.first, pt_world.point.y - last.second);
                                // Clustering tolerance: 0.3m is reasonable
                                if (d < 0.3) current_cluster.add_point(pt_world.point.x, pt_world.point.y);
                                else {
                                    clusters.push_back(current_cluster);
                                    current_cluster = Cluster();
                                    current_cluster.add_point(pt_world.point.x, pt_world.point.y);
                                }
                            }
                        }
                    }
                }
            }
        }
        if(!current_cluster.points.empty()) clusters.push_back(current_cluster);

        for(auto &c : clusters) {
            double width = c.get_width();
            // **CORRECTED FILTER RANGE**
            if (width > 0.2 && width < 0.7) { 
                auto center = c.get_centroid();
                RCLCPP_INFO(this->get_logger(), "!!! FOUND RELOCATED OBJECT/HUMAN AT (%.2f, %.2f) !!!", center.first, center.second);
            }
        }
    }


    // --- INITIALIZATION & NAVIGATION ---
    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.pose.position.x = 2.12;
        msg.pose.pose.position.y = -21.3;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.57); 
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance[0] = 0.25; 
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.06;

        for(int i=0; i<3; i++) {
            init_pose_pub_->publish(msg);
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_INFO(this->get_logger(), "Initial Pose Set: (2.12, -21.3) Yaw: 1.57");
    }

    void navigate_to(double x, double y, double yaw) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server missing!");
            return;
        }
        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->get_clock()->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation = tf2::toMsg(q);

        auto future = nav_client_->async_send_goal(goal);
        auto handle = future.get();
        if (!handle) return;
        nav_client_->async_get_result(handle).get();
    }

    // --- MAIN EXECUTION ---
    void run_mission() {
        std::this_thread::sleep_for(3s); 
        set_initial_pose(); 
        std::this_thread::sleep_for(2s); 

        // Wait for all required data
        while(rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if(latest_map_ && latest_local_costmap_ && latest_scan_ && current_pose_) break;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for Maps, LaserScan, and AMCL Pose...");
            std::this_thread::sleep_for(1s);
        }

        // List of humans to check (X, Y)
        std::vector<std::pair<double, double>> targets = {
            {0.98, 0.16},  // Human 1
            {-12.0, 17.0}  // Human 2
        };

        for (const auto& target : targets) {
            double tx = target.first;
            double ty = target.second;

            // 1. Go to target (stop 2m short)
            double current_x = current_pose_->pose.pose.position.x;
            double current_y = current_pose_->pose.pose.position.y;
            double approach_yaw = std::atan2(ty - current_y, tx - current_x);
            navigate_to(tx - 2.0, ty, approach_yaw); 

            // 2. Loop until checked or confirmed missing
            double check_result = -2.0; 
            bool human_was_missing = false;
            
            while (rclcpp::ok()) {
                check_result = get_target_angle_and_check(tx, ty); 

                if (check_result > 0.0) {
                    // Case 1: Out of FOV - Rotate to face target
                    rotate_by_angle(check_result);
                } else if (check_result == 0.0) {
                    // Case 2: Present (or too close) - Break loop.
                    RCLCPP_INFO(this->get_logger(), "Human is present or verified.");
                    break;
                } else if (check_result == -1.0) {
                    // Case 3: Confirmed Missing - Break loop, trigger search.
                    RCLCPP_WARN(this->get_logger(), "Human is CONFIRMED MISSING.");
                    human_was_missing = true;
                    break;
                } else {
                    // Case 4: Safety catch for unknown states.
                    RCLCPP_ERROR(this->get_logger(), "Verification failed. Assuming missing.");
                    human_was_missing = true;
                    break;
                }
                std::this_thread::sleep_for(100ms); 
            }
            
            // 3. Search Logic: Use Lawnmower Pattern
            if (human_was_missing) { 
                RCLCPP_WARN(this->get_logger(), "Human at (%.2f, %.2f) is MISSING! Initiating Lawnmower Search.", tx, ty);
                
                perform_lawnmower_search();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Mission Complete.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex data_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_local_costmap_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseCommander>();
    std::thread mission_thread([node](){ node->run_mission(); });
    rclcpp::spin(node);
    mission_thread.join();
    rclcpp::shutdown();
    return 0;
}