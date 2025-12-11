#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class WarehouseCommander : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WarehouseCommander() : Node("warehouse_commander")
    {
        // 1. Publisher to set the initial pose
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        // 2. Action Client for Navigation
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Mission Commander Initialized.");
    }

    // --- TEMPORARY TEST FUNCTION ---
    // In this function we pretrent that human 1 is gone (true) and that human 2 is present (false)
    bool mock_check_human_missing(int human_id) {
        RCLCPP_INFO(this->get_logger(), "[MOCK] Checking lidar for Human %d...", human_id);
        
        bool is_missing = (human_id == 1); 

        std::this_thread::sleep_for(1s); // Simulate processing time
        
        if (is_missing) {
            RCLCPP_WARN(this->get_logger(), "[MOCK] Result: Human %d is MISSING!", human_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "[MOCK] Result: Human %d is present.", human_id);
        }
        return is_missing;
    }

    void set_initial_position()
    {
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
        message.pose.covariance[7] = 0.01;
        message.pose.covariance[35] = 0.01;

        for(int i=0; i<3; i++) {
            init_pose_pub_->publish(message);
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_INFO(this->get_logger(), "Initial Pose Set.");
    }

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
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "Navigating to: (%.2f, %.2f)...", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // 1. Send Goal
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        auto goal_handle = goal_handle_future.get();

        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        // 2. Wait for Result
        auto result_future = nav_client_->async_get_result(goal_handle);
        auto result = result_future.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached destination.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach destination.");
        }
    }

    // Place holder for now
    void search_for_human(int human_id) {
        RCLCPP_INFO(this->get_logger(), "[MOCK] Searching for moved Human %d...", human_id);
        // Add search logic here later
    }

    void run_mission()
    {
        // 1. Initialize Localization
        set_initial_position();
        std::this_thread::sleep_for(2s);

        // 2. Define where the humans are (got these values from rviz map)
        double h1_inspect_x = 0.98; 
        double h1_inspect_y = 0.16;

        double h2_inspect_x = -12;
        double h2_inspect_y = 17;

        // Go to the position of human 1 and check whether the human is there or not
        navigate_to(h1_inspect_x, h1_inspect_y, 0.0);
        
        // Call the MOCK detector
        if (mock_check_human_missing(1)) {
            // If mock returns true (Missing), we search
            search_for_human(1);
        }

        // After were done searching for human 1 we check for human 2
        navigate_to(h2_inspect_x, h2_inspect_y, 1.57);
        
        if (mock_check_human_missing(2)) {
            search_for_human(2);
        }

        RCLCPP_INFO(this->get_logger(), "Mission Complete.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarehouseCommander>();
    
    std::thread mission_thread([node](){
        node->run_mission();
    });

    rclcpp::spin(node);
    mission_thread.join();
    rclcpp::shutdown();
    return 0;
}