/**************************** INCLUDES *************************************/
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <csignal>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

using namespace std;
using moveit::planning_interface::MoveGroupInterface;

std::atomic<bool> running(true);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    running = false;
}

int main(int arc, char *argv[]) {
    /********************** INITIALIZATION
     * ****************************/
    rclcpp::init(arc, argv);
    std::signal(SIGINT, signalHandler);

    auto const node = std::make_shared<rclcpp::Node>(
        "AR4_tool_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto const logger = rclcpp::get_logger("AR4_tool_control");

    auto move_group_interface = MoveGroupInterface(node, "ar_manipulator");

    RCLCPP_INFO(logger, "Ready to receive pose targets. Press CTRL+C to quit.");

    /**************************** VELOCITY AND ACCELERATION
     * ***********************************/

    move_group_interface.setMaxVelocityScalingFactor(
        0.7);  // 20% of max velocity
    move_group_interface.setMaxAccelerationScalingFactor(0.5);

    /**************************** SETTING PLANNER
     * ***********************************/
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault");

    double x, y, z = 0.0;
    double roll_deg, pitch_deg, yaw_deg = 0.0;
    double w = 1.0;

    char option = 0;

    while (rclcpp::ok() && running) {
        cout << "\nHome (H or h), new pose (P or p), quit (Q or q): ";
        cin >> option;
        /****************************  HOMING OPTION
         * ***********************************/

        if (option == 'H' || option == 'h') {
            // ---- Go to Home preset ----
            move_group_interface.setNamedTarget("home");

            auto const [success, plan] = [&move_group_interface] {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok =
                    static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();

            if (success) {
                move_group_interface.execute(plan);
                RCLCPP_INFO(logger, "Moved to HOME position.");
            } else {
                RCLCPP_ERROR(logger, "Planning to HOME failed!");
            }
            /**************************** POSITION GIVEN
             * ************************************/

        } else if (option == 'P' || option == 'p') {
            // ---- Custom pose ----
            cout << "\nEnter target pose (x y z roll pitch yaw in degrees): ";
            if (!(cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
                cout << "Invalid input. Exiting...\n";
                break;
            }

            double roll = (roll_deg * M_PI) / 180.0;
            double pitch = (pitch_deg * M_PI) / 180.0;
            double yaw = (yaw_deg * M_PI) / 180.0;

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();

            geometry_msgs::msg::PoseStamped input_pose;
            input_pose.header.frame_id =
                "laser_point";  // pose given in laser_point frame
            input_pose.header.stamp = node->now();
            input_pose.pose.orientation = tf2::toMsg(q);
            input_pose.pose.position.x = x;
            input_pose.pose.position.y = y;
            input_pose.pose.position.z = z;

            geometry_msgs::msg::PoseStamped transformed_pose;
            try {
                transformed_pose = tf_buffer.transform(
                    input_pose, "ee_link", tf2::durationFromSec(1.0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(logger, "Transform error: %s", ex.what());
                continue;  // skip this iteration
            }

            move_group_interface.setPoseTarget(transformed_pose.pose);

            auto const [success, plan] = [&move_group_interface] {
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok =
                    static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();

            if (success) {
                move_group_interface.execute(plan);
                RCLCPP_INFO(logger, "Moved to custom target pose.");
            } else {
                RCLCPP_ERROR(logger, "Planning to target pose failed!");
            }
                /**************************** QUITTING OPTION
     * ***********************************/

        } else if (option == 'q' || option == 'Q')
            break;
    }
    rclcpp::shutdown();
    return 0;
}
