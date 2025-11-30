#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>  // Required for M_PI
#include <csignal>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <limits>  // Required for std::numeric_limits
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

// Define PI if not already defined (M_PI is standard in <cmath> or <math.h>)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    running = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>(
        "AR4_tool_control_final",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // TF buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");

    // MoveIt move group
    MoveGroupInterface move_group_interface(node, "ar_manipulator");

    RCLCPP_INFO(logger, "AR4 tool control (TCP final) started.");

    // Velocity & Acceleration
    move_group_interface.setMaxVelocityScalingFactor(0.7);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);

    // Planner settings
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault");

    // Tolerances
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setGoalJointTolerance(0.001);

    const std::string ee_link = move_group_interface.getEndEffectorLink();
    const std::string planning_frame = move_group_interface.getPlanningFrame();

    RCLCPP_INFO(logger, "MoveIt planning frame: %s", planning_frame.c_str());
    RCLCPP_INFO(logger, "End-effector link: %s", ee_link.c_str());

    double x = 0.0, y = 0.0, z = 0.0;
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    char option = 0;

    // Main loop
    while (rclcpp::ok() && running) {
        // process callbacks (TF updates)
        rclcpp::spin_some(node);

        std::cout << "\nHome (H or h), new pose (P or p), quit (Q or q): ";
        std::cin >> option;

        if (option == 'H' || option == 'h') {
            // --- HOMING OPTION ---
            move_group_interface.setNamedTarget("home");
            move_group_interface.setStartStateToCurrentState();

            auto const [success, plan] = [&move_group_interface]() {
                MoveGroupInterface::Plan msg;
                bool ok = static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();

            if (success) {
                move_group_interface.execute(plan);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger, "Moved to HOME position.");
            } else {
                RCLCPP_ERROR(logger, "Planning to HOME failed!");
            }

        } else if (option == 'P' || option == 'p') {
            // --- POSE OPTION (TCP Control with RPY Input) ---
            std::cout << "\nEnter target laser point (x y z Roll Pitch Yaw in "
                         "degrees) "
                         "in base_link coordinates: ";

            // Input validation and error clearing for 6 values
            if (!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >>
                  yaw_deg)) {
                std::cout << "Invalid input (Expected 6 numbers). Clearing "
                             "stream and continuing...\n";
                // Clear error flags and discard bad input
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(),
                                '\n');
                continue;
            }

            // Convert degrees to radians
            double roll = roll_deg * (M_PI / 180.0);
            double pitch = pitch_deg * (M_PI / 180.0);
            double yaw = yaw_deg * (M_PI / 180.0);

            // Calculate Quaternion from RPY (Roll, Pitch, Yaw)
            tf2::Quaternion q_laser;
            q_laser.setRPY(roll, pitch, yaw);
            q_laser.normalize();

            // 1. Build desired laser pose in base_link (T_base_laser)
            geometry_msgs::msg::PoseStamped desired_laser_base;
            desired_laser_base.header.frame_id = "base_link";
            desired_laser_base.header.stamp = node->now();

            // Translation from user input
            desired_laser_base.pose.position.x = x;
            desired_laser_base.pose.position.y = y;
            desired_laser_base.pose.position.z = z;

            // Orientation from user input (Quaternion)
            desired_laser_base.pose.orientation = tf2::toMsg(q_laser);

            // 2. Lookup transform ee_link -> laser_point (T_ee_laser)
            geometry_msgs::msg::TransformStamped t_ee_laser;
            try {
                // T_ee_laser: Transform from ee_link (source) to laser_point
                // (target)
                t_ee_laser = tf_buffer.lookupTransform(ee_link, "laser_point",
                                                       tf2::TimePointZero, 1s);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(logger,
                             "Failed to lookup transform %s -> laser_point: %s",
                             ee_link.c_str(), ex.what());
                RCLCPP_ERROR(logger,
                             "Ensure tf between ee_link and laser_point exists "
                             "and is static.");
                continue;
            }

            // 3. Compute desired ee pose in base_link: T_base_ee = T_base_laser
            // * (T_ee_laser)^-1

            // Convert msgs to tf2::Transform
            tf2::Transform tf_ee_laser;
            tf2::fromMsg(t_ee_laser.transform, tf_ee_laser);
            tf2::Transform tf_inv_ee_laser =
                tf_ee_laser.inverse();  // (T_ee_laser)^-1

            tf2::Transform tf_desired_laser_base;
            tf2::fromMsg(desired_laser_base.pose, tf_desired_laser_base);

            // Compute desired ee pose in base_link
            tf2::Transform tf_desired_ee_base =
                tf_desired_laser_base *
                tf_inv_ee_laser;  // T_base_ee = T_base_laser * T_laser_ee

            // 4. Convert tf2::Transform -> geometry_msgs::msg::PoseStamped
            geometry_msgs::msg::PoseStamped ee_pose_base;
            // The goal is defined in the 'base_link' frame
            ee_pose_base.header.frame_id = "base_link";
            ee_pose_base.header.stamp = node->now();
            tf2::toMsg(tf_desired_ee_base,
                       ee_pose_base.pose);  // Cleaner conversion

            // Debug info
            RCLCPP_INFO(logger,
                        "Desired laser (base_link): XYZ=%.4f %.4f %.4f | "
                        "RPY_deg=%.0f %.0f %.0f",
                        desired_laser_base.pose.position.x,
                        desired_laser_base.pose.position.y,
                        desired_laser_base.pose.position.z, roll_deg, pitch_deg,
                        yaw_deg);

            RCLCPP_INFO(logger, "Computed ee (base_link): %.4f %.4f %.4f",
                        ee_pose_base.pose.position.x,
                        ee_pose_base.pose.position.y,
                        ee_pose_base.pose.position.z);

            // 5. Send the goal pose directly in its defined frame ("base_link")
            // MoveIt will internally transform this PoseStamped into the
            // planning_frame.

            move_group_interface.clearPoseTargets();
            move_group_interface.setStartStateToCurrentState();

            // Use the PoseStamped version with the correct header.frame_id
            move_group_interface.setPoseTarget(ee_pose_base);

            // 6. Plan and Execute
            auto const [success, plan] = [&move_group_interface]() {
                MoveGroupInterface::Plan msg;
                bool ok = static_cast<bool>(move_group_interface.plan(msg));
                return std::make_pair(ok, msg);
            }();

            if (success) {
                move_group_interface.execute(plan);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger, "Moved laser to target point.");
            } else {
                RCLCPP_ERROR(logger,
                             "Planning to computed ee pose failed! (Check IK "
                             "solution and reach)");
            }

        } else if (option == 'q' || option == 'Q') {
            // --- QUITTING OPTION ---
            break;
        }
    }  // while

    rclcpp::shutdown();
    return 0;
}