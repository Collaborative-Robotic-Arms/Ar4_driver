#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <array>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    running = false;
}

// Polynomial correction function
double poly_correction(double x, double y, const std::array<double,6>& coeff) {
    return coeff[0]*x + coeff[1]*y + coeff[2]*x*x + coeff[3]*y*y + coeff[4]*x*y + coeff[5];
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("AR4_tool_control_final");

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");

    MoveGroupInterface move_group_interface(node, "ar_manipulator");

    move_group_interface.setMaxVelocityScalingFactor(0.7);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault");
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setGoalJointTolerance(0.001);

    const std::string ee_link = move_group_interface.getEndEffectorLink();
    const std::string planning_frame = move_group_interface.getPlanningFrame();

    RCLCPP_INFO(logger, "MoveIt planning frame: %s", planning_frame.c_str());
    RCLCPP_INFO(logger, "End-effector link: %s", ee_link.c_str());

    // Hardcoded polynomial coefficients
    std::array<double, 6> coeff_x = {1.64171585e-02, -7.70986685e-03, 4.33199285e-05, -2.35864795e-05, 3.85886719e-05, -3.60677853e+00};
    std::array<double, 6> coeff_y = {-3.31082145e-02, -1.40551508e-01, 6.53234800e-05, -1.04520660e-04, -2.01740620e-05, -3.91090634e+01};
    std::array<double, 6> coeff_z = {2.04303887e-02, -4.48368468e-02, -4.81871107e-05, -7.90764635e-05, 3.42138435e-05, -1.78324448e+00};

    // Safety limits
    double X_MIN=-500.0, X_MAX=500.0;
    double Y_MIN=-500.0, Y_MAX=0;
    double Z_MIN=0.0, Z_MAX=500.0;

    double x=0, y=0, z=0;
    double roll_deg=0, pitch_deg=0, yaw_deg=0;
    char option=0;

    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        std::cout << "\nHome (H), Pose (P), Quit (Q): ";
        std::cin >> option;

        if (option == 'H' || option == 'h') {
            move_group_interface.setNamedTarget("home");
            move_group_interface.setStartStateToCurrentState();
            MoveGroupInterface::Plan msg;
            bool ok = static_cast<bool>(move_group_interface.plan(msg));
            if(ok) {
                move_group_interface.execute(msg);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger,"Moved to HOME position.");
            } else {
                RCLCPP_ERROR(logger,"Planning to HOME failed!");
            }

        } else if (option == 'P' || option == 'p') {
            std::cout << "\nEnter target (x y z Roll Pitch Yaw in degrees): ";
            if(!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
                std::cout << "Invalid input. Clear and retry.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
                continue;
            }

            // Apply polynomial calibration
            double x_corr = x*1000 - poly_correction(x*1000, y*1000, coeff_x);
            double y_corr = y*1000 - poly_correction(x*1000, y*1000, coeff_y);
            double z_corr = z*1000 - poly_correction(x*1000, y*1000, coeff_z);

            // Safety check
            if (x_corr<X_MIN || x_corr>X_MAX || y_corr<Y_MIN || y_corr>Y_MAX || z_corr<Z_MIN || z_corr>Z_MAX) {
                RCLCPP_WARN(logger,"Corrected pose is OUT OF BOUNDS! Command aborted.");
                continue;
            }

            double roll = roll_deg * M_PI/180.0;
            double pitch = pitch_deg * M_PI/180.0;
            double yaw = yaw_deg * M_PI/180.0;

            tf2::Quaternion q_laser;
            q_laser.setRPY(roll,pitch,yaw);
            q_laser.normalize();

            geometry_msgs::msg::PoseStamped desired_laser_base;
            desired_laser_base.header.frame_id = "base_link";
            desired_laser_base.header.stamp = node->now();
            desired_laser_base.pose.position.x = x_corr/1000;
            desired_laser_base.pose.position.y = y_corr/1000;
            desired_laser_base.pose.position.z = z_corr/1000;
            desired_laser_base.pose.orientation = tf2::toMsg(q_laser);

            geometry_msgs::msg::TransformStamped t_ee_laser;
            try {
                t_ee_laser = tf_buffer.lookupTransform(ee_link,"laser_point",tf2::TimePointZero,1s);
            } catch(const tf2::TransformException& ex) {
                RCLCPP_ERROR(logger,"TF lookup failed: %s", ex.what());
                continue;
            }

            tf2::Transform tf_ee_laser;
            tf2::fromMsg(t_ee_laser.transform, tf_ee_laser);
            tf2::Transform tf_inv_ee_laser = tf_ee_laser.inverse();

            tf2::Transform tf_desired_laser_base;
            tf2::fromMsg(desired_laser_base.pose, tf_desired_laser_base);
            tf2::Transform tf_desired_ee_base = tf_desired_laser_base * tf_inv_ee_laser;

            geometry_msgs::msg::PoseStamped ee_pose_base;
            ee_pose_base.header.frame_id = "base_link";
            ee_pose_base.header.stamp = node->now();
            tf2::toMsg(tf_desired_ee_base, ee_pose_base.pose);

            move_group_interface.clearPoseTargets();
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(ee_pose_base);

            MoveGroupInterface::Plan plan;
            bool success = static_cast<bool>(move_group_interface.plan(plan));
            if(success) {
                move_group_interface.execute(plan);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger,"Moved laser to target point.");
            } else {
                RCLCPP_ERROR(logger,"Planning failed! Check IK and reach.");
            }

        } else if (option=='Q' || option=='q') {
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}