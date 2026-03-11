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
double poly_correction(double x, double y, const std::array<double, 6>& coeff) {
    return coeff[0] * x + coeff[1] * y + coeff[2] * x * x + coeff[3] * y * y + coeff[4] * x * y + coeff[5];
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("AR4_tool_control_final");

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");

    MoveGroupInterface move_group_interface(node, "ar_manipulator");

    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.3);

    move_group_interface.setPlanningPipelineId("pilz");
    move_group_interface.setPlannerId("PTP");

    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(10);

    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setGoalJointTolerance(0.001);

    const std::string ee_link = move_group_interface.getEndEffectorLink();
    const std::string planning_frame = move_group_interface.getPlanningFrame();

    RCLCPP_INFO(logger, "MoveIt planning frame: %s", planning_frame.c_str());
    RCLCPP_INFO(logger, "End-effector link: %s", ee_link.c_str());

    // Hardcoded polynomial coefficients
    std::array<double, 6> coeff_x = { 1.64171585e-02, -7.70986685e-03, 4.33199285e-05, -2.35864795e-05, 3.85886719e-05, -3.60677853e+00 };
    std::array<double, 6> coeff_y = { -3.31082145e-02, -1.40551508e-01, 6.53234800e-05, -1.04520660e-04, -2.01740620e-05, -3.91090634e+01 };
    std::array<double, 6> coeff_z = { 2.04303887e-02, -4.48368468e-02, -4.81871107e-05, -7.90764635e-05, 3.42138435e-05, -1.78324448e+00 };

    // Safety limits
    double X_MIN = -500.0, X_MAX = 500.0;
    double Y_MIN = -500.0, Y_MAX = 0;
    double Z_MIN = 0.0, Z_MAX = 500.0;

    double x = 0, y = 0, z = 0;
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    char option = 0;

    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        std::cout << "\nHome (H), Pose (P), Quit (Q): ";
        std::cin >> option;

        if (option == 'H' || option == 'h') {
            move_group_interface.setNamedTarget("home");
            move_group_interface.setStartStateToCurrentState();
            MoveGroupInterface::Plan msg;
            bool ok = static_cast<bool>(move_group_interface.plan(msg));

            if (ok) {
                move_group_interface.execute(msg);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger, "Moved to HOME position.");
            }
            else {
                RCLCPP_ERROR(logger, "Planning to HOME failed!");
            }

        }
        else if (option == 'P' || option == 'p') {

            std::cout << "\nEnter target (x y z Roll Pitch Yaw in degrees): ";

            if (!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
                std::cout << "Invalid input. Clear and retry.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            // Convert degrees → radians
            double roll = roll_deg * M_PI / 180.0;
            double pitch = pitch_deg * M_PI / 180.0;
            double yaw = yaw_deg * M_PI / 180.0;

            tf2::Quaternion q_input;
            q_input.setRPY(roll, pitch, yaw);
            q_input.normalize();

            // ------------------------------------------------
            // 1️⃣ Input pose in WORLD (ABB base)
            // ------------------------------------------------
            geometry_msgs::msg::PoseStamped pose_in_ABB;
            pose_in_ABB.header.frame_id = "ABB_base_link";
            pose_in_ABB.header.stamp = node->now();
            pose_in_ABB.pose.position.x = x;
            pose_in_ABB.pose.position.y = y;
            pose_in_ABB.pose.position.z = z;
            pose_in_ABB.pose.orientation = tf2::toMsg(q_input);

            // ------------------------------------------------
            // 2️⃣ Transform WORLD → AR4 base_link
            // ------------------------------------------------
            geometry_msgs::msg::PoseStamped pose_ar4_base;
            try {
                pose_ar4_base = tf_buffer.transform(
                    pose_in_ABB,
                    "base_link",
                    tf2::durationFromSec(1.0)
                );
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(logger, "Transform world → base_link failed: %s", ex.what());
                continue;
            }

            // ------------------------------------------------
            // 3️⃣ Apply calibration in AR4 base frame (in mm)
            // ------------------------------------------------

            // Convert meters → mm
            double xb = pose_ar4_base.pose.position.x * 1000.0;
            double yb = pose_ar4_base.pose.position.y * 1000.0;
            double zb = pose_ar4_base.pose.position.z * 1000.0;

            //Apply calibration
            double x_corr = xb + poly_correction(xb, yb, coeff_x);
            double y_corr = yb + poly_correction(xb, yb, coeff_y);
            double z_corr = zb + poly_correction(xb, yb, coeff_z);

            // Convert back to meters
            geometry_msgs::msg::PoseStamped ee_pose_base;
            ee_pose_base.header.frame_id = "base_link";
            ee_pose_base.header.stamp = node->now();

            ee_pose_base.pose.position.x = pose_ar4_base.pose.position.x;
            ee_pose_base.pose.position.y = pose_ar4_base.pose.position.y - 0.01;
            ee_pose_base.pose.position.z = pose_ar4_base.pose.position.z;

            // Keep transformed orientation
            ee_pose_base.pose.orientation = pose_ar4_base.pose.orientation;

            // ------------------------------------------------
            // 🔎 PRINT FINAL COMMAND GOING TO AR4
            // ------------------------------------------------

            tf2::Quaternion q_out;
            tf2::fromMsg(ee_pose_base.pose.orientation, q_out);

            double r, p, yaw_out;

            tf2::Matrix3x3(q_out).getRPY(r, p, yaw_out);

            RCLCPP_INFO(logger,
                "\n====== AR4 FINAL COMMAND (base_link) ======\n"
                "Position (m): X=%.6f Y=%.6f Z=%.6f\n"
                "Orientation (deg): R=%.2f P=%.2f Y=%.2f\n"
                "===========================================",
                ee_pose_base.pose.position.x,
                ee_pose_base.pose.position.y,
                ee_pose_base.pose.position.z,
                r * 180.0 / M_PI,
                p * 180.0 / M_PI,
                yaw_out * 180.0 / M_PI
            );

            // ------------------------------------------------
            // 4️⃣ Plan and Execute
            // ------------------------------------------------
            move_group_interface.clearPoseTargets();
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(ee_pose_base);

            MoveGroupInterface::Plan plan;

            bool success = static_cast<bool>(move_group_interface.plan(plan));

            if (success) {
                move_group_interface.execute(plan);
                move_group_interface.stop();
                move_group_interface.clearPoseTargets();
                RCLCPP_INFO(logger, "Moved to calibrated AR4 pose.");
            }
            else {
                RCLCPP_ERROR(logger, "Planning failed! Check IK / reach.");
            }
        }

        else if (option == 'Q' || option == 'q') {
            std::cout << "Exiting...\n";
            break;
        }
        else {
            std::cout << "Invalid option. Please enter H, P, or Q.\n";
        }
    }

    rclcpp::shutdown();
    return 0;
}