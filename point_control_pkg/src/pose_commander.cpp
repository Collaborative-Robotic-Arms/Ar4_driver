#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

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

std::string format_joint_values_deg(const std::vector<double>& values_rad) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < values_rad.size(); ++i) {
        if (i > 0) {
            oss << " | ";
        }
        oss << "j" << (i + 1) << "=" << (values_rad[i] * 180.0 / M_PI);
    }
    return oss.str();
}

void log_current_joint_state(
    MoveGroupInterface& move_group_interface,
    const rclcpp::Logger& logger,
    const std::string& label) {
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_WARN(logger, "%s joint state unavailable.", label.c_str());
        return;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(move_group_interface.getName());
    if (!joint_model_group) {
        RCLCPP_WARN(logger, "%s joint model group unavailable.", label.c_str());
        return;
    }

    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    RCLCPP_INFO(logger, "%s %s", label.c_str(),
        format_joint_values_deg(joint_values).c_str());
}

void log_planned_joint_target(
    const MoveGroupInterface::Plan& plan,
    const rclcpp::Logger& logger) {
    const auto& trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
        RCLCPP_WARN(logger, "Planned trajectory has no points.");
        return;
    }

    std::ostringstream joint_names_stream;
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
        if (i > 0) {
            joint_names_stream << ", ";
        }
        joint_names_stream << trajectory.joint_names[i];
    }

    const std::string joint_names = joint_names_stream.str();
    RCLCPP_INFO(logger, "Plan joint names: %s", joint_names.c_str());

    RCLCPP_INFO(logger, "Plan final joint target (deg): %s",
        format_joint_values_deg(trajectory.points.back().positions).c_str());
}

struct StoredJointTarget {
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
};

StoredJointTarget extract_final_joint_target(
    const MoveGroupInterface::Plan& plan,
    const rclcpp::Logger& logger) {
    const auto& trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
        RCLCPP_WARN(logger, "Cannot extract joint target from an empty trajectory.");
        return {};
    }

    if (trajectory.joint_names.empty()) {
        RCLCPP_WARN(logger, "Planned trajectory has no joint names.");
        return {};
    }

    const auto& final_positions = trajectory.points.back().positions;
    if (trajectory.joint_names.size() != final_positions.size()) {
        RCLCPP_WARN(
            logger,
            "Planned trajectory joint name/value size mismatch: %zu names vs %zu values.",
            trajectory.joint_names.size(),
            final_positions.size());
        return {};
    }

    return StoredJointTarget{trajectory.joint_names, final_positions};
}

bool plan_pose_from_abb(
    const geometry_msgs::msg::PoseStamped& pose_in_abb,
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Logger& logger,
    tf2_ros::Buffer& tf_buffer,
    MoveGroupInterface& move_group_interface,
    const std::array<double, 6>& coeff_x,
    const std::array<double, 6>& coeff_y,
    const std::array<double, 6>& coeff_z,
    MoveGroupInterface::Plan& plan,
    StoredJointTarget& final_joint_target) {
    geometry_msgs::msg::PoseStamped pose_ar4_base;
    try {
        pose_ar4_base = tf_buffer.transform(
            pose_in_abb,
            "base_link",
            tf2::durationFromSec(1.0)
        );
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(logger, "Transform world -> base_link failed: %s", ex.what());
        return false;
    }

    const double xb = pose_ar4_base.pose.position.x * 1000.0;
    const double yb = pose_ar4_base.pose.position.y * 1000.0;
    const double zb = pose_ar4_base.pose.position.z * 1000.0;

    const double x_corr = xb + poly_correction(xb, yb, coeff_x);
    const double y_corr = yb + poly_correction(xb, yb, coeff_y);
    const double z_corr = zb + poly_correction(xb, yb, coeff_z);

    geometry_msgs::msg::PoseStamped ee_pose_base;
    ee_pose_base.header.frame_id = "base_link";
    ee_pose_base.header.stamp = node->now();
    ee_pose_base.pose.position.x = x_corr / 1000.0;
    ee_pose_base.pose.position.y = y_corr / 1000.0;
    ee_pose_base.pose.position.z = z_corr / 1000.0;
    ee_pose_base.pose.orientation = pose_ar4_base.pose.orientation;

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

    move_group_interface.clearPoseTargets();
    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(ee_pose_base);

    const bool success = static_cast<bool>(move_group_interface.plan(plan));

    if (!success) {
        RCLCPP_ERROR(logger, "Planning failed! Check IK / reach.");
        return false;
    }

    final_joint_target = extract_final_joint_target(plan, logger);
    if (final_joint_target.joint_names.empty() || final_joint_target.joint_values.empty()) {
        RCLCPP_ERROR(logger, "Unable to extract the final joint target from the planned pose.");
        return false;
    }

    return true;
}

bool execute_joint_target(
    const StoredJointTarget& joint_target,
    MoveGroupInterface& move_group_interface,
    const rclcpp::Logger& logger,
    const std::string& target_label) {
    move_group_interface.clearPoseTargets();
    move_group_interface.setStartStateToCurrentState();

    if (!move_group_interface.setJointValueTarget(joint_target.joint_names, joint_target.joint_values)) {
        RCLCPP_ERROR(logger, "Failed to set joint target for replay.");
        return false;
    }

    MoveGroupInterface::Plan plan;
    const bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (!success) {
        RCLCPP_ERROR(logger, "Planning to the stored joint target failed.");
        return false;
    }

    log_current_joint_state(move_group_interface, logger,
        "Measured joints before execute (deg):");
    RCLCPP_INFO(logger, "%s %s",
        target_label.c_str(), format_joint_values_deg(joint_target.joint_values).c_str());
    log_planned_joint_target(plan, logger);

    move_group_interface.execute(plan);
    move_group_interface.stop();
    move_group_interface.clearPoseTargets();
    log_current_joint_state(move_group_interface, logger,
        "Measured joints after execute (deg):");
    return true;
}

bool execute_pose_from_abb(
    const geometry_msgs::msg::PoseStamped& pose_in_abb,
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Logger& logger,
    tf2_ros::Buffer& tf_buffer,
    MoveGroupInterface& move_group_interface,
    const std::array<double, 6>& coeff_x,
    const std::array<double, 6>& coeff_y,
    const std::array<double, 6>& coeff_z) {
    MoveGroupInterface::Plan plan;
    StoredJointTarget final_joint_target;
    if (!plan_pose_from_abb(
            pose_in_abb,
            node,
            logger,
            tf_buffer,
            move_group_interface,
            coeff_x,
            coeff_y,
            coeff_z,
            plan,
            final_joint_target)) {
        return false;
    }

    return execute_joint_target(
        final_joint_target,
        move_group_interface,
        logger,
        "Stored/replayed joint target (deg):");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<rclcpp::Node>("AR4_tool_control_final");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() {
        executor.spin();
    });

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    auto logger = rclcpp::get_logger("AR4_tool_control_final");

    MoveGroupInterface move_group_interface(node, "ar_manipulator");

    // Setting move_group_interface options
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.5);

    move_group_interface.setPlanningPipelineId("ompl");

    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(10);

    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setGoalJointTolerance(0.001);

    // check if robot frames are correct
    const std::string ee_link = move_group_interface.getEndEffectorLink();
    const std::string planning_frame = move_group_interface.getPlanningFrame();

    RCLCPP_INFO(logger, "MoveIt planning frame: %s", planning_frame.c_str());
    RCLCPP_INFO(logger, "End-effector link: %s", ee_link.c_str());

    // Hardcoded polynomial coefficients
    std::array<double, 6> coeff_x = { 1.64171585e-02, -7.70986685e-03, 4.33199285e-05, -2.35864795e-05, 3.85886719e-05, -3.60677853e+00 };
    std::array<double, 6> coeff_y = { -3.31082145e-02, -1.40551508e-01, 6.53234800e-05, -1.04520660e-04, -2.01740620e-05, -3.91090634e+01 };
    std::array<double, 6> coeff_z = { 2.04303887e-02, -4.48368468e-02, -4.81871107e-05, -7.90764635e-05, 3.42138435e-05, -1.78324448e+00 };

    double x = 0, y = 0, z = 0;
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    char option = 0;

    while (rclcpp::ok() && running) {
        std::cout << "\nHome (H), Pose (P), Sequence (S), Replay (R), Quit (Q): ";
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

            // Convert degrees --> radians
            double roll = roll_deg * M_PI / 180.0;
            double pitch = pitch_deg * M_PI / 180.0;
            double yaw = yaw_deg * M_PI / 180.0;

            tf2::Quaternion q_input;
            q_input.setRPY(roll, pitch, yaw);
            q_input.normalize();

            geometry_msgs::msg::PoseStamped pose_in_ABB;
            pose_in_ABB.header.frame_id = "ABB_base_link";
            pose_in_ABB.header.stamp = node->now();
            pose_in_ABB.pose.position.x = x;
            pose_in_ABB.pose.position.y = y;
            pose_in_ABB.pose.position.z = z;
            pose_in_ABB.pose.orientation = tf2::toMsg(q_input);

            if (execute_pose_from_abb(
                    pose_in_ABB,
                    node,
                    logger,
                    tf_buffer,
                    move_group_interface,
                    coeff_x,
                    coeff_y,
                    coeff_z)) {
                RCLCPP_INFO(logger, "Moved to AR4 pose.");
            }
        }

        else if (option == 'S' || option == 's') {
            const std::vector<double> x_values = {0.67, 0.58, 0.55, 0.56, 0.57, 0.65};
            std::vector<double> y_values;
            for (int i = -20; i <= 20; i += 5) {
                y_values.push_back(static_cast<double>(i) / 100.0);
            }

            std::vector<geometry_msgs::msg::PoseStamped> full_sequence;
            full_sequence.reserve(x_values.size() * y_values.size());

            const double seq_z = 0.11;
            const double seq_roll = 180.0 * M_PI / 180.0;
            const double seq_pitch = 0.0;
            const double seq_yaw = 90.0 * M_PI / 180.0;

            tf2::Quaternion seq_quaternion;
            seq_quaternion.setRPY(seq_roll, seq_pitch, seq_yaw);
            seq_quaternion.normalize();

            for (double seq_x : x_values) {
                for (double seq_y : y_values) {
                    geometry_msgs::msg::PoseStamped pose_in_abb;
                    pose_in_abb.header.frame_id = "ABB_base_link";
                    pose_in_abb.pose.position.x = seq_x;
                    pose_in_abb.pose.position.y = seq_y;
                    pose_in_abb.pose.position.z = seq_z;
                    pose_in_abb.pose.orientation = tf2::toMsg(seq_quaternion);
                    full_sequence.push_back(pose_in_abb);
                }
            }

            std::vector<geometry_msgs::msg::PoseStamped> sequence;
            sequence.reserve(5);
            for (size_t i = 0; i < 5; ++i) {
                const size_t sampled_index =
                    i * (full_sequence.size() - 1) / 4;
                sequence.push_back(full_sequence[sampled_index]);
            }

            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            bool sequence_aborted = false;
            for (size_t idx = 0; idx < sequence.size() && rclcpp::ok() && running; ++idx) {
                const auto& pose = sequence[idx];
                std::cout << "\nSequence pose " << (idx + 1) << "/" << sequence.size()
                          << ": x=" << pose.pose.position.x
                          << " y=" << pose.pose.position.y
                          << " z=" << pose.pose.position.z
                          << " rpy=(180, 0, 90)\n"
                          << "Press Enter to execute the next pose...";

                std::string enter_line;
                std::getline(std::cin, enter_line);
                if (!running || !rclcpp::ok()) {
                    sequence_aborted = true;
                    break;
                }

                geometry_msgs::msg::PoseStamped pose_to_execute = pose;
                pose_to_execute.header.stamp = node->now();

                if (!execute_pose_from_abb(
                        pose_to_execute,
                        node,
                        logger,
                        tf_buffer,
                        move_group_interface,
                        coeff_x,
                        coeff_y,
                        coeff_z)) {
                    sequence_aborted = true;
                    break;
                }

                RCLCPP_INFO(logger, "Completed sequence pose %zu/%zu.", idx + 1, sequence.size());
            }

            if (!sequence_aborted && rclcpp::ok() && running) {
                move_group_interface.setNamedTarget("home");
                move_group_interface.setStartStateToCurrentState();
                MoveGroupInterface::Plan msg;
                bool ok = static_cast<bool>(move_group_interface.plan(msg));

                if (ok) {
                    move_group_interface.execute(msg);
                    move_group_interface.stop();
                    move_group_interface.clearPoseTargets();
                    RCLCPP_INFO(logger, "Sequence completed. Moved to HOME position.");
                }
                else {
                    RCLCPP_ERROR(logger, "Sequence finished, but planning to HOME failed!");
                }
            }
        }

        else if (option == 'R' || option == 'r') {
            std::cout << "\nEnter target (x y z Roll Pitch Yaw in degrees): ";

            if (!(std::cin >> x >> y >> z >> roll_deg >> pitch_deg >> yaw_deg)) {
                std::cout << "Invalid input. Clear and retry.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            const double roll = roll_deg * M_PI / 180.0;
            const double pitch = pitch_deg * M_PI / 180.0;
            const double yaw = yaw_deg * M_PI / 180.0;

            tf2::Quaternion q_input;
            q_input.setRPY(roll, pitch, yaw);
            q_input.normalize();

            geometry_msgs::msg::PoseStamped pose_in_abb;
            pose_in_abb.header.frame_id = "ABB_base_link";
            pose_in_abb.header.stamp = node->now();
            pose_in_abb.pose.position.x = x;
            pose_in_abb.pose.position.y = y;
            pose_in_abb.pose.position.z = z;
            pose_in_abb.pose.orientation = tf2::toMsg(q_input);

            MoveGroupInterface::Plan captured_plan;
            StoredJointTarget stored_joint_target;
            if (!plan_pose_from_abb(
                    pose_in_abb,
                    node,
                    logger,
                    tf_buffer,
                    move_group_interface,
                    coeff_x,
                    coeff_y,
                    coeff_z,
                    captured_plan,
                    stored_joint_target)) {
                continue;
            }

            RCLCPP_INFO(logger, "Captured fixed replay joint target (deg): %s",
                format_joint_values_deg(stored_joint_target.joint_values).c_str());

            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            bool replay_running = true;
            while (replay_running && rclcpp::ok() && running) {
                std::cout << "\nReplay mode: Enter=move to stored joint target, H=home, Q=exit replay mode: ";

                std::string replay_command;
                std::getline(std::cin, replay_command);
                if (!rclcpp::ok() || !running) {
                    break;
                }

                if (replay_command.empty()) {
                    if (execute_joint_target(
                            stored_joint_target,
                            move_group_interface,
                            logger,
                            "Stored/replayed joint target (deg):")) {
                        RCLCPP_INFO(logger, "Replay move completed.");
                    }
                    continue;
                }

                const char replay_option = replay_command.front();
                if (replay_option == 'H' || replay_option == 'h') {
                    move_group_interface.setNamedTarget("home");
                    move_group_interface.setStartStateToCurrentState();
                    MoveGroupInterface::Plan home_plan;
                    bool ok = static_cast<bool>(move_group_interface.plan(home_plan));

                    if (ok) {
                        log_current_joint_state(move_group_interface, logger,
                            "Measured joints before execute (deg):");
                        log_planned_joint_target(home_plan, logger);
                        move_group_interface.execute(home_plan);
                        move_group_interface.stop();
                        move_group_interface.clearPoseTargets();
                        log_current_joint_state(move_group_interface, logger,
                            "Measured joints after execute (deg):");
                        RCLCPP_INFO(logger, "Moved to HOME position.");
                    }
                    else {
                        RCLCPP_ERROR(logger, "Planning to HOME failed!");
                    }
                    continue;
                }

                if (replay_option == 'Q' || replay_option == 'q') {
                    replay_running = false;
                    continue;
                }

                std::cout << "Invalid replay option. Use Enter, H, or Q.\n";
            }
        }

        else if (option == 'Q' || option == 'q') {
            std::cout << "Exiting...\n";
            break;
        }
        else {
            std::cout << "Invalid option. Please enter H, P, S, R, or Q.\n";
        }
    }

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}
