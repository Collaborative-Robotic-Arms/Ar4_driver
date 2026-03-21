#include <memory>
#include <thread>
#include <string>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Interfaces
#include "dual_arms_msgs/action/execute_task.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Controller Interfaces
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AR4TaskServer : public rclcpp::Node
{
public:
    using ExecuteTask = dual_arms_msgs::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

    AR4TaskServer() : Node("ar4_task_server")
    {   
        this->declare_parameter("use_sim", false); 
        this->use_sim_ = this->get_parameter("use_sim").as_bool();
        
        if (use_sim_) RCLCPP_INFO(this->get_logger(), "Starting in SIMULATION MODE.");
        else RCLCPP_INFO(this->get_logger(), "Starting in REAL HARDWARE MODE.");
        
        if (use_sim_) {
            // NOTE: Update this name if your AR4 Gazebo controller is named differently
            this->sim_gripper_client_ = rclcpp_action::create_client<TrajectoryAction>(
                this, "/ar4_gripper_controller/follow_joint_trajectory" 
            );
        }

        this->action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this, "ar4_control", 
            std::bind(&AR4TaskServer::handle_goal, this, _1, _2),
            std::bind(&AR4TaskServer::handle_cancel, this, _1),
            std::bind(&AR4TaskServer::handle_accepted, this, _1)
        );

        // Create a separate thread group for the service to prevent deadlocks
        this->service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        this->gripper_service_server_ = this->create_service<std_srvs::srv::SetBool>(
            "ar4_controller/set_gripper", 
            std::bind(&AR4TaskServer::handle_gripper_service, this, _1, _2),
            rmw_qos_profile_services_default,
            this->service_cb_group_ // <-- Assign the group here
        );

        this->gripper_client_ = this->create_client<std_srvs::srv::SetBool>("ar4_gripper/set");
    }

    void init()
    {
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for 'world' frame to appear in TF...");
        while (rclcpp::ok()) {
            if (tf_buffer->canTransform("world", "abb_table", tf2::TimePointZero)) {
                RCLCPP_INFO(this->get_logger(), "'world' frame found. Initializing MoveIt...");
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        static const std::string ROBOT_GROUP_NAME = "ar_manipulator"; 
        try {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOT_GROUP_NAME);
            move_group_->setMaxVelocityScalingFactor(0.4);
            move_group_->setMaxAccelerationScalingFactor(0.3);
            move_group_->setGoalPositionTolerance(0.01);
            move_group_->setGoalOrientationTolerance(0.01);
            move_group_->setPlanningTime(10.0); 
            move_group_->setPoseReferenceFrame("abb_table");
            move_group_->setEndEffectorLink("ar4_ee_link"); 
            move_group_->setWorkspace(-1.5, -1.5, -0.5, 1.5, 1.5, 2.5);

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface Ready for AR4.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
        }
    }

private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_service_server_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    bool use_sim_; 
    rclcpp_action::Client<TrajectoryAction>::SharedPtr sim_gripper_client_;

    // Helper function to decode MoveIt errors
    std::string decode_moveit_error(int32_t code) {
        switch(code) {
            case 1: return "SUCCESS";
            case 99999: return "FAILURE (General)";
            case -1: return "PLANNING_FAILED";
            case -2: return "INVALID_MOTION_PLAN";
            case -3: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
            case -4: return "CONTROL_FAILED";
            case -5: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
            case -6: return "TIMED_OUT";
            case -7: return "PREEMPTED";
            case -10: return "START_STATE_IN_COLLISION";
            case -11: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
            case -12: return "GOAL_IN_COLLISION";
            case -13: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
            case -14: return "GOAL_CONSTRAINTS_VIOLATED";
            case -15: return "INVALID_GROUP_NAME";
            case -16: return "INVALID_GOAL_CONSTRAINTS";
            case -17: return "INVALID_ROBOT_STATE";
            case -18: return "INVALID_LINK_NAME";
            case -19: return "INVALID_OBJECT_NAME";
            case -21: return "FRAME_TRANSFORM_FAILURE";
            case -22: return "COLLISION_CHECKING_UNAVAILABLE";
            case -23: return "ROBOT_STATE_STALE";
            case -24: return "SENSOR_INFO_STALE";
            case -25: return "COMMUNICATION_FAILURE";
            case -31: return "NO_IK_SOLUTION";
            default: return "UNKNOWN_ERROR_CODE";
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received AR4 task: %s", goal->task_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>) {
        RCLCPP_INFO(this->get_logger(), "Cancel requested");
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_gripper_service(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "MTC/Supervisor requested AR4 Gripper: %s", request->data ? "OPEN" : "CLOSE");
        
        // Pass nullptr for the goal_handle since this is a Service, not an Action!
        bool success = control_gripper(request->data, nullptr);
        
        response->success = success;
        response->message = success ? "Gripper moved successfully" : "Gripper failed to move";
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        std::thread{std::bind(&AR4TaskServer::execute, this, _1), goal_handle}.detach();
    }

    // --- HELPER MACRO: Safely exit if Supervisor cancels us mid-execution ---
    #define HANDLE_FAILURE(error_msg) \
        result->success = false; \
        if (goal_handle->is_canceling()) { \
            result->error_message = "Canceled by Supervisor"; \
            goal_handle->canceled(result); \
            RCLCPP_WARN(this->get_logger(), "Task canceled gracefully. Yielding to MTC."); \
        } else { \
            result->error_message = error_msg; \
            goal_handle->abort(result); \
        } \
        return;

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        auto result = std::make_shared<ExecuteTask::Result>();

        if (!move_group_) { HANDLE_FAILURE("MoveGroup not initialized!"); }

        if (goal->task_type == "PICK")
        {
            if (!control_gripper(true, goal_handle)) { HANDLE_FAILURE("PICK: Failed to open gripper"); }

            feedback->current_status = "MOVING_TO_PREGRASP";
            goal_handle->publish_feedback(feedback);
            geometry_msgs::msg::Pose pregrasp = goal->target_pose;
            pregrasp.position.z = pregrasp.position.z + 0.1;
            if (!move_to_pose(pregrasp, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pregrasp"); }

            feedback->current_status = "MOVING_TO_TARGET";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pose"); }

            if (!control_gripper(false, goal_handle)) { HANDLE_FAILURE("PICK: Failed to grasp object"); }

            geometry_msgs::msg::Pose postgrasp = goal->target_pose;
            postgrasp.position.z = postgrasp.position.z + 0.1;
            if (!move_to_pose(postgrasp, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach postgrasp"); }
        }
        else if (goal->task_type == "PLACE")
        {
            feedback->current_status = "MOVING_TO_PRE_PLACE";
            goal_handle->publish_feedback(feedback);
            geometry_msgs::msg::Pose preplace = goal->target_pose;
            preplace.position.z = preplace.position.z + 0.1;
            if (!move_to_pose(preplace, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pre place"); }

            feedback->current_status = "MOVING_TO_PLACE";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pose"); }

            feedback->current_status = "RELEASING_OBJECT";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to release object"); }
            
            feedback->current_status = "RETURNING_HOME";
            goal_handle->publish_feedback(feedback);
            move_to_named_target("home", goal_handle); 
        }
        else if (goal->task_type == "HOME")
        {
            if (!move_to_named_target("home", goal_handle)) { HANDLE_FAILURE("HOME: Failed to reach home"); }
        }
        else if (goal->task_type == "INTERMEDIATE_GIVE")
        {
            // Move to handover zone and wait (keep gripper closed holding the brick)
            feedback->current_status = "MOVING_TO_HANDOVER_ZONE";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_GIVE: Failed to reach pose"); }
        }
        else if (goal->task_type == "INTERMEDIATE_TAKE")
        {
            // 1. Open gripper to prepare for handover
            feedback->current_status = "OPENING_GRIPPER_FOR_TAKE";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to open gripper"); }

            // 2. Move precisely to the grasp point on the hovering brick
            feedback->current_status = "MOVING_TO_HANDOVER_GRASP";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to reach pose"); }

            // 3. Close gripper to secure the brick
            feedback->current_status = "CLOSING_GRIPPER_TO_TAKE";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(false, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to grasp"); }
        }
        else if (goal->task_type == "RELEASE")
        {
            // Tell the giving arm to let go after the taking arm has secured it
            feedback->current_status = "RELEASING_BRICK";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true, goal_handle)) { HANDLE_FAILURE("RELEASE: Failed to open gripper"); }
        }

        // If we reach here naturally, ensure we haven't been canceled at the last millisecond
        if (goal_handle->is_canceling()) { HANDLE_FAILURE("Canceled at finish"); }

        result->success = true;
        result->error_message = "None";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "AR4 Task Completed Successfully.");
    }

    bool move_to_pose(const geometry_msgs::msg::Pose & target, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;

        // 1. Wrap the raw Pose in a PoseStamped to enforce the table frame
        geometry_msgs::msg::PoseStamped stamped_target;
        stamped_target.header.frame_id = "abb_table";
        stamped_target.pose = target;

        // 2. Pass the explicitly framed pose to MoveIt
        move_group_->setPoseTarget(stamped_target);

        // ==========================================================
        // --- NEW DEBUG LOGGING: Verify MoveIt's Frame Knowledge ---
        // ==========================================================
        RCLCPP_INFO(this->get_logger(), "--- MOVEIT FRAME DIAGNOSTICS ---");
        RCLCPP_INFO(this->get_logger(), "1. Target Pose Frame set to: '%s'", stamped_target.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "2. MoveGroup Pose Reference Frame: '%s'", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "3. MoveGroup Planning Frame (Root): '%s'", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "4. MoveGroup End-Effector Link: '%s'", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "--------------------------------");
        
        RCLCPP_INFO(this->get_logger(), 
            "Targeting Pose -> P(x:%.3f, y:%.3f, z:%.3f) | Q(w:%.2f, x:%.2f, y:%.2f, z:%.2f)", 
            target.position.x, target.position.y, target.position.z, 
            target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);

        auto current_state = move_group_->getCurrentState(1.0);
        if (current_state) {
            current_state->enforceBounds();
            move_group_->setStartState(*current_state);
        } else {
            move_group_->setStartStateToCurrentState();
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto error_code = move_group_->plan(plan);
        
        // CRITICAL: Check if Supervisor canceled us WHILE we were doing the heavy math
        if (goal_handle->is_canceling()) {
            move_group_->clearPoseTargets();
            return false; 
        }
        
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            auto exec_code = move_group_->execute(plan);
            bool success = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), 
                    "MoveIt Execution Failed! Code: %d (%s)", 
                    exec_code.val, decode_moveit_error(exec_code.val).c_str());
            }

            move_group_->clearPoseTargets();
            return success;
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "MoveIt Planning Failed! Code: %d (%s)", 
                error_code.val, decode_moveit_error(error_code.val).c_str());
        }
        
        move_group_->clearPoseTargets();
        return false;
    }

    bool move_to_named_target(const std::string & name, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;
        
        RCLCPP_INFO(this->get_logger(), "Targeting Named Pose -> %s", name.c_str());
        
        move_group_->setNamedTarget(name);
        auto current_state = move_group_->getCurrentState(1.0);
        if (current_state) {
            current_state->enforceBounds();
            move_group_->setStartState(*current_state);
        } else {
            move_group_->setStartStateToCurrentState();
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        auto error_code = move_group_->plan(plan);

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            if (goal_handle->is_canceling()) return false; // CRITICAL CANCEL CHECK
            
            auto exec_code = move_group_->execute(plan);
            if (exec_code == moveit::core::MoveItErrorCode::SUCCESS) {
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "MoveIt Execution to %s Failed! Code: %d (%s)", 
                    name.c_str(), exec_code.val, decode_moveit_error(exec_code.val).c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "MoveIt Planning to %s Failed! Code: %d (%s)", 
                name.c_str(), error_code.val, decode_moveit_error(error_code.val).c_str());
        }
        return false;
    }

    bool send_sim_gripper_command(bool open, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (!sim_gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "AR4 Sim Gripper Action Server not found! Check controller name.");
            return true; // Fallback so the state machine doesn't freeze
        }
        
        auto goal_msg = TrajectoryAction::Goal();
        
        // IMPORTANT: Update these joint names to match your AR4 Gazebo URDF!
        goal_msg.trajectory.joint_names = {
            "ar4_gripper_jaw1_joint", 
            "ar4_gripper_jaw2_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Update these positions based on your AR4 URDF joint limits!
        // Usually, 0.0 is closed, and some small positive number (e.g., 0.015) is open, or vice versa.
        double pos = open ? 0.0135 : 0.001; 
        point.positions = {pos, pos};
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal_msg.trajectory.points.push_back(point);

        auto goal_handle_future = sim_gripper_client_->async_send_goal(goal_msg);
        
        // Wait for server to accept the goal while monitoring for MTC cancellations
        while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle != nullptr && goal_handle->is_canceling()) return false;       
        }

        auto action_goal_handle = goal_handle_future.get();
        if (!action_goal_handle) return false;

        auto result_future = sim_gripper_client_->async_get_result(action_goal_handle);
        
        // Wait for the gripper to finish moving while monitoring for MTC cancellations
        while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (goal_handle != nullptr && goal_handle->is_canceling()) return false;
        }

        return true;
    }

    bool control_gripper(bool open, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (use_sim_) {
            RCLCPP_INFO(this->get_logger(), "SIMULATION MODE: Moving AR4 gripper to %s.", open ? "OPEN" : "CLOSE");
            return send_sim_gripper_command(open, goal_handle);
        }

        if (!gripper_client_->wait_for_service(std::chrono::seconds(2))) return false;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = open; 
        auto future = gripper_client_->async_send_request(request);
        
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            return future.get()->success;
        }
        return false;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<AR4TaskServer>();
    
    executor->add_node(node);
    node->init();
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}