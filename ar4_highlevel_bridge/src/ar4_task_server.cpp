#include <memory>
#include <thread>
#include <string>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Interfaces
#include "dual_arms_msgs/action/execute_task.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AR4TaskServer : public rclcpp::Node
{
public:
    using ExecuteTask = dual_arms_msgs::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    AR4TaskServer() : Node("ar4_task_server")
    {   
        // --- SIMULATION PARAMETER ---
        // Defaults to true so you can test in Gazebo without the physical Nano plugged in
        this->declare_parameter("use_sim", true); 
        this->use_sim_ = this->get_parameter("use_sim").as_bool();
        
        if (use_sim_) {
            RCLCPP_INFO(this->get_logger(), "Starting in SIMULATION MODE. Gripper hardware will be bypassed.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Starting in REAL HARDWARE MODE. Waiting for Nano gripper service.");
        }

        // 1. Initialize Action Server
        this->action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this,
            "ar4_control", 
            std::bind(&AR4TaskServer::handle_goal, this, _1, _2),
            std::bind(&AR4TaskServer::handle_cancel, this, _1),
            std::bind(&AR4TaskServer::handle_accepted, this, _1)
        );

        // 2. Initialize Gripper Client
        this->gripper_client_ = this->create_client<std_srvs::srv::SetBool>(
            "ar4_gripper/set"
        );

        RCLCPP_INFO(this->get_logger(), "AR4 Task Server initialized. Waiting for MoveIt init()...");
    }

    void init()
    {
        static const std::string ROBOT_GROUP_NAME = "ar_manipulator"; 
        try {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOT_GROUP_NAME);
            
            move_group_->setPlanningPipelineId("move_group");
            // move_group_->setPlannerId("RRTConnectkConfigDefault");
            move_group_->setMaxVelocityScalingFactor(0.4);
            move_group_->setMaxAccelerationScalingFactor(0.3);
            move_group_->setGoalPositionTolerance(0.01);
            move_group_->setGoalOrientationTolerance(0.01);
            move_group_->setGoalJointTolerance(0.01);
            move_group_->setPlanningTime(15.0); // Gives MoveIt 15 seconds to find a path
            move_group_->setPoseReferenceFrame("abb_table");
            move_group_->setWorkspace(-1.5, -1.5, -0.5, 1.5, 1.5, 2.5);

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface Ready for AR4.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
        }
    }

private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    bool use_sim_; // Tracks if we are skipping hardware

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received AR4 task: %s", goal->task_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel requested");
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        std::thread{std::bind(&AR4TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        auto result = std::make_shared<ExecuteTask::Result>();

        if (!move_group_) {
            result->success = false;
            result->error_message = "MoveGroup not initialized!";
            goal_handle->abort(result);
            return;
        }

        // --- Logic for PICK ---
        if (goal->task_type == "PICK")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard AR4 PICK sequence");
            
            // 1. Open Gripper
            if (!control_gripper(true)) { 
                result->success = false;
                result->error_message = "PICK: Failed to open gripper";
                goal_handle->abort(result);
                return;
            }

            // 2. Pre-Grasp Approach (Z + 10cm)
            feedback->current_status = "MOVING_TO_PREGRASP";
            goal_handle->publish_feedback(feedback);
            
            geometry_msgs::msg::Pose pregrasp = goal->target_pose;
            pregrasp.position.z = pregrasp.position.z + 0.1;
            
            if (!move_to_pose(pregrasp)) {
                result->success = false;
                result->error_message = "PICK: MoveIt failed to reach pregrasp";
                goal_handle->abort(result);
                return;
            }

            // 3. Move to Actual Target
            feedback->current_status = "MOVING_TO_TARGET";
            goal_handle->publish_feedback(feedback);

            if (!move_to_pose(goal->target_pose)) {
                result->success = false;
                result->error_message = "PICK: MoveIt failed to reach pose";
                goal_handle->abort(result);
                return;
            }

            // 4. Close Gripper
            if (!control_gripper(false)) { 
                result->success = false;
                result->error_message = "PICK: Failed to grasp object";
                goal_handle->abort(result);
                return;
            }

            // 5. Post-Grasp Retreat (Z + 10cm)
            geometry_msgs::msg::Pose postgrasp = goal->target_pose;
            postgrasp.position.z = postgrasp.position.z + 0.1;
            
            if (!move_to_pose(postgrasp)) {
                result->success = false;
                result->error_message = "PICK: MoveIt failed to reach postgrasp";
                goal_handle->abort(result);
                return;
            }
        }
        // --- Logic for PLACE ---
        else if (goal->task_type == "PLACE")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard AR4 PLACE sequence");

            // 1. Move to Pre-Place Location
            feedback->current_status = "MOVING_TO_PRE_PLACE";
            goal_handle->publish_feedback(feedback);
            
            geometry_msgs::msg::Pose preplace = goal->target_pose;
            preplace.position.z = preplace.position.z + 0.1;
            if (!move_to_pose(preplace)) {
                result->success = false;
                result->error_message = "PLACE: MoveIt failed to reach pre place";
                goal_handle->abort(result);
                return;
            }

            // 2. Move to Place Location
            feedback->current_status = "MOVING_TO_PLACE";
            goal_handle->publish_feedback(feedback);
            
            if (!move_to_pose(goal->target_pose)) {
                result->success = false;
                result->error_message = "PLACE: MoveIt failed to reach pose";
                goal_handle->abort(result);
                return;
            }

            // 3. Open Gripper (Release Object)
            feedback->current_status = "RELEASING_OBJECT";
            goal_handle->publish_feedback(feedback);

            if (!control_gripper(true)) { 
                result->success = false;
                result->error_message = "PLACE: Failed to release object";
                goal_handle->abort(result);
                return;
            }
            
            // 4. Return to HOME
            feedback->current_status = "RETURNING_HOME";
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "Returning to HOME position...");
            move_to_named_target("home"); // Don't abort if home fails, object is placed
        }
        else {
            result->success = false;
            result->error_message = "Task " + goal->task_type + " not implemented for AR4";
            goal_handle->abort(result);
            return;
        }

        result->success = true;
        result->error_message = "None";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "AR4 Task Completed Successfully.");
    }

    bool move_to_pose(const geometry_msgs::msg::Pose & target)
    {
        move_group_->setPoseTarget(target);
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        RCLCPP_INFO(this->get_logger(), 
            "Group moving to: P(%.4f, %.4f, %.4f) | Q(w:%.2f, x:%.2f, y:%.2f, z:%.2f)", 
            target.position.x, target.position.y, target.position.z, 
            target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z
        );
        // Capture the exact return code
        auto error_code = move_group_->plan(plan);
        
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            bool success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_->clearPoseTargets();
            return success;
        } else {
            // Print the specific reason it failed!
            RCLCPP_ERROR(this->get_logger(), "MoveIt Planning Failed! Error Code: %d", error_code.val);
        }
        
        move_group_->clearPoseTargets();
        return false;
    }

    bool move_to_named_target(const std::string & name)
    {
        move_group_->setNamedTarget(name);
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool control_gripper(bool open)
    {
        // --- SIMULATION BYPASS ---
        if (use_sim_) {
            RCLCPP_INFO(this->get_logger(), "SIMULATION MODE: Pretending to %s gripper.", open ? "OPEN" : "CLOSE");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Simulate mechanical delay
            return true; 
        }
        // -------------------------

        // --- REAL HARDWARE LOGIC ---
        if (!gripper_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper service not found! Is the Nano connected and the service running?");
            return false;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = open; // True = Open, False = Close
        
        auto future = gripper_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            return future.get()->success;
        }
        RCLCPP_ERROR(this->get_logger(), "Gripper service call timed out!");
        return false;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AR4TaskServer>();
    node->init(); // Ensure MoveIt is initialized properly
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}