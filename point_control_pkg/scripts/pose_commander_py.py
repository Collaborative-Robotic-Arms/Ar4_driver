#!/usr/bin/env python3

import time
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


def plan_and_execute(robot, planning_component, logger, single_plan_parameters = None, multi_plan_parameters = None, sleep_time = 0.0):
    logger.info("planning Trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()
        
    if plan_result:
        logger.info("Executing plan!")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("planning failed :(")
        
    time.sleep(sleep_time)

def main():
    rclpy.init()
    logger = get_logger("Ar4 Pose Commander")
    
    AR4 = MoveItPy(node_name="AR4_pose_commander")
    AR4_robot = AR4.get_planning_component("AR4")
    logger.info("Instance created")
    
    AR4_robot.set_start_state(configuration_name = "home")

    AR4_robot.set_goal_state(configuration_name = "upright")
    
    plan_and_execute(AR4, AR4_robot, logger, sleep_time=3.0)
    
if __name__ == "__main__":
    main()