import argparse
import time
import sys
import IPython
import numpy as np
e = IPython.embed

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
import cv2
from calibration.marker_detection import camera2marker
from calibration.calibrations import solve_rigid_transformation, calculate_reprojection_error
from datetime import datetime
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
def solve_calibration(T_eef2marker, T_base2eef_set, T_camera2marker_set, which_arm):
    T_base2marker_set = [np.dot(T_base2eef, T_eef2marker) for T_base2eef in T_base2eef_set]
    
    T_base2camera_set = []
    avg_error_set = []
    
    methods = ["ONE_SAMPLE_ESTIMATE", "SVD_ALGEBRAIC", "CALIB_HAND_EYE_TSAI", "CALIB_HAND_EYE_ANDREFF"]
    
    for method in methods:
        print(f"\nMETHOD: {method}")
        T_base2camera = solve_rigid_transformation(T_base2marker_set, T_camera2marker_set, method=method)
        avg_error, std_error = calculate_reprojection_error(T_base2marker_set, T_camera2marker_set, T_base2camera)
        T_base2camera_set.append(T_base2camera)
        avg_error_set.append(avg_error)
        print(f"Transformation matrix T_base2camera:\n{T_base2camera}")
        print(f"Avg. reprojection error: {avg_error}, std. error: {std_error}")
        
    # 3.2. Save the best calibration and error for debugging
    T_base2camera = T_base2camera_set[np.argmin(avg_error_set)]
    now = datetime.now().strftime("%Y%m%d%H%M")
    np.save("T_base2camera_" + which_arm + now +".npy", T_base2camera)
    
    return T_base2camera

def collect_data(master_bot_left, master_bot_right, puppet_bot_left, puppet_bot_right):
    """ Move all 4 robots to a pose where it is easy to start demonstration """
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot_left.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_left.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_left.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    puppet_bot_right.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_right.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_right.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    torque_on(puppet_bot_left)
    torque_on(master_bot_left)
    torque_on(puppet_bot_right)
    torque_on(master_bot_right)

    # move arms to starting position
    T_camera2marker_LEFT = []
    T_base2eef_LEFT = []
    T_camera2marker_RIGHT = []
    T_base2eef_RIGHT = []

    LEFT_ARM_POSE = [[[-0.3, -0.96, 1.16, 0, 0.3, 0], [-0.3, 0.96, -1.16, 0, 0.3, 0]]]
    RIGHT_ARM_POSE = [[[0.5, -0.96, 1.16, 0, -0.3, 0], [0.5, 0.96, -1.16, 0, -0.3, 0]]]

    LEFT_ARM_POSE.append([[-0.3, -0.96, 1.3, 0, 0.3, 0], [-0.3, 0.96, -1.3, 0, 0.3, 0]])
    RIGHT_ARM_POSE.append([[0.5, -0.96, 1.4, 0, -0.3, 0], [0.5, 0.96, -1.4, 0, -0.3, 0]])
    
    LEFT_ARM_POSE.append([[-0.2, -0.3, 1.16, 0, 0.3, 0], [-0.2, 0.3, -1.16, 0, 0.3, 0]])
    RIGHT_ARM_POSE.append([[0.5, -0.96, 1.16, -0.3, -0.7, 0], [0.5, 0.96, -1.16, -0.3, -0.7, 0]])
    
    # Move arms so that the aruco marker is detectable 
    for i in range(len(LEFT_ARM_POSE)):
        move_arms([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], LEFT_ARM_POSE[i] + RIGHT_ARM_POSE[i], move_time=1.5)

        print('Left End-effector pose: ', puppet_bot_left.arm.get_ee_pose()) # 581
        print('Right End-effector pose: ', puppet_bot_right.arm.get_ee_pose()) # 150

        T_base2eef_LEFT.append(puppet_bot_left.arm.get_ee_pose())
        T_base2eef_RIGHT.append(puppet_bot_left.arm.get_ee_pose())
        calib_transform = camera2marker()

        T_camera2marker_LEFT.append(calib_transform[581])
        T_camera2marker_RIGHT.append(calib_transform[150])

    T_eef2marker = np.array(
        [
            [0.0, 0.0, 1.0, 0.025],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.085],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    print(T_camera2marker_LEFT, T_camera2marker_RIGHT)

    # Solve calibration
    solve_calibration(T_eef2marker, T_base2eef_LEFT, T_camera2marker_LEFT, "left_arm")
    solve_calibration(T_eef2marker, T_base2eef_LEFT, T_camera2marker_LEFT, "right_arm")

def calibrate():
    """ A standalone function for experimenting with teleoperation. No data recording. """
    global_node = create_interbotix_global_node()
    puppet_bot_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_left', node=global_node)
    master_bot_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_left', node=global_node)
    puppet_bot_right = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_right', node=global_node)
    master_bot_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_right', node=global_node)
    robot_startup(global_node)

    collect_data(master_bot_left, master_bot_right, puppet_bot_left, puppet_bot_right)

def gotosleep():
    global_node = create_interbotix_global_node()
    puppet_bot_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_left', node=global_node)
    master_bot_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_left', node=global_node)
    puppet_bot_right = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_right', node=global_node)
    master_bot_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_right', node=global_node)
    robot_startup(global_node)

    """ Move all 4 robots to a pose where it is easy to start demonstration """
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot_left.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_left.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_left.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    puppet_bot_right.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_right.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_right.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    torque_on(puppet_bot_left)
    torque_on(master_bot_left)
    torque_on(puppet_bot_right)
    torque_on(master_bot_right)

    # move arms to starting position
    SLEEP_POSE = [[0, -1.7, 1.57, 0, 0, 0], [0, 1.7, -1.57, 0, 0, 0]]

    move_arms([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], SLEEP_POSE * 2, move_time=1.5)

    # move grippers to starting position
    move_grippers([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2, move_time=0.5)


if __name__ == "__main__":
    # calibrate()
    gotosleep()
