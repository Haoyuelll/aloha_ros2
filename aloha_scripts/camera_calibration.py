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
        
    LEFT_ARM_POSE = [[[-0.3, -0.96, 1.16, 0, 0.3, 0], [-0.3, -0.96, 1.16, 0, 0.3, 0]]]
    RIGHT_ARM_POSE = [[[0.5, -0.96, 1.16, 0, -0.3, 0], [0.5, -0.96, 1.16, 0, -0.3, 0]]]

    LEFT_ARM_POSE.append([[-0.3, -0.96, 1.3, 0, 0.3, 0], [-0.3, -0.96, 1.3, 0, 0.3, 0]])
    RIGHT_ARM_POSE.append([[0.5, -0.96, 1.4, 0, -0.3, 0], [0.5, -0.96, 1.4, 0, -0.3, 0]])
    
    LEFT_ARM_POSE.append([[-0.2, -0.3, 1.16, 0, 0.3, 0], [-0.2, -0.3, 1.16, 0, 0.3, 0]])
    RIGHT_ARM_POSE.append([[0.5, -0.96, 1.16, -0.3, -0.7, 0], [0.5, -0.96, 1.16, -0.3, -0.7, 0]])
    
    
    # Right
    RIGHT_ARM_POSE.extend([
         [[-0.016873789951205254, 0.003067961661145091, 0.2546408176422119, -0.07516506314277649, -0.6120583415031433, 0.061359234154224396]] * 2,
        [[-0.1288543939590454, -0.8068739175796509, 0.7761942744255066, -0.30066025257110596, -0.1702718734741211, -0.20555342733860016]] * 2,
        [[-0.21475732326507568, -0.4325825870037079, 0.4325825870037079, -0.2960582971572876, -0.2715145945549011, 0.05522330850362778]] * 2,
        [[-0.08130098134279251, -0.34514567255973816, 0.7562525272369385, -0.2040194571018219, -0.6458059549331665, -0.004601942375302315]] * 2,
        [[0.08590292930603027, -0.21629129350185394, 0.7056311964988708, -0.19634954631328583, -0.6504078507423401, -0.023009711876511574]] * 2,
        [[-0.10277671366930008, -0.43104860186576843, 0.8774370551109314, -0.20248547196388245, -0.7424467206001282, -0.02147573232650757]] * 2,
        [[0.2684466540813446, -0.3635534644126892, 0.7869321703910828, -0.2040194571018219, -0.5844466686248779, -0.03681553900241852]] * 2,
    ])
    
    # Left
    LEFT_ARM_POSE.extend([
        [[-0.22856314480304718, 0.11504856497049332, 0.1702718734741211, -0.04601942375302315, 0.11044661700725555, 0.04295146465301514]] * 2,
        [[-0.23009712994098663, 0.11198060214519501, 0.5307573676109314, -0.04908738657832146, -0.22856314480304718, -0.052155349403619766]] * 2,
        [[0.08130098134279251, 0.003067961661145091, 0.31446605920791626, -0.03681553900241852, 0.09357283264398575, 0.11811652034521103]] * 2,
        [[0.07976700365543365, -0.42951464653015137, 0.7792622447013855, -0.03988350182771683, 0.3067961633205414, 0.2791845202445984]] * 2,
        [[0.30833014845848083, -0.32520392537117004, 0.6258642077445984, -0.03528155758976936, 0.4924078583717346, 0.6289321184158325]] * 2,
        [[-0.01840776950120926, -0.3374757766723633, 0.7654564380645752, -0.052155349403619766, 0.14419420063495636, 0.3221359848976135]] * 2,
        [[-0.15033012628555298, -0.34514567255973816, 0.7025632262229919, -0.03988350182771683, 0.6320000886917114, 0.4893398880958557]] * 2,
    ])
    
    # Move arms so that the aruco marker is detectable 
    for i in range(len(LEFT_ARM_POSE)):
        move_arms([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], LEFT_ARM_POSE[i] + RIGHT_ARM_POSE[i], move_time=1.5)

        print('Left End-effector pose: ', puppet_bot_left.arm.get_ee_pose()) # 581
        print('Right End-effector pose: ', puppet_bot_right.arm.get_ee_pose()) # 150

        calib_transform = camera2marker()

        try:
            T_camera2marker_LEFT.append(calib_transform[581])
            T_base2eef_LEFT.append(puppet_bot_left.arm.get_ee_pose())

        except KeyError:
            print("Unable to find key 581")
        try:
            T_camera2marker_RIGHT.append(calib_transform[150])
            T_base2eef_RIGHT.append(puppet_bot_left.arm.get_ee_pose())
        except KeyError:
            print("Unable to find key 150")

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
    solve_calibration(T_eef2marker, T_base2eef_RIGHT, T_camera2marker_RIGHT, "right_arm")

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
    # SLEEP_POSE = [[0, -1.7, 1.57, 0, 0, 0], [0, 1.7, -1.57, 0, 0, 0]]
    SLEEP_POSE = [[0, -1.7, 1.57, 0, 0, 0], [0, -1.7, 1.57, 0, 0, 0]]

    move_arms([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], SLEEP_POSE * 2, move_time=1.5)

    # move grippers to starting position
    move_grippers([master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2, move_time=0.5)


if __name__ == "__main__":
    # try:
    #     calibrate()
    #     print("---- Calibration finished ----")
    # except KeyboardInterrupt:
    #     print("---- Keyboard interupt received ----")
    
    gotosleep()
    