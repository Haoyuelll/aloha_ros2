import time
import IPython

e = IPython.embed

import rclpy
from rclpy.node import Node

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import (
    MASTER2PUPPET_JOINT_FN,
    DT,
    START_ARM_POSE,
    MASTER_GRIPPER_JOINT_MID,
    PUPPET_GRIPPER_JOINT_CLOSE,
    SLEEP_POSE,
)
from robot_utils import (
    torque_on,
    torque_off,
    move_arms,
    move_grippers,
    get_arm_gripper_positions,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)


class Leave(Node):
    def __init__(self):
        super().__init__("SLEEP")
        
    def run(self, master_left, master_right, puppet_left, puppet_right):
        move_arms(
            [
                master_left,
                puppet_left,
                master_right,
                puppet_right,
            ],
            SLEEP_POSE * 2,
            move_time=1.5,
        )
        move_grippers(
            [
                master_left,
                puppet_left,
                master_right,
                puppet_right,
            ],
            [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2,
            move_time=0.5,
        )

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.start = True
        self.get_prepared()

    def run(self):
        self.get_logger().info("Start teleop")

        try:
            while self.start:
                self.pub_teleop()
                time.sleep(DT)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received (Ctrl+C).")
            self.start = False
            # sleep(
            #     self.master_bot_left,
            #     self.master_bot_right,
            #     self.puppet_bot_left,
            #     self.puppet_bot_right,
            # )

    def pub_teleop(self):
        # Left side
        master_left = self.master_bot_left.core.joint_states.position[:6]
        # -- flip the joint angles
        # master_left[1] = -master_left[1]
        # master_left[2] = -master_left[2]
        self.puppet_bot_left.arm.set_joint_positions(master_left, blocking=False)

        # Right side
        master_right = self.master_bot_right.core.joint_states.position[:6]
        # -- flip the joint angles
        # master_right[1] = -master_right[1]
        # master_right[2] = -master_right[2]
        self.puppet_bot_right.arm.set_joint_positions(master_right, blocking=False)

        # sync gripper positions
        master_left_gripper_joint = self.master_bot_left.core.joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_left_gripper_joint)
        self.gripper_command.cmd = puppet_gripper_joint_target
        self.puppet_bot_left.gripper.core.pub_single.publish(self.gripper_command)

        # Right side
        master_right_gripper_joint = self.master_bot_right.core.joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_right_gripper_joint)
        self.gripper_command.cmd = puppet_gripper_joint_target
        self.puppet_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def get_prepared(self):
        global_node = create_interbotix_global_node()
        self.puppet_bot_left = InterbotixManipulatorXS(
            robot_model="vx300s",
            group_name="arm",
            gripper_name="gripper",
            robot_name="puppet_left",
            node=global_node,
        )
        self.master_bot_left = InterbotixManipulatorXS(
            robot_model="wx250s",
            group_name="arm",
            gripper_name="gripper",
            robot_name="master_left",
            node=global_node,
        )
        self.puppet_bot_right = InterbotixManipulatorXS(
            robot_model="vx300s",
            group_name="arm",
            gripper_name="gripper",
            robot_name="puppet_right",
            node=global_node,
        )
        self.master_bot_right = InterbotixManipulatorXS(
            robot_model="wx250s",
            group_name="arm",
            gripper_name="gripper",
            robot_name="master_right",
            node=global_node,
        )
        robot_startup(global_node)

        """ Move all 4 robots to a pose where it is easy to start demonstration """
        # reboot gripper motors, and set operating modes for all motors
        self.puppet_bot_left.core.robot_reboot_motors("single", "gripper", True)
        self.puppet_bot_left.core.robot_set_operating_modes("group", "arm", "position")
        self.puppet_bot_left.core.robot_set_operating_modes(
            "single", "gripper", "current_based_position"
        )
        self.master_bot_left.core.robot_set_operating_modes("group", "arm", "position")
        self.master_bot_left.core.robot_set_operating_modes(
            "single", "gripper", "position"
        )
        # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

        self.puppet_bot_right.core.robot_reboot_motors("single", "gripper", True)
        self.puppet_bot_right.core.robot_set_operating_modes("group", "arm", "position")
        self.puppet_bot_right.core.robot_set_operating_modes(
            "single", "gripper", "current_based_position"
        )
        self.master_bot_right.core.robot_set_operating_modes("group", "arm", "position")
        self.master_bot_right.core.robot_set_operating_modes(
            "single", "gripper", "position"
        )
        # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

        torque_on(self.puppet_bot_left)
        torque_on(self.master_bot_left)
        torque_on(self.puppet_bot_right)
        torque_on(self.master_bot_right)

        # move arms to starting position
        start_arm_qpos = START_ARM_POSE[:6]
        combined_pos = [[p for p in start_arm_qpos]]
        combined_pos.append([p for p in start_arm_qpos])
        # -- flip the joint angles
        # combined_pos[1][1] = -combined_pos[1][1]
        # combined_pos[1][2] = -combined_pos[1][2]
        move_arms(
            [
                self.master_bot_left,
                self.puppet_bot_left,
                self.master_bot_right,
                self.puppet_bot_right,
            ],
            combined_pos * 2,
            move_time=1.5,
        )
        # move_arms([master_bot_left, puppet_bot_left], combined_pos, move_time=1.5)
        # move grippers to starting position
        move_grippers(
            [
                self.master_bot_left,
                self.puppet_bot_left,
                self.master_bot_right,
                self.puppet_bot_right,
            ],
            [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2,
            move_time=0.5,
        )
        # move_grippers([master_bot_left, puppet_bot_left], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)

        # press gripper to start data collection
        # disable torque for only gripper joint of master robot to allow user movement
        self.master_bot_left.core.robot_torque_enable("single", "gripper", False)
        self.master_bot_right.core.robot_torque_enable("single", "gripper", False)
        print(f"Close the gripper to start")
        close_thresh = -0.3
        pressed = False
        while not pressed:
            gripper_pos_left = get_arm_gripper_positions(self.master_bot_left)
            gripper_pos_right = get_arm_gripper_positions(self.master_bot_right)
            if (gripper_pos_left < close_thresh) and (
                gripper_pos_right < close_thresh
            ):  # changed to OR temporarily
                pressed = True
            time.sleep(DT / 10)
        torque_off(self.master_bot_left)
        torque_off(self.master_bot_right)
        self.gripper_command = JointSingleCommand(name="gripper")

        print("Ready to go!")


def sleep(master_left, master_right, puppet_left, puppet_right):
    move_arms(
        [
            master_left,
            puppet_left,
            master_right,
            puppet_right,
        ],
        SLEEP_POSE * 2,
        move_time=1.5,
    )
    move_grippers(
        [
            master_left,
            puppet_left,
            master_right,
            puppet_right,
        ],
        [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2,
        move_time=0.5,
    )


def gotosleep():
    global_node = create_interbotix_global_node()
    puppet_bot_left = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        robot_name=f"puppet_left",
        node=global_node,
    )
    master_bot_left = InterbotixManipulatorXS(
        robot_model="wx250s",
        group_name="arm",
        gripper_name="gripper",
        robot_name=f"master_left",
        node=global_node,
    )
    puppet_bot_right = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        robot_name=f"puppet_right",
        node=global_node,
    )
    master_bot_right = InterbotixManipulatorXS(
        robot_model="wx250s",
        group_name="arm",
        gripper_name="gripper",
        robot_name=f"master_right",
        node=global_node,
    )
    robot_startup(global_node)

    """ Move all 4 robots to a pose where it is easy to start demonstration """
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot_left.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_left.core.robot_set_operating_modes(
        "single", "gripper", "current_based_position"
    )
    master_bot_left.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_left.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    puppet_bot_right.core.robot_reboot_motors("single", "gripper", True)
    puppet_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_right.core.robot_set_operating_modes(
        "single", "gripper", "current_based_position"
    )
    master_bot_right.core.robot_set_operating_modes("group", "arm", "position")
    master_bot_right.core.robot_set_operating_modes("single", "gripper", "position")
    # puppet_bot_left.core.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    torque_on(puppet_bot_left)
    torque_on(master_bot_left)
    torque_on(puppet_bot_right)
    torque_on(master_bot_right)

    # move arms to starting position
    # SLEEP_POSE = [[0, -1.7, 1.57, 0, 0, 0], [0, 1.7, -1.57, 0, 0, 0]]

    move_arms(
        [master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right],
        SLEEP_POSE * 2,
        move_time=1.5,
    )

    # move grippers to starting position
    move_grippers(
        [master_bot_left, puppet_bot_left, master_bot_right, puppet_bot_right],
        [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE] * 2,
        move_time=0.5,
    )


def main(args=None):
    rclpy.init(args=args)

    node = TeleopNode()
    master_left, master_right, puppet_left, puppet_right = (
        node.master_bot_left,
        node.master_bot_right,
        node.puppet_bot_left,
        node.puppet_bot_right,
    )

    node.run()
    
    rclpy.init(args=args)
    leaving = Leave()
    leaving.run()


if __name__ == "__main__":
    main()
