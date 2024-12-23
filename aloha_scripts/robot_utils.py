import numpy as np
import time
import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from cv_bridge import CvBridge

DT = 0.01  # Assumed value for DT, make sure to define it if needed

class ImageRecorder(Node):
    def __init__(self, init_node=True, is_debug=False):
        super().__init__('image_recorder')
        self.is_debug = is_debug
        self.bridge = CvBridge()
        self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
        self.camera_subscribers = {}

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
            elif cam_name == 'cam_low':
                callback_func = self.image_cb_cam_low
            elif cam_name == 'cam_left_wrist':
                callback_func = self.image_cb_cam_left_wrist
            elif cam_name == 'cam_right_wrist':
                callback_func = self.image_cb_cam_right_wrist
            else:
                raise NotImplementedError
            # ROS2 way to create subscribers
            self.camera_subscribers[cam_name] = self.create_subscription(
                Image,
                f"/usb_{cam_name}/image_raw",
                callback_func,
                10
            )
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))

        time.sleep(0.5)

    def image_cb(self, cam_name, data):
        setattr(self, f'{cam_name}_image', self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough'))
        setattr(self, f'{cam_name}_secs', data.header.stamp.sec)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nanosec)
        if self.is_debug:
            getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.sec + data.header.stamp.sec * 1e-9)

    def image_cb_cam_high(self, data):
        cam_name = 'cam_high'
        return self.image_cb(cam_name, data)

    def image_cb_cam_low(self, data):
        cam_name = 'cam_low'
        return self.image_cb(cam_name, data)

    def image_cb_cam_left_wrist(self, data):
        cam_name = 'cam_left_wrist'
        return self.image_cb(cam_name, data)

    def image_cb_cam_right_wrist(self, data):
        cam_name = 'cam_right_wrist'
        return self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = dict()
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            self.get_logger().info(f'{cam_name} {image_freq=:.2f}')
        self.get_logger().info('')

# --
class NewRecorder(Node):
    def __init__(self, side):
        super().__init__('joint_state_subscriber')

        # Create a subscriber to listen to the /puppet_left/joint_states topic
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            f"/puppet_{side}/joint_states",
            self.puppet_state_cb,
            10
        )
        self.arm_command_subscriber = self.create_subscription(
            JointGroupCommand,
            f"/puppet_{side}/commands/joint_group",
            self.puppet_arm_commands_cb,
            10
        )
        self.gripper_command_subscriber = self.create_subscription(
            JointSingleCommand,
            f"/puppet_{side}/commands/joint_single",
            self.puppet_gripper_commands_cb,
            10
        )
        # Log a message indicating that the subscriber is active
        self.get_logger().info('JointState Subscriber has been initialized!')


    def joint_state_callback(self, msg):
        # # This function will be called when a message is received on the topic
        # self.get_logger().info(f'Received joint states: {msg.position}')
        # # You can process the JointState data here, e.g., logging the joint positions
        # # Example: Log joint positions (msg.position is a list of joint positions)
        # # For simplicity, here we log the first joint position:
        # if msg.position:
        #     self.get_logger().info(f'First joint position: {msg.position[0]}')
        # else:
        #     self.get_logger().info('No joint positions received.')
        return True
    
    def puppet_state_cb(self, data):
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data

    def puppet_arm_commands_cb(self, data):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def puppet_gripper_commands_cb(self, data):
        self.get_logger().info(f"Received JointState message: {data}")
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())
# --

class Recorder(Node):
    def __init__(self, side, init_node=True, is_debug=False):
        super().__init__('recorder')
        self.secs = None
        self.nsecs = None
        self.qpos = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        # ROS2 subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            f"/puppet_{side}/joint_states",
            self.puppet_state_cb,
            10
        )
        self.arm_command_subscriber = self.create_subscription(
            JointGroupCommand,
            f"/puppet_{side}/commands/joint_group",
            self.puppet_arm_commands_cb,
            10
        )
        self.gripper_command_subscriber = self.create_subscription(
            JointSingleCommand,
            f"/puppet_{side}/commands/joint_single",
            self.puppet_gripper_commands_cb,
            10
        )

        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        time.sleep(0.1)

    def puppet_state_cb(self, data):
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    def puppet_arm_commands_cb(self, data):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def puppet_gripper_commands_cb(self, data):
        self.get_logger().info(f"Received JointState message: {data}")
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        self.get_logger().info(f'{joint_freq=:.2f}')
        self.get_logger().info(f'{arm_command_freq=:.2f}')
        self.get_logger().info(f'{gripper_command_freq=:.2f}')

# Assuming the functions for moving arms, grippers, and setup methods remain unchanged

def get_arm_joint_positions(bot):
    return bot.arm.core.joint_states.position[:6]

def get_arm_gripper_positions(bot):
    joint_position = bot.gripper.core.joint_states.position[6]
    return joint_position

def move_arms(bot_list, target_pose_list, move_time=1):
    num_steps = int(move_time / DT)
    curr_pose_list = [get_arm_joint_positions(bot) for bot in bot_list]
    traj_list = [np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            bot.arm.set_joint_positions(traj_list[bot_id][t], blocking=False)
        time.sleep(DT)

def move_grippers(bot_list, target_pose_list, move_time):
    gripper_command = JointSingleCommand(name="gripper")
    num_steps = int(move_time / DT)
    curr_pose_list = [get_arm_gripper_positions(bot) for bot in bot_list]
    traj_list = [np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            gripper_command.cmd = traj_list[bot_id][t]
            bot.gripper.core.pub_single.publish(gripper_command)
        time.sleep(DT)

def setup_puppet_bot(bot):
    bot.core.robot_reboot_motors("single", "gripper", True)
    bot.core.robot_set_operating_modes("group", "arm", "position")
    bot.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    torque_on(bot)

def setup_master_bot(bot):
    bot.core.robot_set_operating_modes("group", "arm", "pwm")
    bot.core.robot_set_operating_modes("single", "gripper", "current_based_position")
    torque_off(bot)

def set_standard_pid_gains(bot):
    bot.core.robot_set_motor_registers("group", "arm", 'Position_P_Gain', 800)
    bot.core.robot_set_motor_registers("group", "arm", 'Position_I_Gain', 0)

def set_low_pid_gains(bot):
    bot.core.robot_set_motor_registers("group", "arm", 'Position_P_Gain', 100)
    bot.core.robot_set_motor_registers("group", "arm", 'Position_I_Gain', 0)

def torque_off(bot):
    bot.core.robot_torque_enable("group", "arm", False)
    bot.core.robot_torque_enable("single", "gripper", False)

def torque_on(bot):
    bot.core.robot_torque_enable("group", "arm", True)
    bot.core.robot_torque_enable("single", "gripper", True)
