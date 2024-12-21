import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zarr
import numpy as np
from pyk4a import PyK4A, Config, DepthMode, ColorResolution
import threading
import argparse
import os
from termcolor import cprint
from pynput import keyboard
import cv2

class JointStateSubscriber(Node):
    def __init__(self, task_name):
        super().__init__('joint_state_subscriber')

        # Initialize PyK4A for video capture
        self.k4a = PyK4A(Config(color_resolution=ColorResolution.RES_720P, depth_mode=DepthMode.NFOV_UNBINNED))
        self.k4a.start()

        # Subscriber for joint states
        self.left_subscription = self.create_subscription(
            JointState,
            '/puppet_left/joint_states',  # Topic name
            self.joint_state_callback,  # Callback function when a message is received
            10  # Queue size
        )

        self.right_subscription = self.create_subscription(
            JointState,
            '/puppet_right/joint_states',  # Topic name
            self.joint_state_callback,  # Callback function when a message is received
            10  # Queue size
        )

        # Initialize Zarr group
        self.task_name = task_name
        self.zarr_file = f'/home/kyutae/{task_name}.zarr'
        if not os.path.exists(self.zarr_file):
            os.makedirs(self.zarr_file)
        else:
            cprint('Data already exists. Do you want to overwrite? (y/n)', "red")
            user_input = input()
            if user_input == 'y':
                cprint('Overwriting {}'.format(self.zarr_file))
                os.system('rm -rf {}'.format(self.zarr_file))
            else:
                cprint('Exiting', 'red')
                exit(0)

        self.zarr_group = zarr.open(self.zarr_file, mode='w')

        # Episode management
        self.current_episode_idx = 0
        self.recording_active = True
        self.init_new_episode()

        self.get_logger().info(f'Zarr file initialized: {self.zarr_file}')
        self.get_logger().info('JointState Subscriber initialized!')

    def init_new_episode(self):
        """Initialize a new episode in the Zarr file."""
        episode_group_name = f'episode_{self.current_episode_idx}'
        self.episode_group = self.zarr_group.create_group(episode_group_name)
        self.joint_positions = self.episode_group.create_dataset('position', shape=(0, 9), chunks=(1, 9), dtype=float, fill_value=np.nan)
        self.joint_velocities = self.episode_group.create_dataset('velocity', shape=(0, 9), chunks=(1, 9), dtype=float, fill_value=np.nan)
        self.joint_efforts = self.episode_group.create_dataset('effort', shape=(0, 9), chunks=(1, 9), dtype=float, fill_value=np.nan)
        self.video_frames = self.episode_group.create_dataset(
            'video_frames',
            shape=(0, 720, 1280, 3),  # Assuming 720p resolution
            chunks=(1, 720, 1280, 3),
            dtype=np.uint8
        )
        self.get_logger().info(f'Initialized new episode: {episode_group_name}')

    def joint_state_callback(self, msg):
        """Callback for receiving joint state data."""
        if not self.recording_active:
            return  # Ignore incoming data if recording is inactive

        # self.get_logger().info(f'Received joint states: {msg.position}')
        if msg.position:
            # Append joint state data to Zarr
            self.joint_positions.append([msg.position])
            self.joint_velocities.append([msg.velocity])
            self.joint_efforts.append([msg.effort])
            # self.get_logger().info(f'Saved joint state: {msg.position}')

        # Capture a video frame and save to Zarr
        frame = self.k4a.get_capture().color
        if frame is not None:
            color_image_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.video_frames.append([color_image_bgr])  # Append frame
            # self.get_logger().info('Saved video frame.')

    def start_new_episode(self):
        """Start recording a new episode."""
        self.recording_active = True
        self.current_episode_idx += 1
        self.init_new_episode()
        self.get_logger().info('Recording resumed for a new episode.')

    def stop_recording(self):
        """Stop recording for the current episode."""
        self.recording_active = False
        self.get_logger().info('Recording paused.')

    def destroy(self):
        """Clean up resources."""
        self.k4a.stop()
        self.get_logger().info('PyK4A stopped.')


def listen_for_keypress(node):
    """Threaded function to listen for the Right and Left Arrow key presses."""
    def on_press(key):
        if key == keyboard.Key.right:  # Start a new episode
            node.start_new_episode()
        elif key == keyboard.Key.left:  # Stop recording
            node.stop_recording()

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='Task name.', required=True)
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    # Task name provided as part of the script
    task_name = parsed_args.task_name

    # Create the node
    joint_state_subscriber = JointStateSubscriber(task_name=task_name)

    # Start a thread to listen for user input
    input_thread = threading.Thread(target=listen_for_keypress, args=(joint_state_subscriber,))
    input_thread.daemon = True
    input_thread.start()

    # Spin the node to keep it alive and process callbacks
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy()
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
