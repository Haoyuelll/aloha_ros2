# Aloha setup on Ubuntu 22.04

This project is forked from  [ALOHA: A Low-cost Open-source Hardware System for Bimanual Teleoperation](https://tonyzhaozh.github.io/aloha/).

The original Aloha software is built based on Ubuntu 20.04 with ROS Noetic. This doc serves as a guidance to set up aloha tele-operation with Ubuntu 22.04 and ROS2 Humble.



## Steps to go through
1. [Step 1](#hardware-setup): Hardware setup
2. [Step 2](#optional-ros-pkgs): Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
3. [Step 3](#interbotix-workspace-setup): Set up the robot workspace with `interbotix_ros_toolbox`
4. [Step 4](https://github.com/tonyzhaozh/aloha#hardware-installation): Follow the hardware installation guidance in the original repo
5. [Step 5](#aloha): Set up Aloha
6. [Step 6](#test-and-run): Test and run



## Details and tips

### Hardware setup
- Hardware [doc](https://docs.google.com/document/d/1sgRZmpS7HMcZTPfGy3kAxDrqFMtNNzmK-yVtX5cKYME/edit?pli=1&tab=t.0)
- **Proper** micro-usb cables & 2.5 dc power cables; `lsusb` to check the connection
- Follow the hardware installation in original repo
	- *Dynamixel wizard* can work normally on 22.04, no need to worry 



### Optional ROS pkgs

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-graph
```



### Interbotix workspace setup

```bash
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

**Note that:**
- In step 7 of the original software setup, the corresponding script in ROS2 package is `interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py`. However, the `publish_positions` function in ROS2 pkg is a cpp func. check before you modify the code. 



### Aloha

1. Clone the repo
    ```bash
    cd interbotix_ws/src
    git clone https://github.com/Haoyuelll/aloha_ros2.git
    mv aloha_ros2 aloha # You may change the pkg name to avoid potential errors
    ```
    
    
2. Build ros pkg
  
    ```bash
    # Deactivate to avoid python conflicts if you are using conda
    conda deactivate 
    
    cd interbotix_ws
    
    # if you wish to rebuild the whole workspace
    rm -rf build/ install/ log/
    colcon build
    
    # if you only wish to build aloha pkg
    colcon build --packages-select aloha
    
    source ~/interbotix_ws/install/setup.sh
    
    # or echo it to the shell configuration 
    echo "source ~/interbotix_ws/install/setup.bash" >> ~/.bashrc 
    echo "source ~/interbotix_ws/install/setup.zsh" >> ~/.zshrc 
    ```
  
    Building the whole workspace takes about 30 secs (and instantly for aloha pkg alone).
    Then use `ros2 pkg list | grep aloha` to check pkg availability.




4. Setup conda env

  - In original repo, the env is built based on `python=3.8.10`, while ros2 humble requires `python=3.10.x`.
  - All other dependencies are written in `requirements.txt`

    ```bash
    conda create -n aloha python=3.10
    conda activate aloha
    pip install -r requirements.txt
    ```



### Test and run

1. Check robot status

   - Check usb connection
     - The red light on robot board will flash if connected correctly. 
     - `lsusb` will show 4 robots
   - Check power connection
     - Use `Dynamixel Wizard` to check if all 4 robots are connect correctly (guidance [here](https://github.com/tonyzhaozh/aloha?tab=readme-ov-file#hardware-installation))
       - TL;DR: start the wizard, scan; click any motor will make the light flash on the corresponding robot board. Check which robot is missing.

   

2. Commands to run

   We need three terminals to run the tele-op:

   - In terminal 1, launch the robot and ros broadcast
   	```bash
   	source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
   	ros2 launch aloha 4arms_teleop.launch.py
   	```

   	If any error pop up, check the usb and power connection.

   - In terminal 2, run tele-op for the left group:

   	```bash
   	conda activate aloha
   	export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 # This solve the c++ lib conflict between the py script and ros2
   	cd ~/interbotix_ws/src/aloha/aloha_scripts/ 
   	python3 one_side_teleop.py left
   	```

   - In terminal 3, run tele-op for the right group:

     ```bash
     conda activate aloha
     export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 # This solve the c++ lib conflict between the py script and ros2
     cd ~/interbotix_ws/src/aloha/aloha_scripts/ 
     python3 one_side_teleop.py right
     ```



3. Manipulate **safely**

   - After starting the tele-operation script, all robots will move to an easy-to-manipulate position.

   - (Move the master robot to a proper position if the joints are not mapped correctly)

   - Close the gripper on mater to start tele-op .

      

   **Warnings**:

   - Handles of the master robot should head down.

   - The puppet robot might fall and crash the table if the master is not at good position.

   - Don't do anything with hard force when the torque is on.

   - Shut down the power supply and usb connection when finished.

     