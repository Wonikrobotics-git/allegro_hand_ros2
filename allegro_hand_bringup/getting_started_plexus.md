# Allegro Hand Plexus User Guide

## Features

* **ROS 2 Control Integration**: A `ros2_control` `SystemInterface` for the Allegro Hand Plexus, enabling seamless integration with the ROS 2 ecosystem.
* **Hybrid Control Mode**: Supports a unique `position_effort_controller` that accepts both `position` and `effort` commands simultaneously on different topics.
* **Versatile Controller**: Comes with one primary pre-configured controller:
    * **`Position-Effort Controller`**: For direct, real-time joint control via either position or effort (torque) commands. This is ideal for advanced applications requiring dynamic switching between position and force control.
* **Full System Launch**: A single launch file brings up the robot driver, controller, robot state publisher, and RViz for visualization.

## Installation

1.  **Clone all required packages into your workspace:**
    ```bash
    # (Assuming your workspace is at ~/ros2_ws/src)
    cd ~/ros2_ws/src
    # git clone ... (clone this package and all other allegro_hand ROS 2 packages)
    ```

2.  **Install dependencies:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the packages:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

4.  **Source the environment:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Quick Start Guide

This guide walks through launching the Allegro Hand Plexus driver and testing its control modes.

### 1. Prerequisites: Activate CAN Interface

Before launching, ensure your CAN device is configured and active. The system is configured to use `can0` at a bitrate of `1000000`.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

> **NOTE**      
> If you are using dual hands, you must also activate `can1`.
> This example maps the CAN devices as follows:         
> - can0 : right hand     
> - can1 : left hand     


### 2. Launch the System

Launch the main bringup file for the Plexus hand. This will start the `ros2_control` node, load the `allegro_hand_position_effort_controller`, and open RViz.

#### Single Hand 

```bash
ros2 launch allegro_hand_bringup allegro_hand_plexus.launch.py
```

##### Options 

* `hand`:  
    - `left`: left hand 
    - `right`: right hand (default)
* `ros2_control_hardware_type`: 
    - `mock_components`: Use a simulated hardware interface for testing without a physical device.
    - `physical_device`: Use the real hardware interface. (default)

```bash
# Launch with a mock hardware interface
ros2 launch allegro_hand_bringup allegro_hand_plexus.launch.py ros2_control_hardware_type:=mock_components

# Launch the left hand
ros2 launch allegro_hand_bringup allegro_hand_plexus.launch.py hand:=left
```


#### State Topics 

* `/joint_states`: Publishes the position, velocity, and effort of all joints.

#### Check Controllers 

You can check the status of the controllers in a new terminal:
```bash
ros2 control list_controllers
```

### 3. Test with Position Controller


#### 3.1. Manual Control with RQT

The `rqt_forward_command_controller` provides a GUI to manually control each joint.

<img src="./images/rqt_plexus.png" width="600">

1.  Open RQT:
    ```bash
    rqt
    ```
2.  In the RQT window, select **Plugins** > **Robot Tools** > **Forward Command Controller**.
3.  In the plugin's GUI:
    * Select the `controller manager ns` (usually `/`).
    * Select `allegro_hand_position_controller` from the `controller` dropdown.
    * Click the central power button to enable "Control Mode".
    * You can now move the sliders to control each joint of the Allegro Hand in real-time.

> **NOTE**     
> If you don't see `Forward Command Controller` in the list of rqt plugins, you need to run rqt_forward_command_controller once in the terminal as shown below.    
> ```bash
> $ source ${YOUR_WORKSPACE}/install/setup.sh
> $ ros2 run rqt_forward_command_controller rqt_forward_command_controller --force-discover
> ```


#### 3.2 Test with Python Motion Scripts

This package includes Python scripts that demonstrate more complex, continuous motions.  

These scripts can be used with both the `allegro_hand_position_controller` and the `allegro_hand_position_effort_controller`.

*   **Opening and Releasing Motion**: Repeatedly opens and closes the hand in a fist motion. 

    ```bash
    ros2 run allegro_hand_bringup opening_releasing.py --ros-args -p device_name:=plexus -p controller_name:=allegro_hand_position_controller
    ```

*   **Finger Wave Motion**: Executes a continuous, caterpillar-like wave motion with all fingers.

    ```bash
    ros2 run allegro_hand_bringup finger_wave.py --ros-args -p device_name:=plexus -p controller_name:=allegro_hand_position_effort_controller
    ```


## Hardware Interface Details

This document provides detailed information on the `allegro_hand_plexus_hardware` package, which serves as the `ros2_control` hardware interface for the Allegro Hand Plexus. It covers key features and parameter configuration. (link)