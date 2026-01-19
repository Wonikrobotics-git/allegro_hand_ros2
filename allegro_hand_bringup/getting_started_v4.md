# Allegro Hand V4 User Guide

## Features

* **ROS 2 Control Integration**: A `ros2_control` `SystemInterface` for the Allegro Hand, enabling seamless integration with the ROS 2 ecosystem.
* **Multiple Control Modes**: Supports both `position` and `effort` command interfaces, allowing for versatile control strategies.
* **Versatile Controllers**: Comes with three pre-configured controllers:
    * **`Position Controller`**: For direct, real-time joint position control, ideal for testing and teleoperation.
    * **Posture Controller**: An action-based controller (`position_controllers/AllegroHandGraspController`) for commanding the hand to a specific multi-joint position goal.
    * **Grasp Controller**: A sophisticated action-based controller (`effort_controllers/AllegroHandGraspController`) that executes pre-defined grasp types (e.g., power grasp, pinch) by sending torque commands calculated by the new `allegro_hand_grasp_library`.
* **RQT Control GUI**: Includes an RQT plugin (`rqt_forward_command_controller`) to easily test and control the hand.
* **Full System Launch**: A single launch file brings up the robot driver, controllers, robot state publisher, and RViz for visualization.

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

## Gazebo Harmonic Installation

see [Gazebo Plugin](../allegro_hand_hardwares/gazebo/plugin/readme.md)


## Quick Start Guide

This guide walks through launching the Allegro Hand driver and testing its various control modes.

### 1. Prerequisites: Activate CAN Interface

Before launching, ensure your CAN device is configured and active. The system is configured to use `can0` at a bitrate of `1000000`.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

> **NOTE**      
> If you are using dual hands, you must also activate can1.
> This example maps the CAN devices as follows:         
> - can0 : right hand     
> - can1 : left hand     


### 2. Launch the System

Launch the main bringup file. This will start the `ros2_control` node, load the controllers, and open RViz. By default, the `allegro_hand_position_controller` (`ForwardCommandController`) is active.

#### Single Hand 

```bash
ros2 launch allegro_hand_bringup allegro_hand.launch.py
```

##### Options 

* hand :  
    - `left` : left hand 
    - `right` : right hand 

* ros2_control_hardware_type : 
    - `mock_components` : 
    - `physical_device` : 


```bash
ros2 launch allegro_hand_bringup allegro_hand.launch.py ros2_control_hardware_type:=mock_components
```

```bash
ros2 launch allegro_hand_bringup allegro_hand_plexus.launch.py hand:=right
```

#### Single Hand (Gazebo)

To simulate the Allegro Hand in Gazebo Harmonic:

```bash
ros2 launch allegro_hand_bringup allegro_hand.gazebo.launch.py
```

##### Options

* `hand` :
    - `left` : left hand
    - `right` : right hand (default)


#### Dual Hand 

##### Options 

* ros2_control_hardware_type : 
    - `mock_components` : 
    - `physical_device` : 

```bash
ros2 launch allegro_hand_bringup allegro_hand_duo.launch.py
```

#### Dual Hand (Gazebo)

To simulate dual Allegro Hands in Gazebo Harmonic:

```bash
ros2 launch allegro_hand_bringup allegro_hand.gazebo.launch.py configuration:=dual_hand
```

#### State Topics 

* /joint_states
* /dynamic_joint_states


#### Check Controllers 

You can check the status of the controllers in a new terminal:
```bash
ros2 control list_controllers
```

> **NOTE**      
> When running **allegro_hand_duo.launch.py**, be aware that the controller names are distinguished for left and right hands.        
> - e.g.)      
>     - allegro_hand_position_controller_l : left         
>     - allegro_hand_position_controller_r : right        

### 3. Test with Position Controller 

#### 3.1. Manual Control with RQT

The `rqt_forward_command_controller` provides a GUI to manually control each joint.

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

This package includes Python scripts that demonstrate more complex, continuous motions. These scripts will automatically activate the required `ForwardCommandController`.

*   **Opening and Releasing Motion**: Repeatedly opens and closes the hand in a fist motion. 

    ```bash
    ros2 run allegro_hand_bringup opening_releasing.py --ros-args -p device_name:=v4 -p controller_name:=allegro_hand_position_controller
    ```

*   **Finger Wave Motion**: Executes a continuous, caterpillar-like wave motion with all fingers.

    ```bash
    ros2 run allegro_hand_bringup finger_wave.py --ros-args -p device_name:=v4 -p controller_name:=allegro_hand_position_controller
    ```


### 4. Test with Posture Controller

This controller allows you to send a goal with a full set of 16 joint positions via a ROS action.

1.  **Switch Controllers**: Deactivate the allegro_hand_position_controller and activate the allegro_hand_posture_controller.
    ```bash
    ros2 control switch_controllers --strict \
      --deactivate allegro_hand_position_controller \
      --activate allegro_hand_posture_controller
    ```

2.  **Send Action Goals**: Use the `ros2 action send_goal` command to send different poses. The action server is `allegro_hand_posture_controller/posture_cmd` and the type is `control_msgs/action/ParallelGripperCommand`.

    * **Home Position:**
        ```bash
        ros2 action send_goal /allegro_hand_posture_controller/posture_cmd control_msgs/action/ParallelGripperCommand '{
          "command": {
            "name": [
              "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",
              "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",
              "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",
              "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33"
            ],
            "position": [
              0.87266,  0.43633,  0.26179,  0.78539,
              0.00000, -0.17453,  0.78539,  0.78539,
              0.00000, -0.17453,  0.78539,  0.78539,
              0.08726, -0.08726,  0.87266,  0.78539
            ]
          }
        }'
        ```

    * **Scissors Position:**
        ```bash
        ros2 action send_goal /allegro_hand_posture_controller/posture_cmd control_msgs/action/ParallelGripperCommand '{
        "command": {
            "name": [
            "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",
            "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",
            "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",
            "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33"
            ],
            "position": [
            0.876451688853475, 1.2216305543672838, 1.4264144404001213, 1.6641368528133413,
            0.09960998262586793, -0.002163044412170033, -0.033398812297706866, 0.07304468498730649,
            0.144526588225768, 0.029139390304994982, -0.0029649433088131255, -0.018537053421023006,
            0.07774173357748533, 1.6961666504428443, 1.4743204682667252, 0.7695174550487038
            ]
        }
        }'
        ```

    * **Rock Position:**
        ```bash
        ros2 action send_goal /allegro_hand_posture_controller/posture_cmd control_msgs/action/ParallelGripperCommand '{
        "command": {
            "name": [
            "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",
            "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",
            "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",
            "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33"
            ],
            "position": [
            1.3709802140584426, 0.7105499544163194, 1.3933791849172252, 1.5466036317282827,
            0.11362072888585825, 1.614340850824676,  1.579071359681267,  1.4074881893862177,
            0.0673978006577092,  1.4359690049683838, 1.2861311741072952, 0.4064290489333438,
            0.1565676712487908,  1.3445579256140634, 1.1982622668331682, 0.7237147249736905
            ]
        }
        }'
        ```
       
### 5. Test with Grasp Controller
This controller uses the internal `allegro_hand_grasp_library` to execute named, pre-defined grasps by controlling joint torques.

1.  **Switch Controllers**: Deactivate the allegro_hand_posture_controller and activate allegro_hand_grasp_controller.
    ```bash
    ros2 control switch_controllers --strict \
      --deactivate allegro_hand_posture_controller \
      --activate allegro_hand_grasp_controller
    ```

2.  **Send Action Goals**: Use the `ros2 action send_goal` command to send grasp commands. The action server is `/allegro_hand_grasp_controller/grasp_cmd` and the type is `allegro_hand_control_msgs/action/GraspCommand`.

    * **Ready Position:**
        ```bash
        ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "ready" }'
        ```

    * **Four-Finger Grasp:**
        ```bash
        ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "grasp_4" }'
        ```

    * **Index-Thumb Pinch:**
        ```bash
        ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "pinch_it" }'
        ```
       
    * **Envelop (Power) Grasp:**
        ```bash
        ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "envelop" }'
        ```
       

### 6. Return to Default Controller

To return to the manual joint control GUI, switch back to the allegro_hand_position_controller.
```bash
ros2 control switch_controllers --strict \
  --deactivate allegro_hand_grasp_controller \
  --activate allegro_hand_position_controller
```

## Hardware Interface Detail and Tuning Guide

This document provides detailed information on the `allegro_hand_v4_hardware` package, which serves as the `ros2_control` hardware interface for the Allegro Hand V4. It covers key features, parameter configuration, and a comprehensive guide on how to check and dynamically tune PD control gains and other parameters in real-time using ROS 2 tools. ([link](../allegro_hand_hardwares/v4/hardware/readme.md))
