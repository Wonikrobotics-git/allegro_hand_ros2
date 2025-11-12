# Allegro Hand Position-Effort Controller

This package provides an advanced, multi-interface controller for the Allegro Hand, designed for use with the `ros2_control` framework. 

This controller provides both `position` and `effort` command interfaces. Here, 'effort' refers to the maximum torque for each joint motor. While a standard position controller allows adjusting the maximum motor torque as a constant parameter, the Position-Effort Controller enables real-time control of both position and the maximum torque for each motor. This makes it highly applicable for a variety of grasping motions.

It is primarily intended for use with the **Allegro Hand Plexus**.


## Controller

This package provides one controller plugin that implements `controller_interface::ControllerInterface`.

> **Note:** This package is designed for and tested with **Allegro Hand Plexus**.


--- 

### Position-Effort Controller

* **Plugin Name**: `position_effort_controllers/AllegroHandPositionEffortController`
* **Description**: This controller exposes both `position` and `effort` command interfaces for all 16 joints. It allows external clients to send either position targets or effort (torque) commands to the hand simultaneously. The controller prioritizes the last received command type for each joint, making it highly versatile for tasks that require switching between precise positioning and force-based interaction.
* **Command Topics**:
    * `~/commands` (for position control, `std_msgs::msg::Float64MultiArray`)
    * `~/commands_effort` (for effort control, `std_msgs::msg::Float64MultiArray`)

---

## Controller Parameters

The controller uses the following parameters, which can be configured in the controller's YAML configuration file.

| Parameter | Type | Description |
| :--- | :--- | :--- |
| **`joints`** | `string[]` | A list of 16 joint names to be managed by the controller. |
| **`interface_name`** | `string` | The primary command interface type. Should be set to `position`. |

---

## Usage

This controller is loaded as a plugin by the `ros2_control` controller manager.

### Configuration Example

Below is an example snippet from a `ros2_controllers.yaml` file for loading the controller. Note that while the controller handles both position and effort, the primary `interface_name` is declared as `position`.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 333  # Hz 

    allegro_hand_position_effort_controller:
      type: position_effort_controllers/AllegroHandPositionEffortController
      joints:
        - ah_joint00
        # ... (all 16 joints listed)
        - ah_joint33
      interface_name: "position"
```

### Sending Commands

You can test the controller from the terminal using the `ros2 topic pub` command.

#### Position Control Example

Send a command to move all joints to a position of `0.5` radians.

```bash
ros2 topic pub /allegro_hand_position_effort_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 16
    stride: 1
  data_offset: 0
data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

#### Effort (Torque) Control Example

Send a command to apply `0.1` Nm of torque to all joints.

```bash
ros2 topic pub /allegro_hand_position_effort_controller/commands_effort std_msgs/msg/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 16
    stride: 1
  data_offset: 0
data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"
```