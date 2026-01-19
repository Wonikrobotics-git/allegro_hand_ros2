#!/usr/bin/env python3
#
"""
A ROS 2 node to command the Allegro Hand to perform a continuous,
caterpillar-like finger wave motion.

This node uses a sine wave with phase offsets for each finger to create a
smooth, continuous wave. It activates the specified controller upon startup
and can be configured via a ROS parameter.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from common import activate_controller


class MotionGeneratorV4:
    """
    Generates the joint commands for a continuous finger wave motion.
    """

    def __init__(self):
        # Parameters for the sine wave oscillation.
        self.frequency = 5.0  # rad/s, controls speed of the wave
        # Phase offsets create the delayed motion between fingers, forming the wave. 
        self.phase_offsets = {
            "THUMB": 0.0,
            "MIDDLE": np.pi / 2,
            "INDEX": np.pi,
            "RING": 3 * np.pi / 2,
        }

        # Define the motion range (open and closed poses) for the thumb.
        thumb_open_pose = np.array([0.4, 0.1, 0.0, 0.0])
        # The closed pose is reduced to prevent collision with the index finger.
        thumb_closed_pose = np.array([1.2, 0.4, 1.0, 0.6])
        self.thumb_amplitude = (thumb_closed_pose - thumb_open_pose) / 2.0
        self.thumb_offset = (thumb_closed_pose + thumb_open_pose) / 2.0

        # Define the motion range for the other three fingers.
        finger_open_pose = np.array([0.0, 0.0, 0.0, 0.0])
        finger_closed_pose = np.array([0.15, 1.45, 1.5, 0.15])
        self.finger_amplitude = (finger_closed_pose - finger_open_pose) / 2.0
        self.finger_offset = (finger_closed_pose + finger_open_pose) / 2.0

        # Store the initial base position to return to on shutdown.
        self.base_position = np.array(
            [
                # Thumb (neutral position)
                0.5,
                0.2,
                0.0,
                0.0,
                # Index, Middle, Ring (open)
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )

    def get_command(self, t):
        """Calculates and returns the joint command for a given time t."""
        current_command = np.zeros(16)

        # Calculate joint positions for each finger based on a sine wave with a phase offset
        for i, finger in enumerate(["THUMB", "INDEX", "MIDDLE", "RING"]):
            phase = self.phase_offsets[finger]
            oscillation = np.sin(self.frequency * t + phase)

            if finger == "THUMB":
                finger_pose = self.thumb_offset + self.thumb_amplitude * oscillation
            else:
                finger_pose = self.finger_offset + self.finger_amplitude * oscillation

            start_idx, end_idx = 4 * i, 4 * i + 4
            current_command[start_idx:end_idx] = finger_pose
        return current_command

    def get_velocity_command(self, t):
        """Calculates and returns the joint velocity command for a given time t."""
        current_velocity = np.zeros(16)

        # Calculate joint velocities for each finger
        # Velocity is the time derivative of position: d/dt(sin(wt+p)) = w*cos(wt+p)
        for i, finger in enumerate(["THUMB", "INDEX", "MIDDLE", "RING"]):
            phase = self.phase_offsets[finger]
            oscillation_derivative = self.frequency * np.cos(self.frequency * t + phase)

            if finger == "THUMB":
                finger_velocity = self.thumb_amplitude * oscillation_derivative
            else:
                finger_velocity = self.finger_amplitude * oscillation_derivative

            start_idx, end_idx = 4 * i, 4 * i + 4
            current_velocity[start_idx:end_idx] = finger_velocity
        return current_velocity


class MotionGeneratorPlexus:
    """
    Generates the joint commands for a continuous finger wave motion,
    specifically tailored for the Allegro Hand Plexus joint limits.
    """

    def __init__(self):
        # Parameters for the sine wave oscillation.
        self.frequency = 8.0  # rad/s, controls speed of the wave

        # Amplitudes for each joint. Default is 20 degrees.
        # The order is Thumb, Index, Middle, Ring.
        amplitude_rad_default = np.pi / 7.2  # 25 degrees
        amplitudes = np.full(16, amplitude_rad_default)
        # Reduce thumb amplitude by half to prevent collision
        amplitudes[0] /= 4.0
        amplitudes[1] /= 2.0  # Reduce second thumb joint amplitude
        amplitudes[2] /= 2.0  # Reduce third thumb joint amplitude
        # Set a small amplitude for the first joint of I, M, R fingers
        amplitudes[4] = np.pi / 18.0  # 10 degrees for Index finger, joint 0 (joint10)
        amplitudes[8] = np.pi / 18.0  # 10 degrees for Middle finger, joint 0 (joint20)
        amplitudes[12] = np.pi / 18.0  # 10 degrees for Ring finger, joint 0 (joint30)
        self.amplitudes = np.array(amplitudes)

        # Center positions are derived from the joint limits in the URDF.
        # This ensures the motion is safe and within the hardware's capabilities.
        # The order is Thumb, Index, Middle, Ring.
        self.center_positions = np.array(
            [
                # Thumb (joint00-03)
                1.745 / 4.0,
                (-0.349 + 1.169) / 2.0,
                1.169 / 2.0,
                1.570 / 2.0,
                # Index Finger (joint10-13)
                0.0,
                1.169 / 2.0,
                1.169 / 2.0,
                1.169 / 2.0,
                # Middle Finger (joint20-23)
                0.0,
                1.169 / 2.0,
                1.169 / 2.0,
                1.169 / 2.0,
                # Ring Finger (joint30-33)
                0.0,
                1.169 / 2.0,
                1.169 / 2.0,
                1.169 / 2.0,
            ]
        )

        # Phase offsets create the delayed motion between fingers, forming the wave.
        self.phase_offsets = {
            "THUMB": 3 * np.pi / 2,
            "INDEX": 0.0,
            "MIDDLE": np.pi / 2,
            "RING": np.pi,
        }

        # Store the initial base position to return to on shutdown.
        self.base_position = np.array(
            [
                # Thumb (neutral position)
                0.87,
                0.4,
                0.0,
                0.0,
                # Index, Middle, Ring (open)
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )

    def get_command(self, t):
        """Calculates and returns the joint command for a given time t."""
        current_command = np.zeros(16)
        for i, finger_name in enumerate(["THUMB", "INDEX", "MIDDLE", "RING"]):
            phase = self.phase_offsets[finger_name]
            oscillation = np.sin(self.frequency * t + phase)

            # Apply the same oscillation to all 4 joints of the finger
            start_idx, end_idx = 4 * i, 4 * i + 4
            centers = self.center_positions[start_idx:end_idx]
            amplitudes = self.amplitudes[start_idx:end_idx]
            current_command[start_idx:end_idx] = centers + amplitudes * oscillation

        return current_command

    def get_velocity_command(self, t):
        """Calculates and returns the joint velocity command for a given time t."""
        current_velocity = np.zeros(16)

        # Calculate joint velocities for each finger
        # Velocity is the time derivative of position: d/dt(sin(wt+p)) = w*cos(wt+p)
        for i, finger_name in enumerate(["THUMB", "INDEX", "MIDDLE", "RING"]):
            phase = self.phase_offsets[finger_name]
            oscillation_derivative = self.frequency * np.cos(self.frequency * t + phase)

            start_idx, end_idx = 4 * i, 4 * i + 4
            amplitudes = self.amplitudes[start_idx:end_idx]
            current_velocity[start_idx:end_idx] = amplitudes * oscillation_derivative

        return current_velocity


def motion_generator_factory(device_name):
    """
    Factory function to create the appropriate motion generator based on device name.
    """
    if device_name == "v4":
        return MotionGeneratorV4()
    elif device_name == "plexus":
        return MotionGeneratorPlexus()
    else:
        raise ValueError(
            f"Unknown device name: {device_name}. Supported devices are 'v4' and 'plexus'."
        )


class AllegroHandMotionController(Node):
    """
    Controls the Allegro Hand to perform a continuous wave motion.

    This class initializes a ROS 2 node, activates the target controller,
    and then periodically publishes joint commands to create the wave effect.
    """

    def __init__(self):
        super().__init__("finger_wave_node")
        self.cmd_publisher_ = None
        self.vel_publisher_ = None

        # Declare and get the controller name parameter. This allows the node
        # to be flexible and work with different controller configurations,
        # e.g., for the right or left hand.
        self.declare_parameter("controller_name", "allegro_hand_position_controller")
        self.declare_parameter("device_name", "v4")

        self.device_name = (
            self.get_parameter("device_name").get_parameter_value().string_value
        )
        self.controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        pos_topic_name = f"/{self.controller_name}/commands"
        vel_topic_name = f"/{self.controller_name}/command_velocities"

        # Activate the target controller before starting any motion.
        # If activation fails, the node will log an error and not proceed.
        if not activate_controller(self, self.controller_name):
            self.get_logger().error(
                f"Failed to activate controller '{self.controller_name}'. "
                "Node will not perform any motion."
            )
            return

        # Create the publisher only after the controller is successfully activated.
        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, pos_topic_name, 10)
        self.vel_publisher_ = self.create_publisher(
            Float64MultiArray, vel_topic_name, 10
        )

        # Parameters for smooth motion
        self.update_rate = 50  # Hz, a higher rate ensures smoother motion.
        self.timer_period = 1.0 / self.update_rate

        self.timer = self.create_timer(self.timer_period, self.motion_callback)
        self.get_logger().info(
            f"Allegro Hand Finger Wave Node Started (Update rate: {self.update_rate}Hz)"
        )
        self.get_logger().info(f"Publishing position commands to: {pos_topic_name}")
        self.get_logger().info(f"Publishing velocity commands to: {vel_topic_name}")

        # Instantiate the motion generator using the factory
        try:
            self.motion_generator = motion_generator_factory(self.device_name)
            self.get_logger().info(
                f"Using motion generator for device: '{self.device_name}'"
            )
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self.t = 0.0  # Time variable for the continuous sine wave.

    def motion_callback(self):
        """
        Called periodically by the timer to calculate and publish new joint commands.
        """
        pos_msg = Float64MultiArray()
        vel_msg = Float64MultiArray()

        # Increment the time variable to advance the sine wave.
        self.t += self.timer_period

        # Delegate position command generation to the motion generator instance
        current_command = self.motion_generator.get_command(self.t)
        pos_msg.data = current_command.tolist()
        self.cmd_publisher_.publish(pos_msg)

        # Delegate velocity command generation to the motion generator instance
        current_velocity = self.motion_generator.get_velocity_command(self.t)
        vel_msg.data = current_velocity.tolist()
        self.vel_publisher_.publish(vel_msg)


def main(args=None):
    """The main entry point for the script."""
    rclpy.init(args=args)
    motion_controller = AllegroHandMotionController()
    try:
        # Spin the node to process callbacks, services, etc.
        rclpy.spin(motion_controller)
    except KeyboardInterrupt:
        # The user pressed Ctrl-C.
        pass
    finally:
        # This block executes on shutdown (including Ctrl-C).
        # Return the hand to a safe, neutral position.
        # These checks are important in case the node failed to initialize publishers.
        if motion_controller.cmd_publisher_:
            pos_msg = Float64MultiArray()
            pos_msg.data = motion_controller.motion_generator.base_position.tolist()
            motion_controller.cmd_publisher_.publish(pos_msg)
            motion_controller.get_logger().info("Returning to base position.")

        if motion_controller.vel_publisher_:
            vel_msg = Float64MultiArray()
            # Publish zero velocity to stop the hand.
            vel_msg.data = [0.0] * 16
            motion_controller.vel_publisher_.publish(vel_msg)

        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
