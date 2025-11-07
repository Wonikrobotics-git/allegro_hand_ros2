/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file plexus_interface_test.cpp
 * @brief Test node for the ros2_control hardware interface(plugin).
 * @author ('c')void
 */

#include <cmath>
#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("AH_PLEXUS_TEST");

/**
 * @class HardwareInterfaceTestNode
 * @brief A standalone ROS 2 node to test the AllegroHandPlexusHardwareInterface plugin.
 * This node mimics the behavior of the ros2_control controller_manager by loading the
 * hardware interface plugin, configuring it with mock data, and running a read/write loop.
 */
class HardwareInterfaceTestNode : public rclcpp::Node {
  /// @brief Plugin loader for the hardware_interface::SystemInterface.
  pluginlib::ClassLoader<hardware_interface::SystemInterface> plugin_loader_;
  /// @brief Unique pointer to the loaded hardware interface instance.
  std::unique_ptr<hardware_interface::SystemInterface> hardware_;

  /// @brief Mock hardware info, typically parsed from a URDF.
  hardware_interface::HardwareInfo info_;
  /// @brief Vector of command interfaces exported by the hardware.
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  /// @brief Vector of state interfaces exported by the hardware.
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// @brief Maps for quick, name-based access to state interfaces.
  std::map<std::string, hardware_interface::StateInterface*> state_interface_map_;
  /// @brief Maps for quick, name-based access to command interfaces.
  std::map<std::string, hardware_interface::CommandInterface*> command_interface_map_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_ = this->now();

public:
  HardwareInterfaceTestNode()
      : rclcpp::Node("hardware_interface_test_node"), plugin_loader_("hardware_interface", "hardware_interface::SystemInterface") {
    RCLCPP_INFO(LOGGER, "Starting test node.");

    // 1. Load hardware interface plugin
    try {
      hardware_ = std::unique_ptr<hardware_interface::SystemInterface>(
          plugin_loader_.createUnmanagedInstance("allegro_hand_plexus_hardware/AllegroHandPlexusHardwareInterface"));
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(LOGGER, "Failed to load hardware interface plugin: %s", ex.what());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(LOGGER, "Successfully loaded hardware interface plugin.");

    // 2. Create HardwareInfo struct (mocking URDF info)
    // The AllegroHandPlexusHardwareInterface expects 16 joints.
    info_.name = "allegro_hand_plexus";
    info_.type = "system";
    // info_.hardware_class_type = "allegro_hand_plexus_hardware/AllegroHandPlexusHardwareInterface";

    info_.hardware_parameters["io_interface_descriptor"] = "can:can0";
    info_.hardware_parameters["hand_id"] = "1";
    info_.hardware_parameters["device_prefix"] = ""; // Set empty prefix for testing
    RCLCPP_INFO(LOGGER, "Parameter: io_interface_descriptor=%s", info_.hardware_parameters["io_interface_descriptor"].c_str());

    // Define the 16 joint names as expected by the hardware interface
    const std::vector<std::string> joint_names = {
        "joint10", "joint11", "joint12", "joint13", "joint20", "joint21", "joint22", "joint23",
        "joint30", "joint31", "joint32", "joint33", "joint00", "joint01", "joint02", "joint03",
    };

    // Default PD gains. These values are typically loaded from a YAML file in a real setup.
    const std::vector<double> p_gains = {
        1.0, 1.0, 1.0, 1.0, // Index
        1.0, 1.0, 1.0, 1.0, // Middle
        1.0, 1.0, 1.0, 1.0, // Ring/Pinky
        1.0, 1.0, 1.0, 1.0  // Thumb
    };
    const std::vector<double> d_gains = {
        1.0, 1.0, 1.0, 1.0, // Index
        1.0, 1.0, 1.0, 1.0, // Middle
        1.0, 1.0, 1.0, 1.0, // Ring/Pinky
        1.0, 1.0, 1.0, 1.0  // Thumb
    };

    for (const auto& joint_name : joint_names) {
      hardware_interface::ComponentInfo joint;
      joint.name = joint_name;

      hardware_interface::InterfaceInfo if_info;

      // Provide all required command and state interfaces
      if_info.name = "position";
      if_info.initial_value = "0.0";
      joint.command_interfaces.push_back(if_info);

      if_info.name = "effort";
      if_info.initial_value = "0.0";
      joint.command_interfaces.push_back(if_info);

      if_info.name = "position";
      joint.state_interfaces.push_back(if_info);
      if_info.name = "velocity";
      joint.state_interfaces.push_back(if_info);
      if_info.name = "effort";
      joint.state_interfaces.push_back(if_info);
      if_info.name = "temperature";
      joint.state_interfaces.push_back(if_info);

      // Find the index of the current joint to get the correct gain
      auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
      size_t joint_idx = std::distance(joint_names.begin(), it);

      // Provide mock PD gain parameters required by the hardware interface.
      info_.hardware_parameters["p_" + joint_name] = std::to_string(p_gains[joint_idx]);
      info_.hardware_parameters["d_" + joint_name] = std::to_string(d_gains[joint_idx]);

      info_.joints.push_back(joint);
    }
    RCLCPP_INFO(LOGGER, "Created mock HardwareInfo.");

    // 3. Initialize hardware interface (on_init)
    if (hardware_->on_init(info_) != hardware_interface::CallbackReturn::SUCCESS) {
      // This error typically occurs if the mock HardwareInfo is missing required information.
      RCLCPP_FATAL(LOGGER, "on_init() failed. Check if the mock HardwareInfo provides all required joints, interfaces, and parameters.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(LOGGER, "on_init() successful.");

    // 4. Configure hardware interface (on_configure)
    if (hardware_->on_configure(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(LOGGER, "on_configure() failed.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(LOGGER, "on_configure() successful.");

    // 5. Activate hardware interface (on_activate)
    if (hardware_->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(LOGGER, "on_activate() failed. Check if the robot is connected to the network.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(LOGGER, "on_activate() successful. Now starting read/write loop.");

    // 6. Export interfaces
    command_interfaces_ = hardware_->export_command_interfaces();
    state_interfaces_ = hardware_->export_state_interfaces();
    RCLCPP_INFO(LOGGER, "Exported command and state interfaces.");

    // Create maps for quick, name-based interface lookup to avoid index-based errors.
    for (auto& iface : state_interfaces_) {
      state_interface_map_[iface.get_name()] = &iface;
    }
    for (auto& iface : command_interfaces_) {
      command_interface_map_[iface.get_name()] = &iface;
    }

    RCLCPP_INFO(LOGGER, "Created interface maps for quick lookup.");

    // 7. Prepare and perform the command mode switch to 'position' control.
    std::vector<std::string> start_interfaces;
    for (const auto& joint_name : joint_names) {
      start_interfaces.push_back(joint_name + "/position");
    }

    if (hardware_->prepare_command_mode_switch(start_interfaces, {}) != hardware_interface::return_type::OK) {
      RCLCPP_FATAL(LOGGER, "prepare_command_mode_switch() failed.");
    }

    RCLCPP_INFO(LOGGER, "prepare_command_mode_switch.");

    // 8. Perform command mode switch after activation
    if (hardware_->perform_command_mode_switch(start_interfaces, {}) != hardware_interface::return_type::OK) {
      RCLCPP_FATAL(LOGGER, "perform_command_mode_switch() failed.");
    }

    SPDLOG_TRACE("START PLEXUS INTERFACE TEST");

    // 9. Create a timer to run the main read-write loop.
    timer_ = this->create_wall_timer(10ms, std::bind(&HardwareInterfaceTestNode::rw_loop, this));
  }

  ~HardwareInterfaceTestNode() {
    /**
     * @brief Destructor. Calls on_deactivate to ensure a clean shutdown of the hardware.
     */
    if (hardware_) {
      RCLCPP_INFO(LOGGER, "Calling on_deactivate().");
      hardware_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

private:
  /**
   * @brief Generates and sends sine wave position commands to all joints.
   * Each joint oscillates around a center point derived from its URDF limits,
   * creating a continuous, smooth motion for testing.
   */
  void send_sine_command() {
    // Define center and amplitude for each joint's sine wave movement
    // Center values are based on the joint limits from allegro_plexus_right.xacro
    // Amplitude is set to 20 degrees (M_PI / 9.0 radians) for all joints
    // The frequency of the sine wave can be adjusted here.
    constexpr double SINE_FREQUENCY_HZ = 0.5; // Desired frequency in Hz
    const double angular_frequency = 2.0 * M_PI * SINE_FREQUENCY_HZ;

    const double amplitude = M_PI / 9.0;                    // 20 degrees
    const std::map<std::string, double> center_positions = {// Index, Middle, Ring Fingers
                                                            {"joint10", 0.0},
                                                            {"joint11", 1.169 / 2.0},
                                                            {"joint12", 1.169 / 2.0},
                                                            {"joint13", 1.169 / 2.0},
                                                            {"joint20", 0.0},
                                                            {"joint21", 1.169 / 2.0},
                                                            {"joint22", 1.169 / 2.0},
                                                            {"joint23", 1.169 / 2.0},
                                                            {"joint30", 0.0},
                                                            {"joint31", 1.169 / 2.0},
                                                            {"joint32", 1.169 / 2.0},
                                                            {"joint33", 1.169 / 2.0},
                                                            // Thumb
                                                            {"joint00", 1.745 / 2.0},
                                                            {"joint01", (-0.349 + 1.169) / 2.0},
                                                            {"joint02", 1.169 / 2.0},
                                                            {"joint03", 1.570 / 2.0}};

    double time_sec = this->now().seconds();

    // Apply sine wave command to all joints
    for (const auto& joint_info : info_.joints) {
      const auto& joint_name = joint_info.name;
      std::string pos_cmd_if_name = joint_name + "/position";

      if (command_interface_map_.count(pos_cmd_if_name) && center_positions.count(joint_name)) {
        double center = center_positions.at(joint_name);
        double sine_cmd = center + amplitude * sin(angular_frequency * time_sec);

        // Ensure the command is within the joint limits from xacro (optional but good practice)
        // For this test, we assume the center + amplitude will not exceed limits.
        command_interface_map_.at(pos_cmd_if_name)->set_value(sine_cmd);
      }
    }
    RCLCPP_INFO(LOGGER, "Sent sine wave commands to all joints.");
  }

  /**
   * @brief Sends a command to move all joints to the zero position.
   */
  void send_zero_command() {
    RCLCPP_INFO(LOGGER, "Sending zero command to all joints.");
    for (const auto& joint_info : info_.joints) {
      std::string pos_cmd_if_name = joint_info.name + "/position";
      if (command_interface_map_.count(pos_cmd_if_name)) {
        command_interface_map_.at(pos_cmd_if_name)->set_value(0.0);
      }
    }
  }

  /**
   * @brief The main read-write loop, called periodically by the timer.
   * This function mimics the core loop of the ros2_control manager. It reads the
   * latest state from the hardware, prints it, generates new commands, and writes
   * them back to the hardware.
   */
  void rw_loop() {
    auto time = this->now();
    auto period = time - last_time_;
    last_time_ = time;

    // READ
    if (hardware_->read(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(LOGGER, "read() failed");
      return;
    }

    // Format and print state values in a readable table
    std::string state_str = "Reading state:\n";
    state_str += fmt::format("       {:>9} {:>9} {:>9} {:>9}\n", "Joint 0", "Joint 1", "Joint 2", "Joint 3");
    state_str += "--------------------------------------------------\n";

    // Define joint names in the order they are physically arranged for printing
    const std::vector<std::vector<std::string>> finger_joints = {
        {"joint10", "joint11", "joint12", "joint13"}, // Index
        {"joint20", "joint21", "joint22", "joint23"}, // Middle
        {"joint30", "joint31", "joint32", "joint33"}, // Ring
        {"joint00", "joint01", "joint02", "joint03"}  // Thumb
    };
    const std::vector<std::string> finger_names = {"Index ", "Middle", "Ring  ", "Thumb "};
    const std::vector<std::pair<std::string, std::string>> state_if_info = {
        {"Pos", "position"}, {"Vel", "velocity"}, {"Eff", "effort"}, {"Tmp", "temperature"}};

    for (size_t i = 0; i < finger_names.size(); ++i) {
      state_str += fmt::format("{}: \n", finger_names[i]);
      for (const auto& [short_name, if_name] : state_if_info) {
        state_str += fmt::format("  {}: ", short_name);
        for (const auto& joint_name : finger_joints[i]) {
          std::string full_if_name = joint_name + "/" + if_name;
          double value = 0.0;
          if (state_interface_map_.count(full_if_name)) {
            value = state_interface_map_.at(full_if_name)->get_value();
          }
          if (if_name == "position" || if_name == "velocity") {
            value *= 180.0 / M_PI; // Convert rad to deg for display
          }
          state_str += fmt::format("{:>9.3f} ", value);
        }
        state_str += "\n";
      }
    }
    RCLCPP_INFO(LOGGER, "\n%s", state_str.c_str());

    // WRITE: Select which command to send for testing
#if 1
    send_sine_command();
#else
    send_zero_command();
#endif

    if (hardware_->write(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(LOGGER, "write() failed");
    }
  }
};

int main(int argc, char** argv) {
  // Set the logger level to trace for detailed output.
  spdlog::set_level(spdlog::level::trace);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareInterfaceTestNode>();

  // Only run spin when rclcpp::ok() is not false.
  // Because shutdown can be called in the constructor.
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}