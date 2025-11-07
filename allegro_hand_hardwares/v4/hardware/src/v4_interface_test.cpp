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
 * @file v4_interface_test.cpp
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("AH_V4_TEST");

class HardwareInterfaceTestNode : public rclcpp::Node {
public:
  HardwareInterfaceTestNode()
      : rclcpp::Node("hardware_interface_test_node"), plugin_loader_("hardware_interface", "hardware_interface::SystemInterface") {
    RCLCPP_INFO(LOGGER, "Starting test node.");

    // 1. Load hardware interface plugin
    try {
      hardware_ = std::unique_ptr<hardware_interface::SystemInterface>(
          plugin_loader_.createUnmanagedInstance("allegro_hand_v4_hardware/AllegroHandV4HardwareInterface"));
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(LOGGER, "Failed to load hardware interface plugin: %s", ex.what());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(LOGGER, "Successfully loaded hardware interface plugin.");

    // 2. Create HardwareInfo struct (mocking URDF info)
    // AllegroHandV4HardwareInterface expects 16 joints.
    info_.name = "allegro_hand_v4";
    info_.type = "system";
    // info_.hardware_class_type = "allegro_hand_v4_hardware/AllegroHandV4HardwareInterface";

    info_.hardware_parameters["io_interface_descriptor"] = "can:can0";
    info_.hardware_parameters["device_prefix"] = ""; // Set empty prefix for testing
    RCLCPP_INFO(LOGGER, "Parameter: io_interface_descriptor=%s", info_.hardware_parameters["io_interface_descriptor"].c_str());

    // Define the 16 joint names as expected by the hardware interface
    const std::vector<std::string> joint_names = {
        "joint10", "joint11", "joint12", "joint13", "joint20", "joint21", "joint22", "joint23",
        "joint30", "joint31", "joint32", "joint33", "joint00", "joint01", "joint02", "joint03",
    };

    // Default PD gains from the hardware interface implementation, as pd_gains.yaml has placeholder values.
    const std::vector<double> p_gains = {
        600.0,  600.0,  600.0,  1000.0, // Index
        600.0,  600.0,  600.0,  1000.0, // Middle
        600.0,  600.0,  600.0,  1000.0, // Ring/Pinky
        1000.0, 1000.0, 1000.0, 600.0   // Thumb
    };
    const std::vector<double> d_gains = {
        15.0, 20.0, 15.0, 15.0, // Index
        15.0, 20.0, 15.0, 15.0, // Middle
        15.0, 20.0, 15.0, 15.0, // Ring/Pinky
        30.0, 20.0, 20.0, 15.0  // Thumb
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

      // Provide mock PD gain parameters required by the SDK adaptor
      info_.hardware_parameters["p_" + joint_name] = std::to_string(p_gains[joint_idx]);
      info_.hardware_parameters["d_" + joint_name] = std::to_string(d_gains[joint_idx]);

      info_.joints.push_back(joint);
    }
    RCLCPP_INFO(LOGGER, "Created mock HardwareInfo.");

    // 3. Initialize hardware interface (on_init)
    if (hardware_->on_init(info_) != hardware_interface::CallbackReturn::SUCCESS) {
      // Provide a more descriptive error message
      RCLCPP_FATAL(LOGGER, "on_init() failed. Check if the mock HardwareInfo provides all required joints, interfaces, and parameters.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(LOGGER, "on_init() successful.");

    // 4. Export interfaces
    command_interfaces_ = hardware_->export_command_interfaces();
    state_interfaces_ = hardware_->export_state_interfaces();
    RCLCPP_INFO(LOGGER, "Exported command and state interfaces.");

    // Prepare for command mode switch
    std::vector<std::string> start_interfaces;
    for (const auto& joint_name : joint_names) {
      start_interfaces.push_back(joint_name + "/position");
    }

    if (hardware_->prepare_command_mode_switch(start_interfaces, {}) != hardware_interface::return_type::OK) {
      RCLCPP_FATAL(LOGGER, "prepare_command_mode_switch() failed.");
    }

    RCLCPP_INFO(LOGGER, "prepare_command_mode_switch.");

    // 5. Activate hardware interface (on_activate)
    if (hardware_->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(LOGGER, "on_activate() failed. Check if the robot is connected to the network.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(LOGGER, "on_activate() successful. Now starting read/write loop.");

    // Perform command mode switch after activation
    if (hardware_->perform_command_mode_switch(start_interfaces, {}) != hardware_interface::return_type::OK) {
      RCLCPP_FATAL(LOGGER, "perform_command_mode_switch() failed.");
    }

    SPDLOG_TRACE("START V4 INTERFACE TEST");

    // 6. Create timer for Read/Write test
    timer_ = this->create_wall_timer(100ms, std::bind(&HardwareInterfaceTestNode::rw_loop, this));
  }

  ~HardwareInterfaceTestNode() {
    if (hardware_) {
      RCLCPP_INFO(LOGGER, "Calling on_deactivate().");
      hardware_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

private:
  void rw_loop() {

    /*
     *
     * TEST
     *
     */
    return;
    /*
     *
     * TEST
     *
     */

    auto time = this->now();
    auto period = time - last_time_;
    last_time_ = time;

    // READ
    if (hardware_->read(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(LOGGER, "read() failed");
    }

    // Format and print state values in a readable table
    std::string state_str = "Reading state:\n";
    state_str += fmt::format("       {:>9} {:>9} {:>9} {:>9}\n", "Joint 0", "Joint 1", "Joint 2", "Joint 3");
    state_str += "--------------------------------------------------\n";

    const char* finger_names[] = {"Index ", "Middle", "Ring  ", "Thumb "};
    const char* state_names[] = {"Pos", "Vel", "Eff"};

    // state_interfaces are ordered by [pos, vel, eff] for each joint
    for (size_t finger_idx = 0; finger_idx < 4; ++finger_idx) {
      state_str += fmt::format("{}: \n", finger_names[finger_idx]);
      for (size_t state_idx = 0; state_idx < 3; ++state_idx) {
        state_str += fmt::format("  {}: ", state_names[state_idx]);
        for (size_t joint_idx = 0; joint_idx < 4; ++joint_idx) {
          // Each joint has 3 state interfaces (pos, vel, eff)
          size_t interface_idx = (finger_idx * 4 + joint_idx) * 3 + state_idx;
          if (interface_idx < state_interfaces_.size()) {
            double value = state_interfaces_[interface_idx].get_value();
            if (state_idx < 2) { // Position and Velocity are in rad, convert to deg
              value *= 180.0 / M_PI;
            }
            state_str += fmt::format("{:>9.3f} ", value);
          }
        }
        state_str += "\n";
      }
    }
    RCLCPP_INFO(LOGGER, "\n%s", state_str.c_str());

    // WRITE
    // Send sine wave command to the first joint
    double sine_cmd = 0.2 * sin(time.seconds());
    // The command interfaces are exported as [pos0, eff0, pos1, eff1, ...].
    // So the position command for the first joint is at index 0.
    command_interfaces_[0].set_value(sine_cmd);

    // Keep the rest of the joints at their current position
    // Iterate through joints, not interfaces
    for (size_t i = 1; i < info_.joints.size(); ++i) {
      // State interfaces are [pos0, vel0, eff0, pos1, vel1, eff1, ...]
      // Command interfaces are [pos0, eff0, pos1, eff1, ...]
      if ((i * 3) < state_interfaces_.size()) { // Check bounds for state
        command_interfaces_[i * 2].set_value(state_interfaces_[i * 3].get_value());
      }
    }

    if (hardware_->write(time, period) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(LOGGER, "write() failed");
    }

    RCLCPP_INFO(LOGGER, "Write command (joint1): [%f]", command_interfaces_[0].get_value());
  }

  pluginlib::ClassLoader<hardware_interface::SystemInterface> plugin_loader_;
  std::unique_ptr<hardware_interface::SystemInterface> hardware_;

  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_ = this->now();
};

int main(int argc, char** argv) {

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