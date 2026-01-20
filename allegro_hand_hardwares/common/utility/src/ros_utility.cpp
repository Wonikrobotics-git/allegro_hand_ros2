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
/**
 * @file ros_utility.cpp
 * @brief Implements ROS 2 related utility functions.
 * @author ('c')void
 */

#include <allegro_hand_utility/ros_utility.hpp>
#include <sstream>

namespace allegro_hand_utility {

std::string dump_hardware_info(const hardware_interface::HardwareInfo& sysinfo) {
  std::stringstream ss;
  ss << "System Information:\n";
  ss << "  Name: " << sysinfo.name << "\n";
  ss << "  Type: " << sysinfo.type << "\n";

  ss << "  Hardware Parameters:\n";
  for (const auto& param : sysinfo.hardware_parameters) {
    ss << "    - " << param.first << ": " << param.second << "\n";
  }

  ss << "  Joints (" << sysinfo.joints.size() << "):\n";
  for (const auto& joint : sysinfo.joints) {
    ss << "    - Joint: " << joint.name << " (Type: " << joint.type << ")\n";
    ss << "      Command Interfaces:\n";
    for (const auto& iface : joint.command_interfaces) {
      ss << "        - " << iface.name << " (Min: " << iface.min << ", Max: " << iface.max << ")\n";
    }
    ss << "      State Interfaces:\n";
    for (const auto& iface : joint.state_interfaces) {
      ss << "        - " << iface.name << "\n";
    }
    ss << "      Parameters:\n";
    for (const auto& param : joint.parameters) {
      ss << "        - " << param.first << ": " << param.second << "\n";
    }
  }

  ss << "  Sensors (" << sysinfo.sensors.size() << "):\n";
  for (const auto& sensor : sysinfo.sensors) {
    ss << "    - Sensor: " << sensor.name << " (Type: " << sensor.type << ")\n";
    ss << "      State Interfaces:\n";
    for (const auto& iface : sensor.state_interfaces) {
      ss << "        - " << iface.name << "\n";
    }
  }
  return ss.str();
}

/**
 * @brief Constructs the HardwareInfoParser and parses parameters.
 * Initializes default values and then overrides them with parameters loaded from the
 * `HardwareInfo` structure, such as CAN interface name, joint prefix, and PD gains.
 * @param sysinfo The hardware information structure from ros2_control.
 */
HardwareInfoParser::HardwareInfoParser(const Parameters& parameters, const hardware_interface::HardwareInfo& sysinfo) {

  _dump_sysinfo(sysinfo);

  device_name_ = sysinfo.name;
  auto joint_num = parameters.sdk_ordered_joint_base_names.size();

  if (joint_num == 0) {
    throw std::runtime_error("HardwareInfoParser::Parameters requires 'sdk_ordered_joint_base_names' to be non-empty.");
  }

  if (parameters.default_p_gain_table.size() != joint_num) {
    throw std::runtime_error("Size of 'default_p_gain_table' must match the number of joints.");
  }

  if (parameters.default_d_gain_table.size() != joint_num) {
    throw std::runtime_error("Size of 'default_d_gain_table' must match the number of joints.");
  }

  if (parameters.default_torque_limit_table.size() != joint_num) {
    throw std::runtime_error("Size of 'default_torque_limit_table' must match the number of joints.");
  }

  sdk_ordered_joint_base_names_ = parameters.sdk_ordered_joint_base_names;
  p_gain_table_ = parameters.default_p_gain_table;
  d_gain_table_ = parameters.default_d_gain_table;
  torque_limit_table_ = parameters.default_torque_limit_table;

  initial_positions_table_ = parameters.default_initial_positions_table;
  joint_position_limit_min_table_ = parameters.default_joint_position_limit_min_table;
  joint_position_limit_max_table_ = parameters.default_joint_position_limit_max_table;

  if (initial_positions_table_.size() == 0) {
    initial_positions_table_.resize(joint_num);
    std::fill(initial_positions_table_.begin(), initial_positions_table_.end(), 0.0);
  } else {
    if (initial_positions_table_.size() != joint_num) {
      throw std::runtime_error("If provided, size of 'default_initial_positions_table' must match the number of joints.");
    }
  }

  if (joint_position_limit_min_table_.size() == 0) {
    joint_position_limit_min_table_.resize(joint_num);
    std::fill(joint_position_limit_min_table_.begin(), joint_position_limit_min_table_.end(), -M_PI);
  } else {
    if (joint_position_limit_min_table_.size() != joint_num) {
      throw std::runtime_error("If provided, size of 'default_joint_position_limit_min_table' must match the number of joints.");
    }
  }

  if (joint_position_limit_max_table_.size() == 0) {
    joint_position_limit_max_table_.resize(joint_num);
    std::fill(joint_position_limit_max_table_.begin(), joint_position_limit_max_table_.end(), M_PI);
  } else {
    if (joint_position_limit_max_table_.size() != joint_num) {
      throw std::runtime_error("If provided, size of 'default_joint_position_limit_max_table' must match the number of joints.");
    }
  }

  try {
    device_prefix_ = sysinfo.hardware_parameters.at("device_prefix");
  } catch (const std::exception& e) {
    device_prefix_ = "";
  }

  try {
    io_interface_descriptor_ = sysinfo.hardware_parameters.at("io_interface_descriptor");
  } catch (const std::exception& e) {
    throw std::runtime_error("Required hardware parameter 'io_interface_descriptor' not found in URDF.");
  }

  try {
    hand_id_ = static_cast<uint8_t>(std::stoi(sysinfo.hardware_parameters.at("hand_id")));
  } catch (const std::exception& e) {
    throw std::runtime_error("Required hardware parameter 'hand_id' not found or invalid in URDF.");
  }

  sdk_ordered_joint_names_.clear();
  for (const auto& base_name : sdk_ordered_joint_base_names_) {
    sdk_ordered_joint_names_.push_back(device_prefix_ + base_name);
  }

  _load_initial_pd_gains(sysinfo);
  _load_initial_positions(sysinfo);
  _load_joint_position_limits(sysinfo);
  _load_torque_limits(sysinfo);
}

void HardwareInfoParser::_load_initial_pd_gains(const hardware_interface::HardwareInfo& sysinfo) {
  for (size_t i = 0; i < sdk_ordered_joint_base_names_.size(); i++) {
    // Load P gains
    std::string p_gain_key = "p_" + sdk_ordered_joint_base_names_.at(i);
    auto p_it = sysinfo.hardware_parameters.find(p_gain_key);
    if (p_it != sysinfo.hardware_parameters.end()) {
      p_gain_table_[i] = std::atof(p_it->second.c_str());
    }

    // Load D gains
    std::string d_gain_key = "d_" + sdk_ordered_joint_base_names_.at(i);
    auto d_it = sysinfo.hardware_parameters.find(d_gain_key);
    if (d_it != sysinfo.hardware_parameters.end()) {
      d_gain_table_[i] = std::atof(d_it->second.c_str());
    }
  }
}

void HardwareInfoParser::_load_initial_positions(const hardware_interface::HardwareInfo& sysinfo) {
  std::map<std::string, int> joint_inf_map;
  const auto& joint_names = this->sdk_ordered_joint_names();
  for (size_t i = 0; i < joint_names.size(); i++) {
    joint_inf_map[joint_names[i]] = i;
  }

  // Initialize state and command arrays from initial_values in URDF
  for (const auto& joint : sysinfo.joints) {
    // Find the position state interface to get the initial_value safely
    auto pos_state_if_it =
        std::find_if(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                     [](const hardware_interface::InterfaceInfo& if_info) { return if_info.name == hardware_interface::HW_IF_POSITION; });

    if (pos_state_if_it != joint.state_interfaces.end()) {
      double initial_pos = std::stod(pos_state_if_it->initial_value);
      int if_idx = joint_inf_map.at(joint.name);
      initial_positions_table_[if_idx] = initial_pos;
    }
  }
}

/**
 * @brief Loads joint position limits (lower_limit/upper_limit) from the hardware parameters in the URDF.
 *
 * This function populates `joint_position_limit_min_table_` and `joint_position_limit_max_table_`
 * with values from the `<joint><param name="lower_limit">` and `<param name="upper_limit">` tags.
 *
 * e.g.
 *   <joint name="${prefix}joint00">
 *       <command_interface name="position"/>
 *       <command_interface name="effort"/>
 *       <state_interface name="position">
 *         <param name="initial_value">${initial_positions['joint00']}</param>
 *       </state_interface>
 *       ...
 *       <!-- Add the following parameters -->
 *       <param name="lower_limit">${radians(-27.5)}</param>
 *       <param name="upper_limit">${radians(52.5)}</param>
 *   </joint>
 *
 * @param sysinfo The hardware information structure from ros2_control.
 */
void HardwareInfoParser::_load_joint_position_limits(const hardware_interface::HardwareInfo& sysinfo) {
  std::map<std::string, int> joint_inf_map;
  const auto& joint_names = this->sdk_ordered_joint_names();
  for (size_t i = 0; i < joint_names.size(); i++) {
    joint_inf_map[joint_names[i]] = i;
  }

  for (const auto& joint : sysinfo.joints) {
    int if_idx = joint_inf_map.at(joint.name);

    if (joint.parameters.find("lower_limit") != joint.parameters.end()) {
      joint_position_limit_min_table_[if_idx] = std::stod(joint.parameters.at("lower_limit"));
    }

    if (joint.parameters.find("upper_limit") != joint.parameters.end()) {
      joint_position_limit_max_table_[if_idx] = std::stod(joint.parameters.at("upper_limit"));
    }
  }
}

void HardwareInfoParser::_load_torque_limits(const hardware_interface::HardwareInfo& sysinfo) {
  for (size_t i = 0; i < sdk_ordered_joint_base_names_.size(); i++) {
    // Load torque limits
    std::string torque_limit_key = "torque_" + sdk_ordered_joint_base_names_.at(i);
    auto torque_it = sysinfo.hardware_parameters.find(torque_limit_key);
    if (torque_it != sysinfo.hardware_parameters.end()) {
      torque_limit_table_[i] = std::atof(torque_it->second.c_str());
    }
  }
}

void HardwareInfoParser::_dump_sysinfo(const hardware_interface::HardwareInfo& sysinfo) {
  auto msg = allegro_hand_utility::dump_hardware_info(sysinfo);
  SPDLOG_DEBUG(msg);
}

/**
 * @brief Gets the finger name corresponding to a given joint name.
 * @param joint_name The full name of the joint (e.g., "prefix_joint10").
 * @return The name of the finger ("Thumb", "Index", "Middle", "Ring", "Little") or "Unknown".
 */
std::string HardwareInfoParser::get_finger_name(std::string joint_name) const {
  std::string base_joint_name = joint_name;
  if (!device_prefix_.empty() && joint_name.rfind(device_prefix_, 0) == 0) {
    base_joint_name = joint_name.substr(device_prefix_.length());
  }
  if (base_joint_name.rfind("joint0", 0) == 0) {
    return "Thumb";
  }
  if (base_joint_name.rfind("joint1", 0) == 0) {
    return "Index";
  }
  if (base_joint_name.rfind("joint2", 0) == 0) {
    return "Middle";
  }
  if (base_joint_name.rfind("joint3", 0) == 0) {
    return "Ring";
  }
  if (base_joint_name.rfind("joint4", 0) == 0) {
    return "Little";
  }
  return "Unknown";
}

} // namespace allegro_hand_utility