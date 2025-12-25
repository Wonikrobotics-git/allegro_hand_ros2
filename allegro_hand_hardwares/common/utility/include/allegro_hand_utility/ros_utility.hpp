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
 * @file ros_utility.hpp
 * @brief Provides ROS 2 related utility functions for Allegro Hand packages.
 * @author ('c')void
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace allegro_hand_utility {

/**
 * @brief Dumps the contents of a hardware_interface::HardwareInfo struct to a formatted string.
 *
 * This function is useful for debugging, as it provides a human-readable representation
 * of the information loaded from the robot's URDF file by ros2_control.
 * @param sysinfo The HardwareInfo struct to be dumped.
 * @return A std::string containing the formatted information.
 */
std::string dump_hardware_info(const hardware_interface::HardwareInfo& sysinfo);

/*
 * @class HardwareInfoParser
 * @brief Parses and validates hardware information from the ros2_control HardwareInfo structure.
 *
 * This class extracts necessary parameters from the URDF file, such as the CAN interface
 * name, joint names, and PD gains. It also creates a mapping between the device's
 * internal joint order and the order defined in the URDF.
 */
class HardwareInfoParser {
public:
  struct Parameters {
    /// @brief Table of P gains for each joint, in SDK order.
    std::vector<double> default_p_gain_table;
    /// @brief Table of D gains for each joint, in SDK order.
    std::vector<double> default_d_gain_table;
    /// @brief Table of torque limits for each joint, in SDK order.
    std::vector<double> default_torque_limit_table;
    /// @brief Table of initial positions for each joint, in SDK order.
    std::vector<double> default_initial_positions_table;
    /// @brief Table of minimum position limits for each joint, in SDK order.
    std::vector<double> default_joint_position_limit_min_table;
    /// @brief Table of maximum position limits for each joint, in SDK order.
    std::vector<double> default_joint_position_limit_max_table;

    std::vector<std::string> sdk_ordered_joint_base_names;
  };

private:
  /// @brief Name of the CAN interface (e.g., "can0").
  std::string io_interface_descriptor_;
  /// @brief Hand Id. [1~7]
  uint8_t hand_id_;
  /// @brief Device Name
  std::string device_name_;
  /// @brief Optional prefix for joint names defined in the URDF.
  std::string device_prefix_;
  /// @brief Ordered list of base joint names as expected by the SDK.
  /// These are the names without any prefix.
  std::vector<std::string> sdk_ordered_joint_base_names_;
  /// @brief Ordered list of full joint names (with prefix) as expected by the SDK.
  std::vector<std::string> sdk_ordered_joint_names_;

  /// @brief Table of P gains for each joint, in SDK order.
  std::vector<double> p_gain_table_;
  /// @brief Table of D gains for each joint, in SDK order.
  std::vector<double> d_gain_table_;
  /// @brief Table of torque limits for each joint, in SDK order.
  std::vector<double> torque_limit_table_;
  /// @brief Table of initial positions for each joint, in SDK order.
  std::vector<double> initial_positions_table_;
  /// @brief Table of minimum position limits for each joint, in SDK order.
  std::vector<double> joint_position_limit_min_table_;
  /// @brief Table of maximum position limits for each joint, in SDK order.
  std::vector<double> joint_position_limit_max_table_;

  void _load_initial_pd_gains(const hardware_interface::HardwareInfo& sysinfo);
  void _load_initial_positions(const hardware_interface::HardwareInfo& sysinfo);
  void _load_joint_position_limits(const hardware_interface::HardwareInfo& sysinfo);
  void _load_torque_limits(const hardware_interface::HardwareInfo& sysinfo);
  void _dump_sysinfo(const hardware_interface::HardwareInfo& sysinfo);

public:
  explicit HardwareInfoParser(const Parameters& parameters, const hardware_interface::HardwareInfo& sysinfo);
  virtual ~HardwareInfoParser() = default;

  /** @brief Gets the I/O interface descriptor (e.g., "can:can0"). */
  inline const std::string& io_interface_descriptor() const { return io_interface_descriptor_; }

  /** @brief Gets the hand ID. */
  inline const uint8_t& hand_id() const { return hand_id_; }

  /** @brief Gets the node name. */
  inline std::string dev_node_name() const {
    std::string node_name = device_name_;
    if (!this->device_name_prefix().empty()) {
      node_name = this->device_name_prefix() + node_name;
    }
    return node_name;
  }

  /** @brief Gets the joint name prefix. */
  inline std::string device_name_prefix() const { return device_prefix_; }

  /** @brief Gets the base joint names in the order expected by the SDK. */
  inline const std::vector<std::string>& sdk_ordered_joint_base_names() const { return sdk_ordered_joint_base_names_; }

  /**
   * @brief Gets the finger name corresponding to a given joint name.
   * @param joint_name The full name of the joint (e.g., "prefix_joint10").
   * @return The name of the finger ("Index", "Middle", "Ring", "Thumb") or "Unknown".
   */
  std::string get_finger_name(std::string joint_name) const;

  /**
   * @brief Gets the full joint names in the order expected by the SDK.
   * This list is generated by prepending the `device_prefix_` to the base names.
   * @return A vector of full joint names.
   */
  inline const std::vector<std::string>& sdk_ordered_joint_names() const { return sdk_ordered_joint_names_; }

  /** @brief Gets the table of P gains in SDK order. */
  inline const std::vector<double>& k_p_table() const { return p_gain_table_; }
  /** @brief Gets the table of D gains in SDK order. */
  inline const std::vector<double>& k_d_table() const { return d_gain_table_; }
  /** @brief Gets the table of torque limits in SDK order. */
  inline const std::vector<double>& torque_limit_table() const { return torque_limit_table_; }
  /** @brief Gets the table of initial positions in SDK order. */
  inline const std::vector<double>& initial_position_table() const { return initial_positions_table_; }
  /** @brief Gets the table of minimum joint position limits in SDK order. */
  inline const std::vector<double>& joint_position_limit_min_table() const { return joint_position_limit_min_table_; }
  /** @brief Gets the table of maximum joint position limits in SDK order. */
  inline const std::vector<double>& joint_position_limit_max_table() const { return joint_position_limit_max_table_; }
};

} // namespace allegro_hand_utility