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
 * @file hardware_interface.hpp
 * @brief ros2_control hardware interface(plugin).
 * @author ('c')void
 */

#pragma once

#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <allegro_hand_utility/ros_utility.hpp>
#include <allegro_hand_v4_hardware/visibility_control.h>
using allegro_hand_utility::HardwareInfoParser;

namespace allegro_hand_v4_hardware {

class AllegroHandV4HardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AllegroHandV4HardwareInterface)

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ALLEGRO_HAND_V4_HW_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr int JOINT_NUM = 16;
  std::shared_ptr<HardwareInfoParser> hwinfo_parser_;

  std::shared_ptr<rclcpp::Node> dev_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  double joint_position_command_[JOINT_NUM];
  double joint_velocity_command_[JOINT_NUM];
  double joint_torque_command_[JOINT_NUM];

  double joint_position_state_[JOINT_NUM];
  double joint_velocity_state_[JOINT_NUM];
  double joint_torque_state_[JOINT_NUM];
  double joint_temperature_state_[JOINT_NUM];

  double imu_quat_state_[4];
  double imu_acc_state_[3];
  double imu_gyr_state_[3];
};

} // namespace allegro_hand_v4_hardware
