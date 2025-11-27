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
 * @file plexus_sdk.cpp
 * @brief Implements the SDK for the Allegro Hand Plexus, handling communication and control logic.
 * @author ('c')void
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <set>

#include <boost/algorithm/string/join.hpp>
#include <spdlog/spdlog.h>

#include "plexus_sdk.hpp"

namespace plexus_sdk {

/// @brief The name for the torque_limit parameter group, used for ROS 2 parameter handling.
const char* HandPlexus::TORQUE_LIMIT_GROUP = "torque_limit";

// clang-format off
/// @brief For debugging: Maps CAN message IDs to human-readable strings.
static const std::map<int, std::string> MESSAGE_STR_MAP{
    {ID_CMD_SYSTEM_ON_OFF, "ID_CMD_SYSTEM_ON_OFF"},
    {ID_CMD_SET_TORQUE, "ID_CMD_SET_TORQUE"},
    {ID_CMD_SET_TORQUE_1, "ID_CMD_SET_TORQUE_1"},
    {ID_CMD_SET_TORQUE_2, "ID_CMD_SET_TORQUE_2"},
    {ID_CMD_SET_TORQUE_3, "ID_CMD_SET_TORQUE_3"},
    {ID_CMD_SET_TORQUE_4, "ID_CMD_SET_TORQUE_4"},
    {ID_CMD_SET_POSE, "ID_CMD_SET_POSE"},
    {ID_CMD_SET_POSE_1, "ID_CMD_SET_POSE_1"},
    {ID_CMD_SET_POSE_2, "ID_CMD_SET_POSE_2"},
    {ID_CMD_SET_POSE_3, "ID_CMD_SET_POSE_3"},
    {ID_CMD_SET_POSE_4, "ID_CMD_SET_POSE_4"},
    {ID_CMD_SET_PERIOD, "ID_CMD_SET_PERIOD"},
    {ID_CMD_CONFIG, "ID_CMD_CONFIG"},

    {ID_RTR_HAND_INFO, "ID_RTR_HAND_INFO"},
    {ID_RTR_SERIAL, "ID_RTR_SERIAL"},
    {ID_RTR_FINGER_POSE_1, "ID_RTR_FINGER_POSE_1"},
    {ID_RTR_FINGER_POSE_2, "ID_RTR_FINGER_POSE_2"},
    {ID_RTR_FINGER_POSE_3, "ID_RTR_FINGER_POSE_3"},
    {ID_RTR_FINGER_POSE_4, "ID_RTR_FINGER_POSE_4"},
    {ID_RTR_FINGER_TORQUE_1, "ID_RTR_FINGER_TORQUE_1"},
    {ID_RTR_FINGER_TORQUE_2, "ID_RTR_FINGER_TORQUE_2"},
    {ID_RTR_FINGER_TORQUE_3, "ID_RTR_FINGER_TORQUE_3"},
    {ID_RTR_FINGER_TORQUE_4, "ID_RTR_FINGER_TORQUE_4"},
    {ID_RTR_IMU_DATA, "ID_RTR_IMU_DATA"},
    {ID_RTR_TEMPERATURE_1, "ID_RTR_TEMPERATURE_1"},
    {ID_RTR_TEMPERATURE_2, "ID_RTR_TEMPERATURE_2"},
    {ID_RTR_TEMPERATURE_3, "ID_RTR_TEMPERATURE_3"},
    {ID_RTR_TEMPERATURE_4, "ID_RTR_TEMPERATURE_4"},
};

/// @brief A set of message IDs that are sent as Remote Transmission Requests (RTR).
static const std::set<int> RTR_MESSAGE_IDS{
    ID_RTR_HAND_INFO,     
    ID_RTR_SERIAL,          
    ID_RTR_FINGER_POSE_1,   
    ID_RTR_FINGER_POSE_2,   
    ID_RTR_FINGER_POSE_3,
    ID_RTR_FINGER_POSE_4, 
    ID_RTR_FINGER_TORQUE_1, 
    ID_RTR_FINGER_TORQUE_2, 
    ID_RTR_FINGER_TORQUE_3, 
    ID_RTR_FINGER_TORQUE_4,
    ID_RTR_IMU_DATA,      
    ID_RTR_TEMPERATURE_1,   
    ID_RTR_TEMPERATURE_2,   
    ID_RTR_TEMPERATURE_3,   
    ID_RTR_TEMPERATURE_4, 
};
// clang-format on

/*
 * HandPlexus Implementation
 */

/**
 * @brief Constructs the HandPlexus object, initializing the SDK.
 *
 * This constructor is the main entry point for the SDK. It performs the following steps:
 * 1. Sets the maximum torque limit for each joint from the `Options` struct.
 * 2. Calls the private `_initialize()` method to establish communication with the physical hand.
 *
 * @param options The configuration options, including CAN interface descriptor, hand ID, and gain tables.
 * @throws std::runtime_error if `_initialize()` fails (e.g., due to a CAN communication error).
 */
HandPlexus::HandPlexus(const Options& options) : options_(options) {
  request_count_ = 0;
  rtr_message_counter_.clear();

  for (int finger_idx = 0; finger_idx < FINGER_MAX; finger_idx++) {
    for (int joint_idx = 0; joint_idx < JONIT_PER_FINGER; joint_idx++) {
      // Create the PD controller for each joint using the provided gains.
      // const auto& kp = options_.k_p_table[finger_idx * JONIT_PER_FINGER + joint_idx];
      // const auto& kd = options_.k_d_table[finger_idx * JONIT_PER_FINGER + joint_idx];
      // command_.joints[finger_idx][joint_idx].position_controller = std::make_unique<PositionController>(kp, kd);
      command_.joints[finger_idx][joint_idx].torque_max = options_.torque_limit_table[finger_idx * JONIT_PER_FINGER + joint_idx];
    }
  }

  if (!_initialize()) {
    throw std::runtime_error("Failed to initialize Allegro HandPlexus");
  }
}

/**
 * @brief Destroys the HandPlexus object, ensuring a safe shutdown.
 *
 * This destructor ensures a graceful shutdown by sending a "SYSTEM_OFF" command
 * to the hand, which turns off the servo power. This is crucial to prevent unexpected hand movements.
 */
HandPlexus::~HandPlexus() { _set_system_on_off(false); }

/**
 * @brief Converts the current state of the hand to a formatted string.
 *
 * This method provides a human-readable representation of the hand's current state,
 * including IMU data, and for each joint: position, velocity, torque, and temperature.
 * It is primarily used for logging and debugging.
 * @return A string representing the current joint positions, velocities, torques, and temperatures.
 */
std::string HandPlexus::State::to_string() const {
  std::string s;

  s = fmt::format("IMU: rpy=[{:04X}, {:04X}, {:04X}]\n", imu.rpy[0], imu.rpy[1], imu.rpy[2]);

  for (int i = 0; i < FINGER_MAX; ++i) {
    s += fmt::format("Finger {}:\n", i);
    s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Position(d):", joints[i][0].position * 180.0 / M_PI,
                     joints[i][1].position * 180.0 / M_PI, joints[i][2].position * 180.0 / M_PI, joints[i][3].position * 180.0 / M_PI);
    s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Velocity(d/s):", joints[i][0].velocity * 180.0 / M_PI,
                     joints[i][1].velocity * 180.0 / M_PI, joints[i][2].velocity * 180.0 / M_PI, joints[i][3].velocity * 180.0 / M_PI);
    s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Torque:", joints[i][0].torque, joints[i][1].torque,
                     joints[i][2].torque, joints[i][3].torque);
    s += fmt::format("  {:<15} [{:>10.1f}, {:>10.1f}, {:>10.1f}, {:>10.1f}]\n", "Temperature:", joints[i][0].temperature,
                     joints[i][1].temperature, joints[i][2].temperature, joints[i][3].temperature);
  }
  return s;
}

/**
 * @brief Converts the current command for the hand to a formatted string.
 *
 * This method provides a human-readable representation of the hand's current command values,
 * including desired position and torque. It is primarily used for logging and debugging.
 * @return A string representing the current command values.
 */
std::string HandPlexus::Command::to_string() const {
  std::string s;
  s += "Hand Command:\n";

  std::vector<int> finger_indices = {0, 1, 2, 3};

  for (const auto& i : finger_indices) {
    s += fmt::format("Finger {}:\n", i);
    s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Position(d):", joints[i][0].position * 180.0 / M_PI,
                     joints[i][1].position * 180.0 / M_PI, joints[i][2].position * 180.0 / M_PI, joints[i][3].position * 180.0 / M_PI);
    s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Torque:", joints[i][0].torque, joints[i][1].torque,
                     joints[i][2].torque, joints[i][3].torque);
    s += fmt::format("  {:<15} [{:>10d}, {:>10d}, {:>10d}, {:>10d}]\n",
                     "Torque(mA):", static_cast<int>(joints[i][0].torque / TORQUE_CONSTANT),
                     static_cast<int>(joints[i][1].torque / TORQUE_CONSTANT), static_cast<int>(joints[i][2].torque / TORQUE_CONSTANT),
                     static_cast<int>(joints[i][3].torque / TORQUE_CONSTANT));
    // s += fmt::format("  {:<15} [{:>10.4f}, {:>10.4f}, {:>10.4f}, {:>10.4f}]\n", "Torque Max:",
    //                  joints[i][0].torque_max, joints[i][1].torque_max,
    //                  joints[i][2].torque_max, joints[i][3].torque_max);
  }
  return s;
}

/**
 * @brief Parses an incoming CAN message and dispatches it to the appropriate handler.
 *
 * This function is the central callback for received CAN frames. It identifies the
 * message type by its ID and calls the corresponding `_rcv_*` private method to process
 * the data. It operates under a spinlock to ensure thread-safe access to the shared `state_` member.
 * @param message_id The CAN message identifier.
 * @param data A pointer to the raw data payload of the CAN frame.
 * @param data_size The size of the data payload.
 */
void HandPlexus::_parse_message(uint32_t message_id, uint8_t* data, int data_size) {
  (void)data_size;
  std::lock_guard<SpinLock> lock(lock_);

  rtr_message_counter_[message_id]++;

  switch (message_id) {
  case ID_RTR_HAND_INFO:
    _rcv_info(data);
    break;
  case ID_RTR_SERIAL:
    _rcv_serial(data);
    break;
  case ID_RTR_IMU_DATA:
    _rcv_imu(data);
    break;
  case ID_RTR_FINGER_POSE_1:
  case ID_RTR_FINGER_POSE_2:
  case ID_RTR_FINGER_POSE_3:
  case ID_RTR_FINGER_POSE_4:
    _rcv_position(message_id - ID_RTR_FINGER_POSE, data);
    break;
  case ID_RTR_FINGER_VELOCITY_1:
  case ID_RTR_FINGER_VELOCITY_2:
  case ID_RTR_FINGER_VELOCITY_3:
  case ID_RTR_FINGER_VELOCITY_4:
    _rcv_velocity(message_id - ID_RTR_FINGER_VELOCITY, data);
    break;
  case ID_RTR_FINGER_TORQUE_1:
  case ID_RTR_FINGER_TORQUE_2:
  case ID_RTR_FINGER_TORQUE_3:
  case ID_RTR_FINGER_TORQUE_4:
    _rcv_torque(message_id - ID_RTR_FINGER_TORQUE, data);
    break;
  case ID_RTR_TEMPERATURE_1:
  case ID_RTR_TEMPERATURE_2:
  case ID_RTR_TEMPERATURE_3:
  case ID_RTR_TEMPERATURE_4:
    _rcv_temperature(message_id - ID_RTR_TEMPERATURE, data);
    break;
  default:
    SPDLOG_WARN("Unknown message ID: 0x{:x}", message_id);
    break;
  };
}

/**
 * @brief Initializes the communication interface and the hand itself.
 *
 * Creates the `CommIo` object for CAN communication, sets up the necessary callbacks
 * for encoding CAN IDs and decoding received frames. It then sends initial commands
 * to power on the hand, request its information, set the operation mode, and configure data sampling periods.
 * @return True if initialization is successful, false otherwise.
 */
bool HandPlexus::_initialize() {

  // Create CAN Interface
  auto decode_io_frame_can = [this](const uint32_t& header, uint8_t* data, int data_size) {
    _parse_message(((can_id_t*)&header)->message_id, data, data_size);
  };

  auto encode_io_frame_header_can = [this](const uint32_t& message_id) {
    can_id_t id_struct = {};
    id_struct.message_id = message_id;
    id_struct.hand_id = options_.hand_id;
    id_struct.rtr = (RTR_MESSAGE_IDS.count(message_id) > 0);
    uint32_t can_id = 0;
    memcpy(&can_id, &id_struct, sizeof(can_id));
    return can_id;
  };

  comm_io_ = allegro_hand_io::CommIo::Create(options_.io_interface_descriptor, decode_io_frame_can, encode_io_frame_header_can);

  if (comm_io_) {
    _req_hand_info();
    _set_sampling_period();
    set_control_mode(true);
  }
  return true;
}

/**
 * @brief Sends a command to turn the hand's servo motors on or off.
 * A short delay is added after sending the command to allow the hand to process it.
 * This is a critical function for starting and stopping the hand safely.
 * @param on_off True to turn power on, false to turn off.
 * @return True on success, false on failure.
 */
bool HandPlexus::_set_system_on_off(bool on_off) {
  uint8_t data = on_off;
  signal_rate_monitor_.tick("system_on_off");
  auto res = comm_io_->send_message(ID_CMD_SYSTEM_ON_OFF, &data, sizeof(data));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return res > 0;
}

/**
 * @brief Sends a command to set the hand's operation mode.
 * @param mode The desired operation mode (e.g., POSITION_CONTROL, TORQUE_CONTROL).
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandPlexus::_set_operation_mode(OperationMode mode) {
  data_config_t data;
  data.encode(mode);
  signal_rate_monitor_.tick("set_operation_mode");
  return comm_io_->send_message(ID_CMD_CONFIG, (uint8_t*)&data, sizeof(data)) > 0;
}

/**
 * @brief Requests general information from the hand.
 * This includes hardware/firmware versions, hand type, and status flags.
 * This is sent as an RTR (Remote Transmission Request) frame.
 * @return True on success, false on failure.
 */
bool HandPlexus::_req_hand_info() {
  signal_rate_monitor_.tick("req_hand_info");
  return comm_io_->send_message(ID_RTR_HAND_INFO) > 0;
}

/**
 * @brief Configures the data sampling periods on the hand.
 *
 * This function sends a command to the hand to set the update rate (in milliseconds)
 * for position, torque, IMU, and temperature data streams. The periods are defined by constants
 * in `v4_spec.hpp`.
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandPlexus::_set_sampling_period() {
  data_period_t data;
  data.position = POSITION_SAMPLING_PERIOD * 1000;
  data.velocity = VELOCITY_SAMPLING_PERIOD * 1000;
  data.torque = TORQUE_SAMPLING_PERIOD * 1000;
  data.imu = IMU_SAMPLING_PERIOD * 1000;
  data.temperature = 255;
  signal_rate_monitor_.tick("set_sampling_period");
  return comm_io_->send_message(ID_CMD_SET_PERIOD, (uint8_t*)&data, sizeof(data)) > 0;
}

/**
 * @brief Sends a position command to a specific finger (used in POSITION_CONTROL mode).
 *
 * This function packs the four joint position commands for a given finger into a
 * single CAN message and sends it. The message ID is determined by adding the finger index
 * to the base ID `ID_CMD_SET_POSE`.
 * @param finger_idx The index of the finger to command.
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandPlexus::_set_position(unsigned int finger_idx) {
  assert(finger_idx < FINGER_MAX);
  data_position_t data;
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    data.encode(j_idx, command_.joints[finger_idx][j_idx].position);
  }
  signal_rate_monitor_.tick(fmt::format("snd_position{}", finger_idx));
  return comm_io_->send_message(uint32_t(ID_CMD_SET_POSE + finger_idx), (uint8_t*)&data, sizeof(data)) > 0;
}

/**
 * @brief Sends a torque command to a specific finger.
 *
 * This function packs the four joint torque commands for a given finger into a
 * single CAN message and sends it.
 *
 * @param finger_idx The index of the finger (0-3) to command.
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandPlexus::_set_torque(unsigned int finger_idx) {
  assert(finger_idx < FINGER_MAX);
  data_torque_t data;
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    data.encode(j_idx, command_.joints[finger_idx][j_idx].torque);
  }
  signal_rate_monitor_.tick(fmt::format("snd_torque{}", finger_idx));
  return comm_io_->send_message(uint32_t(ID_CMD_SET_TORQUE + finger_idx), (uint8_t*)&data, sizeof(data)) > 0;
}

/**
 * @brief Handles a received position data frame.
 *
 * This private callback is triggered when a CAN message containing joint positions
 * for a specific finger is received. It decodes the raw data for each of the four
 * joints and updates the corresponding `JointState` objects.
 * @param finger_idx The index of the finger providing the data.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_position(unsigned int finger_idx, const uint8_t* data) {
  assert(finger_idx < FINGER_MAX);
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    auto position = reinterpret_cast<const data_position_t*>(data)->decode(j_idx);
    state_.joints[finger_idx][j_idx].set_position(position);
  }
  signal_rate_monitor_.tick(fmt::format("rcv_position{}", finger_idx));
}

/**
 * @brief Handles a received velocity data frame.
 *
 * This private callback decodes raw velocity data for a finger and updates the `JointState`.
 * @param finger_idx The index of the finger providing the data.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_velocity(unsigned int finger_idx, const uint8_t* data) {
  assert(finger_idx < FINGER_MAX);
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    auto velocity = reinterpret_cast<const data_velocity_t*>(data)->decode(j_idx);
    state_.joints[finger_idx][j_idx].set_velocity(velocity);
  }
  signal_rate_monitor_.tick(fmt::format("rcv_velocity{}", finger_idx));
}

/**
 * @brief Handles a received torque data frame.
 *
 * This private callback decodes raw torque data for a finger and updates the `JointState`.
 * @param finger_idx The index of the finger providing the data.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_torque(unsigned int finger_idx, const uint8_t* data) {
  assert(finger_idx < FINGER_MAX);
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    state_.joints[finger_idx][j_idx].set_torque(reinterpret_cast<const data_torque_t*>(data)->decode(j_idx));
  }
  signal_rate_monitor_.tick(fmt::format("rcv_torque{}", finger_idx));
}

/**
 * @brief Handles a received temperature data frame.
 *
 * This private callback is triggered when a CAN message containing motor temperatures
 * for a specific finger is received. It decodes the raw data for each of the four
 * motors and updates the corresponding `JointState` objects.
 * @param finger_idx The index of the finger providing the data.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_temperature(unsigned int finger_idx, const uint8_t* data) {
  assert(finger_idx < FINGER_MAX);
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    auto temperature = reinterpret_cast<const data_temperature_t*>(data)->decode(j_idx);
    state_.joints[finger_idx][j_idx].set_temperature(temperature);
  }
  signal_rate_monitor_.tick("rcv_temperature");
}

/**
 * @brief Handles a received IMU data frame.
 *
 * This private callback is triggered when a CAN message containing IMU data
 * is received. It decodes the raw roll, pitch, and yaw data and updates the
 * `ImuState` object.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_imu(const uint8_t* data) {
  data_imu_rpy_t* imu_data = (data_imu_rpy_t*)data;
  state_.imu.rpy[0] = imu_data->decode(0);
  state_.imu.rpy[1] = imu_data->decode(1);
  state_.imu.rpy[2] = imu_data->decode(2);
  signal_rate_monitor_.tick("rcv_imu");
}

/**
 * @brief Handles a received hand information frame.
 *
 * This private callback is triggered when a CAN message containing general
 * hand information is received. It decodes the data and logs it for debugging
 * and informational purposes, storing it in `state_.main_board_info`.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_info(const uint8_t* data) {
  state_.main_board_info = *((data_info_t*)data);
  // SPDLOG_TRACE("main_board_info\n{}", state_.main_board_info.to_string());
  signal_rate_monitor_.tick("rcv_info");
}

/**
 * @brief Handles a received serial number frame.
 *
 * This private callback is triggered when a CAN message containing the hand's
 * serial number is received. It decodes the data and logs it.
 * @param data A pointer to the raw CAN data payload.
 */
void HandPlexus::_rcv_serial(const uint8_t* data) {
  data_serial_t* serial_data = (data_serial_t*)data;
  state_.serial_number = serial_data->to_string();
  signal_rate_monitor_.tick("rcv_serial");
}

/*
 * Public API
 */

/**
 * @brief Activates the control loop and sets the control mode on the hardware.
 *
 * This function sets the internal control mode and sends the corresponding command
 * to the physical hand to switch between position and torque control.
 * @param position_control_mode True to activate position control, false for torque control.
 */
void HandPlexus::set_control_mode(bool position_control_mode) {
  operation_mode_ = position_control_mode ? OperationMode::CURRENT_BASED_POSITION_CONTROL : OperationMode::TORQUE_CONTROL;
  _set_system_on_off(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  _set_operation_mode(operation_mode_);

  if (operation_mode_ == OperationMode::CURRENT_BASED_POSITION_CONTROL) {
    // set maximum torque
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      command_.joints[finger_idx][joint_idx].set_torque(options_.torque_limit_table.at(io_idx));
    }

    for (int finger_idx = 0; finger_idx < FINGER_MAX; finger_idx++) {
      _set_torque(finger_idx);
    }

    // clear all
    for (int f_idx = 0; f_idx < FINGER_MAX; f_idx++) {
      for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
        command_.joints[f_idx][j_idx].set_torque(0.0);
      }
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  _set_system_on_off(true);
}

/**
 * @brief A periodic function called by an external timer.
 *
 * This function is responsible for tasks that need to be executed periodically,
 * such as requesting non-critical data from the hand. It sends RTR messages to
 * request hand info and serial number, which are not part of the high-frequency data stream.
 */
void HandPlexus::periodic_request() {
  // clang-format off
  std::vector<MessageId> rtr_msg_ids = {
      ID_RTR_HAND_INFO,       
      ID_RTR_SERIAL,         
  };
  // clang-format on 

  size_t rtr_send_count = 0;
  for (const auto& msg_id : rtr_msg_ids) {
    if (comm_io_->send_message(msg_id) > 0) {
      rtr_send_count++;
    }
  }

#if 0 /// DEBUG
  SPDLOG_TRACE("periodic_request----");
  {
    std::vector<std::string> lines;
    for (const auto& [key, num] : rtr_message_counter_) {
      if (MESSAGE_STR_MAP.count(key)) {
        lines.push_back(fmt::format(" - {:<25}: {}", MESSAGE_STR_MAP.at(key), num));
      }
    }
    std::sort(lines.begin(), lines.end());
    SPDLOG_TRACE("rtr_message_counter_[{}]\n{}", request_count_, boost::algorithm::join(lines, "\n"));
  }

  SPDLOG_TRACE("State\n{}", state_.to_string());

  if (rtr_msg_ids.size() == rtr_send_count) {
    SPDLOG_TRACE("ALL RTR MESSAGES SEND OK");
  } else {
    SPDLOG_ERROR("NOT ALL RTR MESSAGES SEND OK");
  }
#endif /// DEBUG 

  request_count_++; 
}

/**
 * @brief Gets the serial number of the hand.
 *
 * This method provides thread-safe access to the serial number string by using a spinlock.
 * The serial number is received from the hand upon request and stored in the `state_` member.
 * @return The serial number as a string.
 */
std::string HandPlexus::get_serial_number() const {
  std::lock_guard<SpinLock> lock(lock_);
  return state_.serial_number;
}

/**
 * @brief Gets the current state of the main board.
 *
 * This method provides thread-safe access to the main board's status information,
 * including hardware/firmware versions, temperature, and various fault flags.
 * The data is received from the hand upon request and stored in the `state_` member.
 * @return A `data_info_t` structure containing the main board state.
 */
data_info_t HandPlexus::get_main_board_state() const {
  std::lock_guard<SpinLock> lock(lock_);
  return state_.main_board_info;
}

/**
 * @brief Reads a specific group of gains from the SDK's internal command structure.
 *
 * This function retrieves the current values for a specified gain group (p_gain, d_gain, or torque_max)
 * for all joints and returns them in a vector, ordered according to the SDK's joint sequence.
 * It provides thread-safe access to the gain values via a spinlock.
 *
 * @param group_name The name of the gain group to read.
 * @return A vector of gain values.
 */
std::vector<double> HandPlexus::read_gains(const char* group_name) {

  auto read_torque_max = [this]{
    std::vector<double> gains;
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      gains.push_back(command_.joints[finger_idx][joint_idx].torque_max);
    }
    return gains;
  };
 
  std::lock_guard<SpinLock> lock(lock_);

  std::vector<double> table;
  if (strcmp(group_name, HandPlexus::TORQUE_LIMIT_GROUP) == 0) {
    table = read_torque_max();
  }

  return table;
}

/**
 * @brief Writes a specific group of gains to the SDK's internal command structure.
 *
 * This function updates the values for a specified gain group (p_gain, d_gain, or torque_max)
 * for all joints using the provided vector of gains. The input vector must be ordered_
 * according to the SDK's internal joint sequence. It provides thread-safe access via a spinlock.
 *
 * @param group_name The name of the gain group to write.
 * @param gains A vector of gain values to set.
 */
void HandPlexus::write_gains(const char* group_name, const std::vector<double>& gains) {
  assert(gains.size() == JONIT_MAX);

  auto write_torque_max = [this](const std::vector<double>& torque_limits) {
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      command_.joints[finger_idx][joint_idx].torque_max = torque_limits[io_idx];
    }
  };

  std::lock_guard<SpinLock> lock(lock_);

  if (strcmp(group_name, HandPlexus::TORQUE_LIMIT_GROUP) == 0) {
    write_torque_max(gains);
  }
}

/**
 * @brief Reads the current state from the SDK and populates the ros2_control state arrays.
 * This function is called by the `hardware_interface` in its `read()` method.
 * It copies the latest joint data from the internal `state_` object to the arrays
 * provided by `ros2_control`, ensuring thread-safe access with a spinlock.
 * @param position_state Pointer to the position state array for ros2_control (16 elements).
 * @param velocity_state Pointer to the velocity state array for ros2_control.
 * @param torque_state Pointer to the torque state array for ros2_control.
 * @param temperature_state Pointer to the temperature state array for ros2_control.
 */
void HandPlexus::read_states(double* position_state, double* velocity_state, double* torque_state, double* temperature_state) {
  std::lock_guard<SpinLock> lock(lock_);
  for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
    int finger_idx = io_idx / JONIT_PER_FINGER;
    int joint_idx = io_idx % JONIT_PER_FINGER;
    position_state[io_idx] = state_.joints[finger_idx][joint_idx].position;
    velocity_state[io_idx] = state_.joints[finger_idx][joint_idx].velocity;
    torque_state[io_idx] = state_.joints[finger_idx][joint_idx].torque;
    temperature_state[io_idx] = state_.joints[finger_idx][joint_idx].temperature;
  }
}

/**
 * @brief Reads the current IMU state from the SDK and populates the ros2_control state arrays.
 *
 * This function is called by the `hardware_interface` in its `read()` method.
 * It copies the latest IMU data from the internal `state_` member to the arrays
 * provided by `ros2_control`.
 * @param acc Pointer to the linear acceleration state array [x, y, z] for ros2_control.
 * @param gyr Pointer to the angular velocity state array [x, y, z] for ros2_control.
 */
void HandPlexus::read_imu_states(double* acc, double* gyr) {
  std::lock_guard<SpinLock> lock(lock_);
  for (int i = 0; i < 3; ++i) {
    acc[i] = state_.imu.acc[i];
    gyr[i] = state_.imu.gyr[i];
  }
}
/**
 * @brief Writes commands from ros2_control to the SDK.
 *
 * This function is called by the `hardware_interface` in its `write()` method.
 * Based on the currently active `operation_mode_`, it takes either the position or
 * effort commands from `ros2_control`.
 * - In POSITION_CONTROL mode, it updates the internal position command and sends it to the hardware.
 * - In TORQUE_CONTROL mode, it updates the internal torque command and sends it to the hardware.
 * The actual sending of the CAN message is handled by `_set_position` or `_set_torque`.
 *
 * @param position_command Pointer to the position command array from ros2_control.
 * @param velocity_command Pointer to the velocity command array from ros2_control (currently unused).
 * @param torque_command Pointer to the effort (torque) command array from ros2_control (used in torque control mode).
 */
void HandPlexus::write_commands(const double* position_command, const double* velocity_command, const double* torque_command) {
  (void)velocity_command;

  for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
    int finger_idx = io_idx / JONIT_PER_FINGER;
    int joint_idx = io_idx % JONIT_PER_FINGER;
    if (operation_mode_ == OperationMode::TORQUE_CONTROL) { 
      command_.joints[finger_idx][joint_idx].set_torque(torque_command[io_idx]); 
    }
    else {
      // Current Based Position Control
      command_.joints[finger_idx][joint_idx].set_position(position_command[io_idx]);
      command_.joints[finger_idx][joint_idx].set_torque(torque_command[io_idx]); 
    }
  }

  // Send Messages
  for (int finger_idx = 0; finger_idx < FINGER_MAX; finger_idx++) {
    if (operation_mode_ == OperationMode::TORQUE_CONTROL) { 
      _set_torque(finger_idx);
    }
    else {
      // Current Based Position Control 
      _set_position(finger_idx);
      _set_torque(finger_idx);
    } 
  } 
}

} // namespace plexus_sdk
