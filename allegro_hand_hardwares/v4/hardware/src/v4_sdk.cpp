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
 * @file v4_sdk.cpp
 * @brief Implements the SDK for the Allegro Hand V4, handling communication and control logic.
 * @author ('c')void
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <set>

#include <boost/algorithm/string/join.hpp>
#include <spdlog/spdlog.h>

#include "v4_sdk.hpp"

namespace v4_sdk {

const char* HandV4::P_GAIN_GROUP = "p_gain";
const char* HandV4::D_GAIN_GROUP = "d_gain";
const char* HandV4::TORQUE_LIMIT_GROUP = "torque_limit";

// clang-format off
// For debugging: Maps CAN message IDs to human-readable strings.
static const std::map<int, std::string> MESSAGE_STR_MAP{
    {ID_CMD_SYSTEM_ON, "ID_CMD_SYSTEM_ON"},         
    {ID_CMD_SYSTEM_OFF, "ID_CMD_SYSTEM_OFF"},
    {ID_CMD_SET_TORQUE_1, "ID_CMD_SET_TORQUE_1"},   
    {ID_CMD_SET_TORQUE_2, "ID_CMD_SET_TORQUE_2"},
    {ID_CMD_SET_TORQUE_3, "ID_CMD_SET_TORQUE_3"},  
    {ID_CMD_SET_TORQUE_4, "ID_CMD_SET_TORQUE_4"},
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
    {ID_RTR_TEMPERATURE_1, "ID_RTR_TEMPERATURE_1"},
    {ID_RTR_TEMPERATURE_2, "ID_RTR_TEMPERATURE_2"},
    {ID_RTR_TEMPERATURE_3, "ID_RTR_TEMPERATURE_3"}, 
    {ID_RTR_TEMPERATURE_4, "ID_RTR_TEMPERATURE_4"},
};

// A set of message IDs that are sent as Remote Transmission Requests (RTR).
static const std::set<int> RTR_MESSAGE_IDS{
    ID_RTR_HAND_INFO, 
    ID_RTR_SERIAL,        
    ID_RTR_FINGER_POSE_1, 
    ID_RTR_FINGER_POSE_2, 
    ID_RTR_FINGER_POSE_3, 
    ID_RTR_FINGER_POSE_4,
    ID_RTR_IMU_DATA,  
    ID_RTR_TEMPERATURE_1, 
    ID_RTR_TEMPERATURE_2, 
    ID_RTR_TEMPERATURE_3, 
    ID_RTR_TEMPERATURE_4,
};
// clang-format on

/*
 * HandV4 Implementation
 */

/**
 * @brief Constructs the HandV4 object.
 *
 * This is the main entry point for the SDK. It initializes the hardware info parser,
 * sets up maximum torque limits and PD controllers for each joint, and then calls
 * `_initialize()` to establish communication with the physical hand.
 * @param options The configuration options, including CAN interface name and PD gains.
 * @throws std::runtime_error if the SDK fails to initialize (e.g., CAN communication error).
 */
HandV4::HandV4(const Options& options) : options_(options) {
  request_count_ = 0;
  rtr_message_counter_.clear();

  for (int finger_idx = 0; finger_idx < FINGER_MAX; finger_idx++) {
    for (int joint_idx = 0; joint_idx < JONIT_PER_FINGER; joint_idx++) {
      // Get the gains from the parser (already converted to torque units) and create the PD controller.
      const auto& kp = options_.k_p_table[finger_idx * JONIT_PER_FINGER + joint_idx];
      const auto& kd = options_.k_d_table[finger_idx * JONIT_PER_FINGER + joint_idx];
      command_.joints[finger_idx][joint_idx].position_controller = std::make_unique<PdController>(kp, kd);
      command_.joints[finger_idx][joint_idx].torque_max = options_.torque_limit_table[finger_idx * JONIT_PER_FINGER + joint_idx];
    }
  }

  if (!_initialize()) {
    throw std::runtime_error("Failed to initialize Allegro HandV4");
  }
}

/**
 * @brief Destroys the HandV4 object.
 *
 * This destructor ensures a graceful shutdown by sending a "SYSTEM_OFF" command
 * to the hand, which turns off the servo power. This is crucial to prevent the hand
 * before the object is destroyed.
 */
HandV4::~HandV4() { _set_system_on_off(false); }

/**
 * @brief Converts the current state of the hand to a formatted string.
 *
 * This method provides a human-readable representation of the hand's current state,
 * including IMU data, and for each joint: position, velocity, torque, and temperature.
 * It is primarily used for logging and debugging purposes.
 * @return A string representing the current joint positions, velocities, torques, and temperatures.
 */
std::string HandV4::State::to_string() const {
  std::string s;

  s = fmt::format("IMU: rpy=[{:04X}, {:04X}, {:04X}]\n", imu.rpy[0], imu.rpy[1], imu.rpy[2]);

  for (int i = 0; i < FINGER_MAX; ++i) {
    s += fmt::format("Finger {}:\n", i);
    s += fmt::format("  Position(d):  [{:>8.2f}, {:>8.2f}, {:>8.2f}, {:>8.2f}]\n", joints[i][0].position * 180.0 / M_PI,
                     joints[i][1].position * 180.0 / M_PI, joints[i][2].position * 180.0 / M_PI, joints[i][3].position * 180.0 / M_PI);
    s += fmt::format("  Velocity(d/s):[{:>8.2f}, {:>8.2f}, {:>8.2f}, {:>8.2f}]\n", joints[i][0].velocity * 180.0 / M_PI,
                     joints[i][1].velocity * 180.0 / M_PI, joints[i][2].velocity * 180.0 / M_PI, joints[i][3].velocity * 180.0 / M_PI);
    s += fmt::format("  Torque:       [{:>8.4f}, {:>8.4f}, {:>8.4f}, {:>8.4f}]\n", joints[i][0].torque, joints[i][1].torque,
                     joints[i][2].torque, joints[i][3].torque);
    s += fmt::format("  Temperature:  [{:>8.1f}, {:>8.1f}, {:>8.1f}, {:>8.1f}]\n", joints[i][0].temperature, joints[i][1].temperature,
                     joints[i][2].temperature, joints[i][3].temperature);
  }
  return s;
}

/**
 * @brief Parses an incoming CAN message and dispatches it to the appropriate handler.
 *
 * This function is the central callback for received CAN frames. It identifies the
 * message type by its ID and calls the corresponding `_rcv_*` private method to process
 * the data. It operates under a spinlock to ensure thread-safe access to the shared `state_` object.
 * @param message_id The CAN message identifier.
 * @param data A pointer to the raw data payload of the CAN frame.
 * @param data_size The size of the data payload.
 */
void HandV4::_parse_message(uint32_t message_id, uint8_t* data, int data_size) {
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
 * to power on the hand, request its information, and configure the data sampling periods.
 * @return True if initialization is successful, false otherwise.
 */
bool HandV4::_initialize() {

  // Create CAN Interface
  auto decode_io_frame_can = [this](const uint32_t& header, uint8_t* data, int data_size) {
    _parse_message(((can_id_t*)&header)->message_id, data, data_size);
  };

  auto encode_io_frame_header_can = [this](const uint32_t& message_id) {
    can_id_t id_struct = {};
    id_struct.message_id = message_id;
    id_struct.rtr = (RTR_MESSAGE_IDS.count(message_id) > 0);
    uint32_t can_id = 0;
    memcpy(&can_id, &id_struct, sizeof(can_id));
    return can_id;
  };

  comm_io_ = allegro_hand_io::CommIo::Create(options_.io_interface_descriptor, decode_io_frame_can, encode_io_frame_header_can);

  if (comm_io_) {
    _set_system_on_off(true);
    _req_hand_info();
    _set_sampling_period();
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
bool HandV4::_set_system_on_off(bool on_off) {
  signal_rate_monitor_.tick("system_on_off");
  auto msg_id = on_off ? ID_CMD_SYSTEM_ON : ID_CMD_SYSTEM_OFF;
  auto res = comm_io_->send_message(msg_id, nullptr, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return res > 0;
}

/**
 * @brief Requests general information from the hand.
 * This includes hardware/firmware versions, hand type, and status flags.
 * This is sent as an RTR (Remote Transmission Request) frame.
 * @return True on success, false on failure.
 */
bool HandV4::_req_hand_info() { 
  signal_rate_monitor_.tick("req_hand_info");
  return comm_io_->send_message(ID_RTR_HAND_INFO) > 0; 
}

/**
 * @brief Configures the data sampling periods on the hand.
 *
 * This function sends a command to the hand to set the update rate (in milliseconds)
 * for position and temperature data streams. The periods are defined by constants
 * in `v4_spec.hpp`.
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandV4::_set_sampling_period() {
  signal_rate_monitor_.tick("set_sampling_period");
  data_period_t data;
  data.position = POSITION_SAMPLING_PERIOD * 1000;
  data.temperature = TEMPERATURE_SAMPLING_PERIOD * 1000;
  data.imu = 0;
  return comm_io_->send_message(ID_CMD_SET_PERIOD, (uint8_t*)&data, sizeof(data)) > 0;
}

/**
 * @brief Sends a position command to a specific finger.
 * @note This function is currently not implemented as the V4 protocol uses torque commands for control. Position control is handled at a
 * higher level by converting position targets to torque commands via a PD controller.
 * @param finger_idx The index of the finger to command.
 * @return Always returns false.
 */
bool HandV4::_set_position(unsigned int finger_idx) {
  // nothing to do
  (void)finger_idx;
  return false;
}

/**
 * @brief Sends a torque command to a specific finger.
 * This function packs the four joint torque commands for a given finger into a
 * single CAN message and sends it.
 *
 * @param finger_idx The index of the finger (0-3) to command.
 * @return True if the command was sent successfully, false otherwise.
 */
bool HandV4::_set_torque(unsigned int finger_idx) {
  assert(finger_idx < FINGER_MAX);
  data_torque_t data;
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    data.encode(j_idx, command_.joints[finger_idx][j_idx].torque);
  }
  signal_rate_monitor_.tick(fmt::format("snd_torque{}", finger_idx));
  return comm_io_->send_message(uint32_t(ID_CMD_SET_TORQUE + finger_idx), (uint8_t*)&data, sizeof(data_torque_t)) > 0;
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
void HandV4::_rcv_position(unsigned int finger_idx, const uint8_t* data) {
  assert(finger_idx < FINGER_MAX);
  for (int j_idx = 0; j_idx < JONIT_PER_FINGER; j_idx++) {
    auto position = reinterpret_cast<const data_position_t*>(data)->decode(j_idx);
    state_.joints[finger_idx][j_idx].set_position(position);
  }
  signal_rate_monitor_.tick(fmt::format("rcv_position{}", finger_idx));
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
void HandV4::_rcv_temperature(unsigned int finger_idx, const uint8_t* data) {
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
void HandV4::_rcv_imu(const uint8_t* data) {
  data_imu_rpy_t* imu_data = (data_imu_rpy_t*)data;
  state_.imu.rpy[0] = imu_data->decode(0);
  state_.imu.rpy[1] = imu_data->decode(1);
  state_.imu.rpy[2] = imu_data->decode(2);
}

/**
 * @brief Handles a received hand information frame.
 *
 * This private callback is triggered when a CAN message containing general
 * hand information is received. It decodes the data and logs it for debugging
 * and informational purposes, storing it in `state_.main_board_info`.
 * @param data A pointer to the raw CAN data payload.
 */
void HandV4::_rcv_info(const uint8_t* data) { 
  state_.main_board_info = *((data_info_t*)data); 
  signal_rate_monitor_.tick("rcv_info");
}

/**
 * @brief Handles a received serial number frame.
 *
 * This private callback is triggered when a CAN message containing the hand's
 * serial number is received. It decodes the data and logs it.
 * @param data A pointer to the raw CAN data payload.
 */
void HandV4::_rcv_serial(const uint8_t* data) {
  data_serial_t* serial_data = (data_serial_t*)data;
  state_.serial_number = serial_data->to_string();
  signal_rate_monitor_.tick("rcv_serial");
}

/*
 *
 */
void HandV4::JointCommand::set_torque(double q_desired, double q, double q_dot) {
  assert(position_controller);
  position = q_desired;
  float tau = position_controller->update(q_desired, q, q_dot);
  torque = std::clamp(tau, -torque_max, torque_max);
}

/*
 * Public API
 */

/**
 * @brief A periodic function called by an external timer.
 *
 * This function is responsible for tasks that need to be executed periodically,
 * such as requesting data from the hand or performing other maintenance tasks.
 * It sends RTR messages to request non-real-time data like hand info and serial number.
 */
void HandV4::periodic_request() {
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

#if 0
  if (rtr_msg_ids.size() == rtr_send_count) {
    SPDLOG_TRACE("ALL RTR MESSAGES SEND OK");
  } else {
    SPDLOG_ERROR("NOT ALL RTR MESSAGES SEND OK");
  }
#endif 

  request_count_++; 
}

/**
 * @brief Gets the serial number of the hand.
 * @return The serial number as a string.
 */
std::string HandV4::get_serial_number() const {
  std::lock_guard<SpinLock> lock(lock_);
  return state_.serial_number;
}

/**
 * @brief Gets the current state of the main board.
 * This includes status flags (e.g., faults, servo power) and temperature.
 * @return A `data_info_t` structure containing the main board state.
 */
data_info_t HandV4::get_main_board_state() const {
  std::lock_guard<SpinLock> lock(lock_);
  return state_.main_board_info;
}

/**
 * @brief Reads a specific group of gains from the SDK's internal command structure.
 *
 * This function retrieves the current values for a specified gain group (p_gain, d_gain, or torque_max)
 * for all joints and returns them in a vector, ordered according to the SDK's joint sequence.
 * It provides thread-safe access to the gain values.
 *
 * @param group_name The name of the gain group to read.
 * @return A vector of gain values.
 */
std::vector<double> HandV4::read_gains(const char* group_name) {
  
  auto read_p_gains = [this]{
    std::vector<double> gains;
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      gains.push_back(command_.joints[finger_idx][joint_idx].position_controller->get_kp());
    }
    return gains;
  };

  auto read_d_gains = [this]{
    std::vector<double> gains;
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      gains.push_back(command_.joints[finger_idx][joint_idx].position_controller->get_kd()); 
    }
    return gains;
  };

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
  if (strcmp(group_name, HandV4::P_GAIN_GROUP) == 0) {
    table = read_p_gains();
  } else if (strcmp(group_name, HandV4::D_GAIN_GROUP) == 0) {
    table = read_d_gains(); 
  } else if (strcmp(group_name, HandV4::TORQUE_LIMIT_GROUP) == 0) {
    table = read_torque_max();
  }

  return table;
}

/**
 * @brief Writes a specific group of gains to the SDK's internal command structure.
 *
 * This function updates the values for a specified gain group (p_gain, d_gain, or torque_max)
 * for all joints using the provided vector of gains. The input vector must be ordered_
 * according to the SDK's internal joint sequence. It provides thread-safe access.
 *
 * @param group_name The name of the gain group to write.
 * @param gains A vector of gain values to set.
 */
void HandV4::write_gains(const char* group_name, const std::vector<double>& gains) {

  assert(gains.size() == JONIT_MAX);

  auto write_p_gains = [this](const std::vector<double>& new_gains) {
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      command_.joints[finger_idx][joint_idx].position_controller->set_kp(new_gains[io_idx]);
    }
  };

  auto write_d_gains = [this](const std::vector<double>& new_gains) {
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      command_.joints[finger_idx][joint_idx].position_controller->set_kd(new_gains[io_idx]);
    }
  };

  auto write_torque_max = [this](const std::vector<double>& torque_limits) {
    for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
      int finger_idx = io_idx / JONIT_PER_FINGER;
      int joint_idx = io_idx % JONIT_PER_FINGER;
      command_.joints[finger_idx][joint_idx].torque_max = torque_limits[io_idx];
    }
  };

  std::lock_guard<SpinLock> lock(lock_);

  if (strcmp(group_name, HandV4::P_GAIN_GROUP) == 0) {
    write_p_gains(gains);
  } else if (strcmp(group_name, HandV4::D_GAIN_GROUP) == 0) {
    write_d_gains(gains);
  } else if (strcmp(group_name, HandV4::TORQUE_LIMIT_GROUP) == 0) {
    write_torque_max(gains);
  }
}

/**
 * @brief Reads the current state from the SDK and populates the ros2_control state arrays.
 * This function is called by the `hardware_interface` in its `read()` method.
 * It copies the latest joint data from the internal `state_` object to the arrays
 * provided by `ros2_control`, ensuring thread-safe access with a lock.
 * @param position_state Pointer to the position state array for ros2_control (16 elements).
 * @param velocity_state Pointer to the velocity state array for ros2_control.
 * @param torque_state Pointer to the torque state array for ros2_control.
 * @param temperature_state Pointer to the temperature state array for ros2_control.
 */
void HandV4::read_states(double* position_state, double* velocity_state, double* torque_state, double* temperature_state) {
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
 * This function is called by the `hardware_interface` in its `read()` method.
 * It copies the latest IMU data from the internal `state_` object to the arrays
 * provided by `ros2_control`.
 * @param acc Pointer to the linear acceleration state array [x, y, z] for ros2_control.
 * @param gyr Pointer to the angular velocity state array [x, y, z] for ros2_control.
 */
void HandV4::read_imu_states(double* acc, double* gyr) {
  std::lock_guard<SpinLock> lock(lock_);
  for (int i = 0; i < 3; ++i) {
    acc[i] = state_.imu.acc[i];
    gyr[i] = state_.imu.gyr[i];
  }
}
/**
 * @brief Writes commands from ros2_control to the SDK.
 *
 * This function takes the command arrays from ros2_control, calculates the
 * necessary torques based on the current control mode (position or torque),
 * updates the internal command and state objects, and sends the torque commands
 * to the hand via CAN.
 *
 * @param position_command Pointer to the position command array from ros2_control.
 * @param velocity_command Pointer to the velocity command array from ros2_control (currently unused).
 * @param torque_command Pointer to the effort (torque) command array from ros2_control (used in torque control mode).
 */
void HandV4::write_commands(const double* position_command, const double* velocity_command, const double* torque_command) {
  (void)velocity_command;

  for (int io_idx = 0; io_idx < JONIT_MAX; io_idx++) {
    int finger_idx = io_idx / JONIT_PER_FINGER;
    int joint_idx = io_idx % JONIT_PER_FINGER; 

    if (control_mode_ == POSITION_CONTROL) {
      std::lock_guard<SpinLock> lock(lock_);
      double cur_q = state_.joints[finger_idx][joint_idx].position;
      double cur_q_dot = state_.joints[finger_idx][joint_idx].velocity;
      double desired_q = position_command[io_idx];
      command_.joints[finger_idx][joint_idx].set_torque(desired_q, cur_q, cur_q_dot);
    } else { // TORQUE_CONTROL
      command_.joints[finger_idx][joint_idx].set_torque(torque_command[io_idx]);
    }

    {
      // Since V4 does not provide torque feedback, the desired torque is recorded in the state.
      std::lock_guard<SpinLock> lock(lock_); 
      state_.joints[finger_idx][joint_idx].torque = command_.joints[finger_idx][joint_idx].torque;
    }
  }

  for (int finger_idx = 0; finger_idx < FINGER_MAX; finger_idx++) {
    _set_torque(finger_idx);
  }
}

} // namespace v4_sdk
