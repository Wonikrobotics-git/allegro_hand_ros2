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
 * @file plexus_sdk.hpp
 * @brief Defines the SDK classes for interfacing with the Allegro Hand Plexus.
 * @author ('c')void
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include <allegro_hand_io/comm.hpp>
#include <allegro_hand_utility/utility.hpp>

#include "plexus_spec.hpp"

using allegro_hand_utility::SignalRateMonitor;
using allegro_hand_utility::SpinLock;

namespace plexus_sdk {

/**
 * @class HandPlexus
 * @brief Main SDK class for controlling the Allegro Hand Plexus.
 *
 * This class encapsulates all the logic for communication, state management,
 * and control of the Allegro Hand. It provides an interface for `ros2_control`
 * to read states and write commands.
 */
class HandPlexus {
public:
  /**
   * @struct Options
   * @brief Configuration options for initializing the HandPlexus SDK.
   */
  struct Options {
    /// @brief The name of the io interface to use (e.g., "can:can0").
    std::string io_interface_descriptor;
    /// @brief The ID of the hand, used for CAN communication.
    uint8_t hand_id;
    /// @brief A flat array of proportional gains for all 16 joints.
    std::vector<double> k_p_table;
    /// @brief A flat array of derivative gains for all 16 joints.
    std::vector<double> k_d_table;
    /// @brief A flat array of torque limits for all 16 joints.
    std::vector<double> torque_limit_table;
  };

  ///// PARAMETER GROUPS ///////

  /// @brief The name for the torque_limit parameter group.
  static const char* TORQUE_LIMIT_GROUP;

private:
  /**
   * @struct JointState
   * @brief Holds the current state of a single joint, including filtered values.
   */
  struct JointState {
    /// @brief Current joint position in radians.
    float position;
    /// @brief Current joint velocity in radians/sec.
    float velocity;
    /// @brief Current motor temperature in Celsius.
    float temperature;
    /// @brief Current commanded torque
    float torque;
    /// @brief Low-pass filter for the velocity signal.
    allegro_hand_utility::LowpassFilter velocity_filter;
    /// @brief Filter to derive velocity from the position signal.
    allegro_hand_utility::DerivativeFilter position_derivatice_filter;

    JointState()
        : velocity_filter(1.0 / POSITION_SAMPLING_PERIOD, LOWPASS_FILTER_CUTOFF_FREQ),
          position_derivatice_filter(1.0 / POSITION_SAMPLING_PERIOD) {
      position = 0;
      velocity = 0;
      torque = 0;
      temperature = 0;
    }
    /** @brief Sets the joint position, applying a low-pass filter and deriving velocity. */
    inline void set_position(const float value) { position = value; }
    /** @brief Sets the joint velocity, applying a low-pass filter. */
    inline void set_velocity(float value) {
      velocity = value; // No Filtering
      // velocity = velocity_filter.filtering(value);
    }
    /** @brief Sets the joint temperature. */
    inline void set_temperature(float value) { temperature = value; }
    /** @brief Sets the joint torque state (for feedback). */
    inline void set_torque(float value) { torque = value; }
  };

  /**
   * @struct ImuState
   * @brief Holds the state of the Inertial Measurement Unit (IMU).
   */
  struct ImuState {
    /// @brief Linear acceleration vector (x, y, z).
    Eigen::Vector3f acc;
    /// @brief Angular velocity vector (x, y, z).
    Eigen::Vector3f gyr;
    /// @brief Raw roll, pitch, yaw data from the device (for testing/debugging).
    uint16_t rpy[3]; // FIXME. Quaternion

    ImuState() {
      acc.setZero();
      gyr.setZero();
      rpy[0] = 0;
      rpy[1] = 0;
      rpy[2] = 0;
    }
  };

  /**
   * @struct JointCommand
   * @brief Holds the command values for a single joint.
   */
  struct JointCommand {
    /// @brief Desired joint position in radians (used in position control mode).
    float position;
    /// @brief Desired torque
    float torque;
    /// @brief Maximum allowed torque
    float torque_max;
    /// @brief The PD controller instance for this joint.
    // std::unique_ptr<PositionController> position_controller;
    JointCommand() {
      position = 0;
      torque = 0;
    }
    /** @brief Directly sets the desired torque (for torque control mode). */
    inline void set_torque(float tau) { torque = std::clamp(tau, -torque_max, torque_max); }
    inline void set_position(float pos) { position = pos; }
  };

  /** @brief Aggregates the state of the entire hand. */
  struct State {
    /// @brief Array of states for all 16 joints.
    JointState joints[FINGER_MAX][JONIT_PER_FINGER];
    /// @brief Array of temperatures for each finger (deprecated, use JointState::temperature).
    float temperature[FINGER_MAX];
    /// @brief The serial number of the hand.
    std::string serial_number;
    /// @brief Information from the main control board.
    data_info_t main_board_info;
    /// @brief State of the IMU.
    ImuState imu;
    std::string to_string() const;
  };

  /** @brief Aggregates the commands for the entire hand. */
  struct Command {
    /// @brief Array of commands for all 16 joints.
    JointCommand joints[FINGER_MAX][JONIT_PER_FINGER];
    std::string to_string() const;
  };

  /// @brief Spinlock for thread-safe access to shared data.
  mutable SpinLock lock_;

  /// @brief Configuration options passed during construction.
  Options options_;

  /// @brief Communication interface object (e.g., for CAN).
  std::shared_ptr<allegro_hand_io::CommIo> comm_io_;

  /// @brief Current operation mode (position or torque).
  OperationMode operation_mode_;
  /// @brief Current state of the hand.
  State state_;
  /// @brief Current command for the hand.
  Command command_;

  // --- Debugging members ---
  /// @brief Counter for periodic requests.
  int request_count_;
  /// @brief Counter for received RTR messages, indexed by message ID.
  std::unordered_map<int, int> rtr_message_counter_;

  SignalRateMonitor signal_rate_monitor_;

  /** @brief Initializes the communication interface and the hand. */
  bool _initialize();
  /** @brief Parses an incoming CAN message and dispatches it to the correct handler. */
  void _parse_message(uint32_t message_id, uint8_t* data, int data_size);

  /** @brief Sends a command to turn the servo power on or off. */
  bool _set_system_on_off(bool on_off);
  /** @brief Sends a command to set the hand's operation mode (e.g., position or torque control). */
  bool _set_operation_mode(OperationMode mode);
  /** @brief Sends a position command to a finger. */
  bool _set_position(unsigned int finger_idx);
  /** @brief Sends a torque command to a finger. */
  bool _set_torque(unsigned int finger_idx);
  /** @brief Sends a command to configure the hand's data sampling periods. */
  bool _set_sampling_period();
  /** @brief Sends an RTR frame to request general hand information. */
  bool _req_hand_info();

  /** @brief Callback for received position data. */
  void _rcv_position(unsigned int finger_idx, const uint8_t* data);
  /** @brief Callback for received velocity data. */
  void _rcv_velocity(unsigned int finger_idx, const uint8_t* data);
  /** @brief Callback for received torque data. */
  void _rcv_torque(unsigned int finger_idx, const uint8_t* data);
  /** @brief Callback for received temperature data. */
  void _rcv_temperature(unsigned int finger_idx, const uint8_t* data);
  /** @brief Callback for received IMU data. */
  void _rcv_imu(const uint8_t* data);
  /** @brief Callback for received hand information data. */
  void _rcv_info(const uint8_t* data);
  /** @brief Callback for received serial number data. */
  void _rcv_serial(const uint8_t* data);

public:
  /**
   * @brief Constructs the HandPlexus object.
   * @param options Configuration options including CAN interface and PD gains.
   * @throws std::runtime_error if initialization fails.
   */
  HandPlexus(const Options& options);
  /** @brief Destructor, ensures the hand is powered off safely. */
  virtual ~HandPlexus();

  /** @brief Checks if the hand is initialized and ready. */
  inline bool is_ready() const {
    // A simple check to see if the communication interface is up.
    // A more robust check might involve verifying initial data reception.
    return comm_io_ != nullptr;
  }

  /**
   * @brief Activates the control loop and sets the control mode.
   * @param position_control_mode True for position control, false for torque control.
   */
  void set_control_mode(bool position_control_mode);

  /** @brief A periodic function for requesting non-critical data like temperature. */
  void periodic_request();

  /**
   * @brief Gets the hand's serial number.
   * This method provides thread-safe access to the serial number string.
   * @return The serial number as a std::string.
   */
  std::string get_serial_number() const;

  /**
   * @brief Gets the state of the main board.
   * This method provides thread-safe access to the main board's status information,
   * including version, temperature, and fault flags.
   * @return A data_info_t structure containing the main board state.
   */
  data_info_t get_main_board_state() const;

  /**
   * @brief Reads a specific group of gains from the SDK.
   * @param group_name The name of the gain group to read ('p_gain', 'd_gain', or 'torque_max').
   * @return A vector of gain values, ordered according to the SDK's internal joint sequence.
   */
  std::vector<double> read_gains(const char* group_name);

  /**
   * @brief Writes a specific group of gains to the SDK.
   * @param group_name The name of the gain group to write ('p_gain', 'd_gain', or 'torque_max').
   * @param gains A vector of gain values, ordered according to the SDK's internal joint sequence.
   */
  void write_gains(const char* group_name, const std::vector<double>& gains);

  /** @brief Reads the current state from the SDK for ros2_control. */
  void read_states(double* position_state, double* velocity_state, double* torque_state, double* temperature_state);
  /** @brief Reads the current IMU state from the SDK for ros2_control. */
  void read_imu_states(double* acc, double* gyr);
  /** @brief Writes commands from ros2_control to the SDK. */
  void write_commands(const double* position_command, const double* velocity_command, const double* torque_command);

  inline const SignalRateMonitor& get_rate_monitor() const { return signal_rate_monitor_; }
};

} // namespace plexus_sdk
