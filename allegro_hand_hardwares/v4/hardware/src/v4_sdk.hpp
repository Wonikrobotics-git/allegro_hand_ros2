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
 * @file v4_sdk.hpp
 * @brief Defines the SDK classes for interfacing with the Allegro Hand V4.
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

#include "v4_spec.hpp"

using allegro_hand_utility::SignalRateMonitor;
using allegro_hand_utility::SpinLock;

namespace v4_sdk {

/**
 * @class PdController
 * @brief A simple Proportional-Derivative (PD) controller.
 *
 * This class calculates a control output (typically torque or force) based on the
 * position error (Proportional term) and the current velocity (Derivative term).
 * The control law is: `output = Kp * (position_desired - position_current) - Kd * velocity_current`.
 *
 * @tparam T The data type for gains and state variables (e.g., float, double).
 */
class PdController {
  // @brief Proportional gain (Kp).
  double kp_;
  // @brief Derivative gain (Kd).
  double kd_;

public:
  /**
   * @brief Constructs a PdController.
   * @param kp Proportional gain.
   * @param kd Derivative gain.
   */
  PdController(double kp, double kd) : kp_(kp), kd_(kd) {}

  /** @brief Gets the current proportional gain (Kp). */
  inline const double& get_kp() const { return kp_; }

  /** @brief Gets the current derivative gain (Kd). */
  inline const double& get_kd() const { return kd_; }

  /** @brief Sets the proportional gain. */
  inline void set_kp(double kp) { kp_ = kp; }

  /** @brief Sets the derivative gain. */
  inline void set_kd(double kd) { kd_ = kd; }

  /**
   * @brief Calculates the control output (torque).
   *
   * This method applies the PD control law to compute the desired torque.
   * `torque = Kp * (q_desired - q) - Kd * q_dot`
   *
   * @param q_desired The desired joint position.
   * @param q The current joint position.
   * @param q_dot The current joint velocity.
   * @return The calculated torque value.
   */
  inline double update(double q_desired, double q, double q_dot) { return kp_ * (q_desired - q) - kd_ * q_dot; }
};

// using PdController = allegro_hand_utility::PdController;

/**
 * @class HandV4
 * @brief Main SDK class for controlling the Allegro Hand V4.
 *
 * This class encapsulates all the logic for communication, state management,
 * and control of the Allegro Hand. It provides an interface for ros2_control
 * to read states and write commands.
 */
class HandV4 {
public:
  /**
   * @struct Options
   * @brief Configuration options for initializing the HandV4 SDK.
   */
  struct Options {
    /// @brief The name of the io interface to use (e.g., "can:can0").
    std::string io_interface_descriptor;
    /// @brief A flat array of proportional gains for all 16 joints.
    std::vector<double> k_p_table;
    /// @brief A flat array of derivative gains for all 16 joints.
    std::vector<double> k_d_table;
    /// @brief A flat array of torque limits for all 16 joints.
    std::vector<double> torque_limit_table;
  };

  /// @brief The name for the p_gain parameter group.
  static const char* P_GAIN_GROUP;
  /// @brief The name for the d_gain parameter group.
  static const char* D_GAIN_GROUP;
  /// @brief The name for the torque_limit parameter group.
  static const char* TORQUE_LIMIT_GROUP;

private:
  /** @brief Defines the control mode of the hand. */
  enum ControlMode {
    POSITION_CONTROL, ///< Position control mode.
    TORQUE_CONTROL,   ///< Torque control mode.
  };

  struct JointState {
    /// @brief Current joint position in radians.
    float position;
    /// @brief Current joint velocity in radians/sec.
    float velocity;
    /// @brief Current motor temperature in Celsius.
    float temperature;
    /// @brief Current commanded torque (normalized PWM ratio [-1, 1]).
    float torque;
    /// @brief Low-pass filter for the position signal.
    allegro_hand_utility::LowpassFilter position_filter;
    /// @brief Low-pass filter for the velocity signal.
    allegro_hand_utility::LowpassFilter velocity_filter;
    /// @brief Filter to derive velocity from the position signal.
    allegro_hand_utility::DerivativeFilter position_derivatice_filter;
    JointState()
        : position_filter(1.0 / POSITION_SAMPLING_PERIOD, LOWPASS_FILTER_CUTOFF_FREQ),
          velocity_filter(1.0 / POSITION_SAMPLING_PERIOD, LOWPASS_FILTER_CUTOFF_FREQ),
          position_derivatice_filter(1.0 / POSITION_SAMPLING_PERIOD) {
      position = 0;
      velocity = 0;
      torque = 0;
      temperature = 0;
    }
    /** @brief Sets the joint position, applying a low-pass filter and deriving velocity. */
    inline void set_position(const float value) {
      position = position_filter.filtering(value);
      set_velocity(position_derivatice_filter.filtering(position));
    }
    /** @brief Sets the joint velocity, applying a low-pass filter. */
    inline void set_velocity(float value) { velocity = velocity_filter.filtering(value); }
    /** @brief Sets the joint temperature. */
    inline void set_temperature(float value) { temperature = value; }
    /** @brief Sets the joint torque state (for feedback). */
    inline void set_torque(float value) { torque = value; }
  };

  struct ImuState {
    /// @brief Linear acceleration vector (x, y, z).
    Eigen::Vector3f acc;
    /// @brief Angular velocity vector (x, y, z).
    Eigen::Vector3f gyr;
    /// @brief Raw roll, pitch, yaw data from the device (for testing/debugging).
    uint16_t rpy[3];
  };

  struct JointCommand {
    /// @brief Desired joint position in radians (used in position control mode).
    float position;
    /// @brief Desired torque (normalized PWM ratio [-1, 1]).
    float torque;
    /// @brief Maximum allowed torque (normalized, positive value).
    float torque_max;
    /// @brief The PD controller instance for this joint.
    std::unique_ptr<PdController> position_controller;
    JointCommand() {
      position = 0;
      torque = 0;
    }
    /** @brief Calculates and sets the desired torque using the PD controller for position control.
     * The output is saturated by `torque_max`. */
    void set_torque(double q_desired, double q, double q_dot);
    /** @brief Directly sets the desired torque (for torque control mode). */
    inline void set_torque(float tau) { torque = tau; }
  };

  /** @brief Aggregates the state of the entire hand. */
  struct State {
    /// @brief Array of states for all 16 joints.
    JointState joints[FINGER_MAX][JONIT_PER_FINGER];
    /// @brief Array of temperatures for each finger (deprecated, use JointState::temperature).
    float temperature[FINGER_MAX]; // celsius
    ///
    std::string serial_number;
    ///
    data_info_t main_board_info;
    ///
    ImuState imu;
    std::string to_string() const;
  };

  /** @brief Aggregates the commands for the entire hand. */
  struct Command {
    /// @brief Array of commands for all 16 joints.
    JointCommand joints[FINGER_MAX][JONIT_PER_FINGER];
  };

  /// @brief Spinlock for thread-safe access to shared data.
  mutable SpinLock lock_;

  /// @brief Configuration options passed during construction.
  Options options_;

  /// @brief Communication interface object (e.g., for CAN).
  std::shared_ptr<allegro_hand_io::CommIo> comm_io_;

  /// @brief Current control mode (position or torque).
  ControlMode control_mode_;
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
  /** @brief Sends a position command to a finger (not implemented for V4). */
  bool _set_position(unsigned int finger_idx);
  /** @brief Sends a torque command to a finger. */
  bool _set_torque(unsigned int finger_idx);
  /** @brief Sends a command to configure the hand's data sampling periods. */
  bool _set_sampling_period();
  /** @brief Sends an RTR frame to request general hand information. */
  bool _req_hand_info();

  /** @brief Callback for received position data. */
  void _rcv_position(unsigned int finger_idx, const uint8_t* data);
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
   * @brief Constructs the HandV4 object.
   * @param options Configuration options including CAN interface and PD gains.
   * @throws std::runtime_error if initialization fails.
   */
  HandV4(const Options& options);
  /** @brief Destructor, ensures the hand is powered off safely. */
  virtual ~HandV4();

  /** @brief Checks if the hand is initialized and ready. */
  inline bool is_ready() const { return true; }
  /**
   * @brief Activates the control loop and sets the control mode.
   * @param position_control_mode True for position control, false for torque control.
   */
  inline void set_control_mode(bool position_control_mode = true) {
    control_mode_ = position_control_mode ? POSITION_CONTROL : TORQUE_CONTROL;
  }
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

} // namespace v4_sdk
