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
 * @file hardware_interface.cpp
 * @brief ros2_control hardware interface(plugin) for the Allegro Hand Plexus.
 * @author ('c')void
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>
#include <spdlog/spdlog.h>

#include "plexus_sdk.hpp"
#include <allegro_hand_plexus_hardware/hardware_interface.hpp>
#include <allegro_hand_utility/ros_utility.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp> // For DiagnosticStatus constants
#include <diagnostic_updater/diagnostic_updater.hpp> // For diagnostics
#include <realtime_tools/realtime_buffer.hpp>

using allegro_hand_utility::HardwareInfoParser;

namespace allegro_hand_plexus_hardware {

/// @brief Helper macro to get a pointer to the DeviceNode instance.
#define GetDeviceNode() (std::dynamic_pointer_cast<DeviceNode>(dev_node_))
/// @brief Helper macro to get a pointer to the SDK device instance.
#define GetDevice() GetDeviceNode()->get_device()
/// @brief Helper macro to get the logger for this hardware interface.
#define GetLogger() rclcpp::get_logger("AllegroHandPlexusHardwareInterface")

/**
 * @class Diagnostics
 * @brief Manages and publishes diagnostic information for the hardware.
 *
 * This class uses the `diagnostic_updater` to periodically check the status of the
 * Allegro Hand and publish it to the `/diagnostics` topic. It reports on connection
 * status, servo power, temperature faults, and communication errors.
 */
class Diagnostics {
  /// @brief The ROS 2 node to which the updater is attached.
  rclcpp::Node* node_;
  /// @brief A shared pointer to the SDK device instance.
  std::shared_ptr<plexus_sdk::HandPlexus> device_;
  /// @brief The diagnostic updater instance.
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  /**
   * @brief Diagnostic task that checks the overall hardware status.
   *
   * This function is called periodically by the updater. It checks for faults,
   * servo status, temperature, and other device information, then populates
   * the DiagnosticStatusWrapper.
   * @param stat The diagnostic status wrapper to be filled.
   */
  void hardware_status_task(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!device_ || !device_->is_ready()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device not connected or not ready.");
      stat.add("Status", "Not Initialized");
      return;
    }

    auto main_board_state = device_->get_main_board_state();
    auto serial_number = device_->get_serial_number();

    // Read current joint states to get temperatures
    double pos[plexus_sdk::JONIT_MAX], vel[plexus_sdk::JONIT_MAX], eff[plexus_sdk::JONIT_MAX], temp[plexus_sdk::JONIT_MAX];
    device_->read_states(pos, vel, eff, temp);

    auto rate_monitoring_results = device_->get_rate_monitor().get_info();

    bool has_error = main_board_state.joint_status.overload_error || main_board_state.joint_status.electrical_shock_error ||
                     main_board_state.joint_status.temperature_error || main_board_state.joint_status.input_voltage_error ||
                     main_board_state.joint_status.f1_connection_error || main_board_state.joint_status.f2_connection_error ||
                     main_board_state.joint_status.f3_connection_error || main_board_state.joint_status.f4_connection_error;

    // Check for faults
    // The order of checks determines the priority of the summary message.
    if (has_error) {
      std::string error_msg = "Device has errors: ";
      if (main_board_state.joint_status.overload_error)
        error_msg += "Overload ";
      if (main_board_state.joint_status.electrical_shock_error)
        error_msg += "ElectricalShock ";
      if (main_board_state.joint_status.temperature_error)
        error_msg += "Temperature ";
      if (main_board_state.joint_status.input_voltage_error)
        error_msg += "InputVoltage ";
      if (main_board_state.joint_status.f1_connection_error)
        error_msg += "F1_Connection ";
      if (main_board_state.joint_status.f2_connection_error)
        error_msg += "F2_Connection ";
      if (main_board_state.joint_status.f3_connection_error)
        error_msg += "F3_Connection ";
      if (main_board_state.joint_status.f4_connection_error)
        error_msg += "F4_Connection ";
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Device is OK.");
    }

    // Add detailed information
    stat.add("Serial Number", serial_number);
    stat.add("Hardware Version", fmt::format("0x{:X}", (int)main_board_state.hand_version));
    stat.add("Firmware Version", fmt::format("0x{:X}", (int)main_board_state.firmware_version));
    stat.add("Hand Type", main_board_state.hand_type == 0x01 ? "Right" : "Left");
    stat.add("Hub Temperature (C)", std::to_string(main_board_state.hub_temperature));
    stat.add("Overload Error", std::to_string(main_board_state.joint_status.overload_error));
    stat.add("Electrical Shock Error", std::to_string(main_board_state.joint_status.electrical_shock_error));
    stat.add("Temperature Error", std::to_string(main_board_state.joint_status.temperature_error));
    stat.add("Input Voltage Error", std::to_string(main_board_state.joint_status.input_voltage_error));
    stat.add("Finger Connection Errors",
             fmt::format("F1: {}, F2: {}, F3: {}, F4: {}", (int)main_board_state.joint_status.f1_connection_error,
                         (int)main_board_state.joint_status.f2_connection_error, (int)main_board_state.joint_status.f3_connection_error,
                         (int)main_board_state.joint_status.f4_connection_error));

    // Add joint temperatures
    stat.addf("Index Finger Temp (C)", "%.1f, %.1f, %.1f, %.1f", temp[0], temp[1], temp[2], temp[3]);
    stat.addf("Middle Finger Temp (C)", "%.1f, %.1f, %.1f, %.1f", temp[4], temp[5], temp[6], temp[7]);
    stat.addf("Ring Finger Temp (C)", "%.1f, %.1f, %.1f, %.1f", temp[8], temp[9], temp[10], temp[11]);
    stat.addf("Thumb Temp (C)", "%.1f, %.1f, %.1f, %.1f", temp[12], temp[13], temp[14], temp[15]);

    // Add communication rate monitoring results
    for (const auto& pair : rate_monitoring_results) {
      stat.addf("Signal: " + pair.first, "Freq: %.2f Hz, Jitter: %.3f ms, Cnt : %d", pair.second.frequency, pair.second.jitter * 1000.0,
                pair.second.count);
    }
  }

public:
  /**
   * @brief Constructs the Diagnostics handler.
   * @param node The ROS 2 node to which the updater will be attached.
   */
  explicit Diagnostics(rclcpp::Node* node) : node_(node), device_(nullptr) {
    updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID("allegro_hand_plexus");
    updater_->add("Hardware Status", this, &Diagnostics::hardware_status_task);
  }
  ~Diagnostics() = default;
  /**
   * @brief Forces an immediate update of the diagnostics.
   */
  inline void update() { updater_->force_update(); }

  /**
   * @brief Binds the SDK device instance to the diagnostics handler.
   * @param device A shared pointer to the SDK device instance.
   */
  inline void bind_device(const std::shared_ptr<plexus_sdk::HandPlexus>& device) { device_ = device; }

  /**
   * @brief Sets a parameter for the diagnostics updater.
   * Currently handles 'diagnostic_updater.period'.
   * @param parameter The parameter to be set.
   * @return True if the parameter was handled, false otherwise.
   */
  bool set_parameter(const rclcpp::Parameter& parameter) {
    if (parameter.get_name() == "diagnostic_updater.period") {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        updater_->setPeriod(parameter.as_double());
        return true;
      }
    }
    // Return false if the parameter is not handled by this class.
    return false;
  }
};

/*
 * @class ParameterHandler
 * @brief Manages ROS 2 parameters for the hardware interface.
 *
 * This class encapsulates all logic related to ROS 2 parameters. It declares parameters for
 * torque limits for each joint.
 * It provides a callback to handle real-time parameter updates from external tools
 * like `ros2 param set` or `rqt_reconfigure`. The class also includes helper functions
 * to manage the mapping between ROS parameter names and the internal data structures
 * of the SDK.
 *
 */
class ParameterHandler {
  /// @brief The ROS 2 node for parameter handling.
  rclcpp::Node* node_;
  /// @brief A shared pointer to the hardware info parser.
  std::shared_ptr<HardwareInfoParser> info_parser_;
  /// @brief A shared pointer to the SDK device instance.
  std::shared_ptr<plexus_sdk::HandPlexus> device_;
  /// @brief A shared pointer to the diagnostics handler.
  std::shared_ptr<Diagnostics> diagnostics_;

  /// @brief Callback handle for parameter changes.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_set_callback_;

  /// @brief Map of torque limits, keyed by base joint name.
  std::map<std::string, double> torque_limit_table_;
  /// @brief Map of initial positions, keyed by base joint name.
  std::map<std::string, double> initial_position_table_;

  /**
   * @brief Encodes a parameter name for ROS 2.
   *
   * Creates a ROS 2-compliant parameter name in the format "group/finger_name/joint_name".
   * @param param_group The parameter group (e.g., "p_gain").
   * @param joint_name The base name of the joint (e.g., "joint00").
   * @return The encoded parameter name string.
   */
  std::string encode_finger_parameter_name(const std::string& param_group, const std::string& joint_name) {
    auto finger_name = info_parser_->get_finger_name(joint_name);

    // The joint name format is 'joint10', 'joint11', etc. which is valid for ROS parameters.
    // No replacement is needed.
    return fmt::format("{}/{}/{}", param_group, finger_name, joint_name);
  };

  /**
   * @brief Decodes a ROS 2 parameter name into its group and original joint name.
   *
   * This is the reverse of `encode_finger_parameter_name`. It parses a string like "torque_limit/Index/joint10"
   * and returns the base group ("torque_limit") and the original joint name ("joint10").
   * @param parameter_name The full ROS 2 parameter name.
   * @return A pair containing the group name and the original joint name.
   */
  std::pair<std::string, std::string> decode_finger_parameter_name(const std::string& parameter_name) {
    // Example parameter_name: "0_p_gain/Index/joint10" or "p_gain/Index/joint10"
    std::vector<std::string> parts;
    boost::split(parts, parameter_name, boost::is_any_of("/"));

    // Expected format: [group_name, finger_name, joint_name]
    if (parts.size() != 3) {
      return {}; // Invalid format
    }

    std::string param_group_name = parts[0];
    const std::string& joint_name = parts[2];

    // We check for the base group patterns and return the base name if found.
    if (param_group_name.find(plexus_sdk::HandPlexus::TORQUE_LIMIT_GROUP) != std::string::npos) {
      return {plexus_sdk::HandPlexus::TORQUE_LIMIT_GROUP, joint_name};
    }
    return {param_group_name, joint_name}; // Return original if no match
  }

  /**
   * @brief Callback for handling parameter changes.
   *
   * This function is triggered when a `ros2 param set` command is issued. It decodes the parameter
   * name to identify the group (`torque_limit`) and joint, updates the local table, and then
   * calls the SDK's `write_gains` function to apply the change to the hardware in real-time.
   * @param parameters The list of parameters being changed.
   * @return A result indicating if the parameter change was successful.
   */
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
    SPDLOG_TRACE("DeviceNode::set_parameters_callback");

    size_t success_count = 0;

    for (const auto& param : parameters) {
      // Delegate handling for parameters that are not part of the gain groups.
      if (param.get_name().find("diagnostic_updater") != std::string::npos) {
        RCLCPP_INFO(node_->get_logger(), "Handled '%s' parameter change.", param.get_name().c_str());
        if (diagnostics_->set_parameter(param)) {
          success_count++;
        }
        continue;
      }

      // Process parameters belonging to the torque limit group.
      auto [param_group_name, joint_name] = decode_finger_parameter_name(param.get_name());

      if (param_group_name.empty() || joint_name.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid parameter name format: %s", param.get_name().c_str());
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;
        return result;
      }

      if (param_group_name != plexus_sdk::HandPlexus::TORQUE_LIMIT_GROUP) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid parameter group: %s", param_group_name.c_str());
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;
        return result;
      }

      std::map<std::string, double>* table_to_update = nullptr;

      if (param_group_name == plexus_sdk::HandPlexus::TORQUE_LIMIT_GROUP) {
        torque_limit_table_[joint_name] = param.as_double();
        table_to_update = &torque_limit_table_;
      }
      if (device_ && table_to_update) {
        device_->write_gains(param_group_name.c_str(), serialize_table(*table_to_update));
        success_count++;
      }
    }

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
  }

  /*
   * @brief Deserializes a vector of SDK-ordered values into a map.
   *
   * Converts a vector of values, ordered by the SDK's joint sequence, into a
   * map where keys are the base joint names.
   * @param sdk_ordered_values The vector of values from the SDK.
   * @return A map of joint names to values.
   */
  std::map<std::string, double> deserialize_table(const std::vector<double>& sdk_ordered_values) const {
    const auto& joint_names = info_parser_->sdk_ordered_joint_base_names();
    assert(joint_names.size() == sdk_ordered_values.size());
    std::map<std::string, double> result_map;
    std::transform(joint_names.begin(), joint_names.end(), sdk_ordered_values.begin(), std::inserter(result_map, result_map.end()),
                   [](const std::string& name, double value) { return std::make_pair(name, value); });

    return result_map;
  }

  /**
   * @brief Serializes a map of joint values into a vector ordered for the SDK.
   *
   * Converts a map of joint names to values into a vector of doubles, ordered
   * according to the SDK's internal joint sequence. This is necessary before calling `write_gains`.
   * @param table The map of joint names to values.
   * @return A vector of values ordered for the SDK.
   */
  std::vector<double> serialize_table(const std::map<std::string, double>& table) const {
    std::vector<double> result;
    result.reserve(info_parser_->sdk_ordered_joint_base_names().size());
    for (const auto& joint_name : info_parser_->sdk_ordered_joint_base_names()) {
      result.push_back(table.at(joint_name));
    }
    return result;
  }

  /*
   * @brief Initializes and declares all managed ROS 2 parameters.
   * This function follows a two-step initialization process:
   * 1. Sets the initial parameter values based on the factory defaults from the `HardwareInfoParser`.
   * 2. Declares these parameters to the ROS 2 parameter server, allowing them to be overridden by user configurations at launch.
   */
  void init_parameters() {
    torque_limit_table_ = deserialize_table(info_parser_->torque_limit_table());
    initial_position_table_ = deserialize_table(info_parser_->initial_position_table());

    const auto& joint_position_limit_min_table = deserialize_table(info_parser_->joint_position_limit_min_table());
    const auto& joint_position_limit_max_table = deserialize_table(info_parser_->joint_position_limit_max_table());

    auto torque_limit_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    torque_limit_descriptor.description = "Maximum torque limit for a joint. normalized torque units.";
    torque_limit_descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange torque_range;
    torque_range.from_value = 0.0;
    torque_range.to_value = 100.0;
    torque_limit_descriptor.floating_point_range.push_back(torque_range);

    auto initial_position_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    initial_position_descriptor.description = "Initial Position for a joint.";
    initial_position_descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    initial_position_descriptor.read_only = true;

    std::string TORQUE_LIMIT_GROUP_NAME = plexus_sdk::HandPlexus::TORQUE_LIMIT_GROUP;
    std::string INITIAL_POSITION_GROUP_NAME = "initial_position";

    const auto& joint_names = info_parser_->sdk_ordered_joint_base_names();
    for (const auto& joint_name : joint_names) {
      node_->declare_parameter(encode_finger_parameter_name(TORQUE_LIMIT_GROUP_NAME, joint_name), torque_limit_table_.at(joint_name),
                               torque_limit_descriptor);
      node_->declare_parameter(encode_finger_parameter_name(INITIAL_POSITION_GROUP_NAME, joint_name),
                               initial_position_table_.at(joint_name), initial_position_descriptor);
    }
  }

public:
  explicit ParameterHandler(rclcpp::Node* node, const std::shared_ptr<HardwareInfoParser>& info_parser,
                            const std::shared_ptr<Diagnostics> diagnostics = nullptr)
      : node_(node), info_parser_(info_parser), device_(nullptr), diagnostics_(diagnostics) {
    init_parameters();
    param_set_callback_ = node->get_node_parameters_interface()->add_on_set_parameters_callback(
        std::bind(&ParameterHandler::set_parameters_callback, this, std::placeholders::_1));
  }
  ~ParameterHandler() = default;

  /**
   * @brief Binds the SDK device instance to the parameter handler.
   * @param device A shared pointer to the SDK device instance.
   */
  inline void bind_device(const std::shared_ptr<plexus_sdk::HandPlexus>& device) { device_ = device; }

  /// @brief Gets the current torque limits, serialized into a vector in SDK order.
  inline std::vector<double> torque_limit_table() const { return serialize_table(torque_limit_table_); }
  /// @brief Gets the current initial positions, serialized into a vector in SDK order.
  inline std::vector<double> initial_position_table() const { return serialize_table(initial_position_table_); }

}; // ParameterHandler

/*
 * @class DeviceNode
 * @brief A private ROS 2 node that encapsulates the Allegro Hand Plexus SDK instance.
 *
 * This class serves as a wrapper around the `plexus_sdk::HandPlexus` object.
 * It creates a dedicated ROS 2 node to handle non-real-time tasks, such as
 * periodic data requests (e.g., for temperature and device info), without
 * interfering with the real-time control loop managed by `ros2_control`.
 */
class DeviceNode : public rclcpp::Node {
  /// @brief A shared pointer to the SDK device instance.
  std::shared_ptr<plexus_sdk::HandPlexus> device_;
  /// @brief Timer for periodic, non-real-time tasks like diagnostics.
  rclcpp::TimerBase::SharedPtr timer_;
  /// @brief A shared pointer to the parameter handler.
  std::shared_ptr<ParameterHandler> param_handler_;
  /// @brief A shared pointer to the diagnostics handler.
  std::shared_ptr<Diagnostics> diagnostics_handler_;

public:
  /**
   * @brief Constructs the DeviceNode.
   * @param options Node options for the rclcpp::Node.
   */
  explicit DeviceNode(const std::shared_ptr<HardwareInfoParser>& info_parser, const rclcpp::NodeOptions& options)
      : rclcpp::Node(info_parser->dev_node_name(), options) {
    using namespace std::chrono_literals;

    diagnostics_handler_ = std::make_shared<Diagnostics>(this);
    param_handler_ = std::make_shared<ParameterHandler>(this, info_parser, diagnostics_handler_);

    /**
     * @brief Create a timer for non-real-time tasks.
     *
     * This timer periodically calls the SDK's `periodic_request` for requesting
     * non-critical data (like temperature) and also triggers the diagnostic updater.
     */
    auto timer_period = 100ms; // Diagnostics are typically updated at a lower rate
    timer_ = this->create_wall_timer(timer_period, [&]() {
      if (device_) {
        device_->periodic_request();
      }
      if (diagnostics_handler_) {
        diagnostics_handler_->update();
      }
    });
  }
  /**
   * @brief Gets a raw pointer to the underlying SDK device object.
   * @return A pointer to the `plexus_sdk::HandPlexus` instance.
   */
  inline plexus_sdk::HandPlexus* get_device() {
    assert(device_);
    return device_.get();
  }

  /// @brief Gets a shared pointer to the parameter handler.
  inline std::shared_ptr<ParameterHandler> get_parameter_handler() const { return param_handler_; }

  /**
   * @brief Deactivates the device by destroying the SDK instance.
   */
  inline void deactivate() {
    if (device_) {
      device_.reset();
    }
  }
  /**
   * @brief Activates the device by creating the SDK instance.
   * This method constructs the `plexus_sdk::HandPlexus` object, passing it the necessary
   * options parsed from the URDF.
   * @param info_parser A shared pointer to the `HardwareInfoParser` instance.
   * @return True on success, false if the SDK fails to initialize. Returns true if already activated.
   */
  inline bool activate(const std::shared_ptr<HardwareInfoParser>& info_parser) {
    if (device_) {
      return true;
    }

    plexus_sdk::HandPlexus::Options options;
    options.io_interface_descriptor = info_parser->io_interface_descriptor();
    options.hand_id = info_parser->hand_id();
    options.torque_limit_table = param_handler_->torque_limit_table();

    try {
      device_ = std::make_shared<plexus_sdk::HandPlexus>(options);
      param_handler_->bind_device(device_);
      diagnostics_handler_->bind_device(device_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(GetLogger(), "Failed to create HandPlexus SDK instance: %s", e.what());
      return false;
    }

    return true;
  }
};

/**
 * @brief Initializes the hardware interface.
 *
 * This method is called by the `ros2_control` controller manager to initialize the hardware interface.
 * It parses the URDF for joint, sensor, and hardware parameters, validates them,
 * and sets up the internal `DeviceNode` and its executor.
 * @param sysinfo A struct containing information from the URDF.
 * @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::ERROR` otherwise.
 */
hardware_interface::CallbackReturn AllegroHandPlexusHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo) {
  if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  spdlog::set_level(spdlog::level::trace);

  info_ = sysinfo;

  // clang-format off
  HardwareInfoParser::Parameters parser_param;
  parser_param.sdk_ordered_joint_base_names = {
      "joint10", "joint11", "joint12", "joint13", // Index
      "joint20", "joint21", "joint22", "joint23", // Middle
      "joint30", "joint31", "joint32", "joint33", // Ring
      "joint00", "joint01", "joint02", "joint03", // Thumb
  };

  parser_param.default_p_gain_table = {
      2.0, 2.0, 2.0, 2.0, // Index
      2.0, 2.0, 2.0, 2.0, // Middle
      2.0, 2.0, 2.0, 2.0, // Ring
      2.0, 2.0, 2.0, 2.0  // Thumb
  };

  parser_param.default_d_gain_table = {
      0.1, 0.1, 0.1, 0.1, // Index
      0.1, 0.1, 0.1, 0.1, // Middle
      0.1, 0.1, 0.1, 0.1, // Ring
      0.1, 0.1, 0.1, 0.1, // Thumb
  };

  // Nm 
  parser_param.default_torque_limit_table = { 
      0.205, 0.300, 0.205, 0.205, // Index
      0.205, 0.205, 0.205, 0.205, // Middle
      0.205, 0.205, 0.205, 0.205, // Ring
      0.205, 0.205, 0.205, 0.205, // Thumb
  };
  // clang-format on

  hwinfo_parser_ = std::make_shared<HardwareInfoParser>(parser_param, info_);
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  const std::set<std::string> expected_joint_name_if(joint_names.begin(), joint_names.end());

  const std::set<std::string> expected_joint_command_if = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_EFFORT};
  const std::set<std::string> expected_joint_state_if = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
                                                         hardware_interface::HW_IF_EFFORT, "temperature"};

  /*
   * Check Joint Interface
   */
  if (info_.joints.size() != JOINT_NUM) {
    RCLCPP_FATAL(GetLogger(), "Joint size error");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int idx = 0; idx < (int)info_.joints.size(); idx++) {
    const auto& joint = info_.joints.at(idx);

    if (expected_joint_name_if.find(joint.name) == expected_joint_name_if.end()) {
      RCLCPP_FATAL(GetLogger(), "Joint '%s' is not an expected joint name.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Joint Command Interfaces
    if (joint.command_interfaces.size() != expected_joint_command_if.size()) {
      RCLCPP_FATAL(GetLogger(), "Joint '%s' has %zu command interfaces. %zu expected.", joint.name.c_str(), joint.command_interfaces.size(),
                   expected_joint_command_if.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto& iface : joint.command_interfaces) {
      if (expected_joint_command_if.find(iface.name) == expected_joint_command_if.end()) {
        RCLCPP_FATAL(GetLogger(), "Joint '%s' has unexpected command interface: %s", joint.name.c_str(), iface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Joint State Interfaces
    if (joint.state_interfaces.size() != expected_joint_state_if.size()) {
      RCLCPP_FATAL(GetLogger(), "Joint '%s' has %zu state interfaces. %zu expected.", joint.name.c_str(), joint.state_interfaces.size(),
                   expected_joint_state_if.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto& iface : joint.state_interfaces) {
      if (expected_joint_state_if.find(iface.name) == expected_joint_state_if.end()) {
        RCLCPP_FATAL(GetLogger(), "Joint '%s' has unexpected state interface: %s", joint.name.c_str(), iface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  /*
   * Check Sensor Interface
   */
  if (info_.sensors.size() > 0) {
    if (info_.sensors.size() != 1) {
      RCLCPP_FATAL(GetLogger(), "Sensor size error. Expected 1 sensor, but got %zu.", info_.sensors.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& sensor = info_.sensors.at(0);
    if (sensor.name.find("imu") == std::string::npos) {
      RCLCPP_FATAL(GetLogger(), "Sensor name error. Expected a name containing 'imu', but got '%s'.", sensor.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // The IMU sensor should expose 10 state interfaces (4 for orientation, 3 for angular velocity, 3 for linear acceleration)
    if (sensor.state_interfaces.size() != 10) {
      RCLCPP_FATAL(GetLogger(), "Sensor '%s' has %zu state interfaces. 10 expected.", sensor.name.c_str(), sensor.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // clang-format off
    const std::set<std::string> expected_interfaces = {
        "orientation.x",      
        "orientation.y",      
        "orientation.z",         
        "orientation.w",         
        "angular_velocity.x", 
        "angular_velocity.y", 
        "angular_velocity.z", 
        "linear_acceleration.x", 
        "linear_acceleration.y", 
        "linear_acceleration.z"
      };
    // clang-format on 

    for (const auto& iface : sensor.state_interfaces) {
      if (expected_interfaces.find(iface.name) == expected_interfaces.end()) {
        RCLCPP_FATAL(GetLogger(), "Sensor '%s' has an unexpected state interface: %s", sensor.name.c_str(), iface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  dev_node_ = std::make_shared<DeviceNode>(hwinfo_parser_, rclcpp::NodeOptions());
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(dev_node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Configures the hardware interface.
 * This method is called by the controller manager after `on_init`.
 * @param previous_state The previous lifecycle state (unused).
 * @return `CallbackReturn::SUCCESS`.
 */
hardware_interface::CallbackReturn AllegroHandPlexusHardwareInterface::on_configure(const rclcpp_lifecycle::State&) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Exports the state interfaces for all joints and the IMU sensor.
 *
 * This method is called by the controller manager to gather all available state interfaces.
 * It creates `StateInterface` objects for each joint's position, velocity, effort, and temperature,
 * as well as for the IMU's orientation, angular velocity, and linear acceleration.
 * @return A vector of `StateInterface` objects for joint states (position, velocity, effort) and IMU sensor states.
 */
std::vector<hardware_interface::StateInterface> AllegroHandPlexusHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint State Interfaces
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  for (size_t i = 0; i < joint_names.size(); i++) {
    // The order must match the SDK's internal joint order.
    const auto& joint_name = joint_names.at(i);
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_EFFORT, &joint_torque_state_[i]));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint_name, "temperature", &joint_temperature_state_[i]));
  }

  // IMU State Interfaces
  if (info_.sensors.size() > 0) {
    const auto& sensor = info_.sensors.at(0);
    // Expose IMU interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.x", &imu_quat_state_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.y", &imu_quat_state_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.z", &imu_quat_state_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.w", &imu_quat_state_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "linear_acceleration.x", &imu_acc_state_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "linear_acceleration.y", &imu_acc_state_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "linear_acceleration.z", &imu_acc_state_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "angular_velocity.x", &imu_gyr_state_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "angular_velocity.y", &imu_gyr_state_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "angular_velocity.z", &imu_gyr_state_[2]));
  }

  return state_interfaces;
}

/**
 * @brief Exports the command interfaces for all joints. 
 *
 * This method is called by the controller manager to gather all available command interfaces.
 * It creates `CommandInterface` objects for each joint's position and effort, allowing
 * controllers to send commands to the hardware.
 * @return A vector of `CommandInterface` objects for position and effort for each joint, in the order defined by the SDK.
 */
std::vector<hardware_interface::CommandInterface> AllegroHandPlexusHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  for (size_t i = 0; i < joint_names.size(); i++) {
    // The order must match the SDK's internal joint order.
    const auto& joint_name = joint_names.at(i);
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_EFFORT, &joint_torque_command_[i]));
  }

  return command_interfaces;
}

/**
 * @brief Activates the hardware interface.
 *
 * This method is called by the controller manager when the hardware should become active.
 * It initializes the state and command arrays, sets initial command values based on parameters,
 * creates and starts the `DeviceNode`'s executor thread,
 * and checks if the hardware is ready for communication.
 * @param previous_state The previous lifecycle state (unused).
 * @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::ERROR` otherwise.
 */
hardware_interface::CallbackReturn AllegroHandPlexusHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
  using namespace std::chrono_literals;
  (void)previous_state;
  RCLCPP_INFO(GetLogger(), "Starting[%p] ...please wait...", (void*)this);

  memset(joint_position_command_, 0, sizeof(joint_position_command_));
  memset(joint_velocity_command_, 0, sizeof(joint_velocity_command_));
  memset(joint_torque_command_, 0, sizeof(joint_torque_command_));

  memset(joint_position_state_, 0, sizeof(joint_position_state_));
  memset(joint_velocity_state_, 0, sizeof(joint_velocity_state_));
  memset(joint_torque_state_, 0, sizeof(joint_torque_state_));
  memset(joint_temperature_state_, 0, sizeof(joint_temperature_state_));

  memset(imu_quat_state_, 0, sizeof(imu_quat_state_));
  memset(imu_acc_state_, 0, sizeof(imu_acc_state_));
  memset(imu_gyr_state_, 0, sizeof(imu_gyr_state_));

  auto initial_position_table = GetDeviceNode()->get_parameter_handler()->initial_position_table();
  for(int i=0; i<JOINT_NUM; i++) {
    joint_position_command_[i] = initial_position_table.at(i);
  } 

  auto initial_torque_limit_table = GetDeviceNode()->get_parameter_handler()->torque_limit_table();
  for(int i=0; i<JOINT_NUM; i++) {
    joint_torque_command_[i] = initial_torque_limit_table.at(i);
  } 

  if (!GetDeviceNode()->activate(hwinfo_parser_)) {
    RCLCPP_FATAL(GetLogger(), "Failed to activate device node.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  executor_thread_ = std::thread([this]() { executor_->spin(); });
  std::this_thread::sleep_for(100ms);

  if (GetDevice()->is_ready()) {
    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    return hardware_interface::CallbackReturn::ERROR;
  }
}

/**
 * @brief Deactivates the hardware interface.
 *
 * This method is called by the controller manager to stop the hardware.
 * It stops the executor thread and deactivates the `DeviceNode`, ensuring a graceful shutdown.
 * @param previous_state The previous lifecycle state (unused).
 * @return `CallbackReturn::SUCCESS`.
 */
hardware_interface::CallbackReturn AllegroHandPlexusHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  RCLCPP_INFO(GetLogger(), "Stopping ...please wait...");

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  GetDeviceNode()->deactivate();

  RCLCPP_INFO(GetLogger(), "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Extracts the base interface names from a list of full interface names.
 *
 * This helper function filters a list of fully-qualified interface names (e.g., "joint10/position")
 * to include only those that belong to the joints in `joint_name_set`. It returns a list of the
 * base interface names (e.g., "position").
 * @param joint_name_set A set of joint names to filter against.
 * @param interfaces A vector of fully-qualified interface names.
 * @return A vector of filtered, base interface names.
 */
static std::vector<std::string> extract_interface_names(const std::set<std::string>& joint_name_set,
                                                        const std::vector<std::string>& interfaces) {
  std::vector<std::string> interface_names;
  interface_names.reserve(interfaces.size());

  for (const auto& full_interface_name : interfaces) {
    size_t pos = full_interface_name.find_last_of('/');
    if (pos != std::string::npos) {
      auto joint_name = full_interface_name.substr(0, pos);
      if (joint_name_set.count(joint_name) > 0) {
        interface_names.push_back(full_interface_name.substr(pos + 1));
      }
    }
  }
  return interface_names;
}

/**
 * @brief Prepares for a command mode switch.
 *
 * This method is called by the controller manager before `perform_command_mode_switch`.
 * It validates that the requested switch is valid. This hardware supports two command modes:
 * 1. Position-only: All 16 joints are claimed with the "position" command interface.
 * 2. Position-and-Effort: All 16 joints are claimed with both "position" and "effort" command interfaces.
 * Effort-only mode is not supported.
 * @param stop_interfaces A list of interfaces to be stopped.
 * @return `return_type::OK` if the switch is valid, `return_type::ERROR` otherwise.
 */
hardware_interface::return_type
AllegroHandPlexusHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                            const std::vector<std::string>& stop_interfaces) {
  (void) stop_interfaces;

  if (start_interfaces.empty()) {
    // nothing to do 
    return hardware_interface::return_type::OK;
  }                                                            

  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  const std::set<std::string> expected_joint_name_if(joint_names.begin(), joint_names.end());

  std::vector<std::string> start_interface_names = extract_interface_names(expected_joint_name_if, start_interfaces); 

  std::set<std::string> start_interface_name_set(start_interface_names.begin(), start_interface_names.end());
  std::vector<std::string> start_joint_names;
  for (const auto& full_interface_name : start_interfaces) {
    size_t pos = full_interface_name.find_last_of('/');
    if (pos != std::string::npos) {
      start_joint_names.push_back(full_interface_name.substr(0, pos));
    }
  }

  // Check if all interfaces are either "position" or "position and effort"
  bool all_position = true;
  bool all_effort = false; // effort-only 모드는 지원하지 않으므로 false로 초기화
  bool all_position_effort = false;

  // 모든 조인트가 'position'과 'effort'를 모두 사용하는지 확인
  if (start_interfaces.size() == JOINT_NUM * 2) {
    std::map<std::string, std::set<std::string>> joint_if_map;
    for (const auto& full_if_name : start_interfaces) {
      size_t pos = full_if_name.find_last_of('/');
      if (pos != std::string::npos) {
        joint_if_map[full_if_name.substr(0, pos)].insert(full_if_name.substr(pos + 1));
      }
    }

    if (joint_if_map.size() == JOINT_NUM) {
      all_position_effort = true;
      for (const auto& pair : joint_if_map) {
        if (pair.second.size() != 2 || pair.second.count("position") == 0 || pair.second.count("effort") == 0) {
          all_position_effort = false;
          break;
        }
      }
    }
  }

  if (!all_position_effort) {
    // 'position'만 사용하거나 'effort'만 사용하는 경우 확인
    all_position = (start_interfaces.size() == JOINT_NUM) && (start_interface_name_set.size() == 1 && *start_interface_name_set.begin() == hardware_interface::HW_IF_POSITION);
  }

  if (!(all_position || all_effort || all_position_effort)) {
    RCLCPP_FATAL(GetLogger(), "Unsupported command mode. Allowed modes are 'position' only for all joints, or 'position' and 'effort' for all joints. Effort-only mode is not supported.");
    return hardware_interface::return_type::ERROR;
  } 

  return hardware_interface::return_type::OK;
}

/**
 * @brief Performs the command mode switch.
 *
 * This method is called by the controller manager to actually switch the control mode.
 * It determines the new mode (position-only or position-effort) based on the interfaces being started
 * and calls the SDK's `set_control_mode` method accordingly. It validates that all joints are being
 * switched to a single, consistent command mode.
 * @param stop_interfaces A list of fully-qualified interface names to be stopped.
 * @return `return_type::OK` on success, `return_type::ERROR` otherwise.
 */
hardware_interface::return_type
AllegroHandPlexusHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                            const std::vector<std::string>& stop_interfaces) {
  (void)stop_interfaces;

  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  const std::set<std::string> expected_joint_name_if(joint_names.begin(), joint_names.end());

  std::vector<std::string> start_interface_names = extract_interface_names(expected_joint_name_if, start_interfaces);
  std::vector<std::string> stop_interface_names = extract_interface_names(expected_joint_name_if, stop_interfaces);

  std::set<std::string> start_interface_name_set(start_interface_names.begin(), start_interface_names.end());
  std::set<std::string> stop_interface_name_set(stop_interface_names.begin(), stop_interface_names.end());

  // This hardware interface requires that a controller claims all joints.
  // Partial mode switching is not supported.
  if (!start_interface_names.empty()) {
    bool is_position_only = start_interface_names.size() == JOINT_NUM && start_interface_name_set.size() == 1 && *start_interface_name_set.begin() == "position";
    bool is_position_effort = start_interface_names.size() == JOINT_NUM * 2 && start_interface_name_set.count("position") && start_interface_name_set.count("effort");

    if (!is_position_only && !is_position_effort) {
      RCLCPP_FATAL(GetLogger(), "Cannot switch mode. Invalid combination of start interfaces.");
      return hardware_interface::return_type::ERROR;
    }
  }

  if (!stop_interface_names.empty()) {
    bool is_stopping_all = stop_interface_names.size() == JOINT_NUM || stop_interface_names.size() == JOINT_NUM * 2;
    if (!is_stopping_all) {
      RCLCPP_FATAL(GetLogger(), "Cannot switch mode for a subset of joints. Expected all interfaces to be stopped.");
      return hardware_interface::return_type::ERROR;
    }
  }

  // All joints must be switched to the same command interface type (e.g., all 'position' or all 'effort').
  // Switching to multiple command modes simultaneously is not supported.
  if (std::set<std::string>(stop_interface_names.begin(), stop_interface_names.end()).size() > 1) {
    RCLCPP_FATAL(GetLogger(), "All joints must use the same command interface. Found multiple interface types in stop request.");
    return hardware_interface::return_type::ERROR;
  }

  // Start Control Interface
  std::string start_interface;
  std::string stop_interface;

  if (start_interface_name_set.size() > 0) {
    start_interface = *start_interface_name_set.begin();
  }

  if (stop_interface_name_set.size() > 0) {
    stop_interface = *stop_interface_name_set.begin();
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief Reads the latest states from the hardware.
 *
 * This method is called in each `ros2_control` update cycle. It calls the SDK's
 * `read_states` function to populate the internal state arrays with the latest
 * position, velocity, effort, and temperature data from the hand, as well as IMU data if configured.
 * @param time The current time.
 * @param period The time elapsed since the last update.
 */
hardware_interface::return_type AllegroHandPlexusHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  GetDevice()->read_states(joint_position_state_, joint_velocity_state_, joint_torque_state_, joint_temperature_state_);
  if (info_.sensors.size() > 0) {
    GetDevice()->read_imu_states(imu_acc_state_, imu_gyr_state_);
  }
  return hardware_interface::return_type::OK;
}

/**
 * @brief Writes the latest commands to the hardware.
 *
 * This method is called in each `ros2_control` update cycle. It calls the SDK's 
 * `write_commands` function to send the latest position or effort commands from the controller to the hand.
 * @param time The current time.
 * @param period The time elapsed since the last update.
 */
hardware_interface::return_type AllegroHandPlexusHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  GetDevice()->write_commands(joint_position_command_, joint_velocity_command_, joint_torque_command_);
  return hardware_interface::return_type::OK;
}

} // namespace allegro_hand_plexus_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(allegro_hand_plexus_hardware::AllegroHandPlexusHardwareInterface, hardware_interface::SystemInterface)
