// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <joystick_controller/joystick_controller.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joystick_controller
{

// Constants for better maintainability
namespace constants
{
constexpr size_t TACT_SWITCH_INTERFACE_INDEX = 2;
constexpr double TACT_SWITCH_THRESHOLD = 0.5;
constexpr double DEFAULT_JOG_SCALE = 0.1;

  // cmd_vel scaling factors
constexpr double LINEAR_X_SCALE = 3.0;
constexpr double LINEAR_Y_SCALE = 3.0;
constexpr double ANGULAR_Z_SCALE = 2.0;

  // Sensor names
const char LEFT_JOYSTICK_NAME[] = "sensorxel_l_joy";
const char RIGHT_JOYSTICK_NAME[] = "sensorxel_r_joy";

  // Mode names
const char SWERVE_MODE[] = "swerve";
const char ARM_CONTROL_MODE[] = "arm_control";
}  // namespace constants

JoystickController::JoystickController()
: controller_interface::ControllerInterface()
{
}

// Helper methods for better code organization
double JoystickController::normalize_joystick_value(double raw_adc, bool is_tact_switch) const
{
  if (is_tact_switch) {
    return raw_adc;
  }

  double normalized_value;
  if (raw_adc < params_.joystick_calibration_center) {
    normalized_value = -(params_.joystick_calibration_center - raw_adc) /
      (params_.joystick_calibration_center - params_.joystick_calibration_min);
  } else {
    normalized_value = (raw_adc - params_.joystick_calibration_center) /
      (params_.joystick_calibration_max - params_.joystick_calibration_center);
  }

  // Apply deadzone
  if (std::abs(normalized_value) < params_.deadzone) {
    return 0.0;
  }

  // Normalize after deadzone
  if (normalized_value > 0) {
    normalized_value = (normalized_value - params_.deadzone) / (1.0 - params_.deadzone);
  } else {
    normalized_value = (normalized_value + params_.deadzone) / (1.0 - params_.deadzone);
  }

  return normalized_value;
}

std::vector<double> JoystickController::read_and_normalize_sensor_values(size_t sensor_idx) const
{
  std::vector<double> normalized_values(state_interface_types_.size(), 0.0);

  for (size_t j = 0; j < state_interface_types_.size(); ++j) {
    if (j >= joint_state_interface_.size() || sensor_idx >= joint_state_interface_[j].size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid interface access: j=%zu, i=%zu", j,
          sensor_idx);
      continue;
    }

    auto opt_value = joint_state_interface_[j][sensor_idx].get().get_optional();
    if (!opt_value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No value for state interface [%zu][%zu]", j,
          sensor_idx);
      continue;
    }

    double raw_adc = opt_value.value();
    bool is_tact_switch = (j == constants::TACT_SWITCH_INTERFACE_INDEX);
    double normalized_value = normalize_joystick_value(raw_adc, is_tact_switch);

    // Apply reverse if needed
    const auto & interface_name = state_interface_types_[j];
    const auto & reverse_interfaces =
      sensor_reverse_interfaces_.at(sensorxel_joy_names_[sensor_idx]);
    if (std::find(reverse_interfaces.begin(), reverse_interfaces.end(),
        interface_name) != reverse_interfaces.end())
    {
      normalized_value = -normalized_value;
    }

    normalized_values[j] = normalized_value;
  }

  return normalized_values;
}

void JoystickController::update_joystick_values(
  const std::string & sensor_name,
  const std::vector<double> & normalized_values,
  JoystickValues & joystick_values,
  bool & left_tact_pressed,
  bool & right_tact_pressed) const
{
  if (sensor_name == constants::LEFT_JOYSTICK_NAME) {
    joystick_values.left_x = normalized_values[0];
    joystick_values.left_y = normalized_values[1];
    if (normalized_values.size() > constants::TACT_SWITCH_INTERFACE_INDEX) {
      left_tact_pressed = (normalized_values[constants::TACT_SWITCH_INTERFACE_INDEX] >
        constants::TACT_SWITCH_THRESHOLD);
    }
  } else if (sensor_name == constants::RIGHT_JOYSTICK_NAME) {
    joystick_values.right_x = normalized_values[0];
    joystick_values.right_y = normalized_values[1];
    if (normalized_values.size() > constants::TACT_SWITCH_INTERFACE_INDEX) {
      right_tact_pressed = (normalized_values[constants::TACT_SWITCH_INTERFACE_INDEX] >
        constants::TACT_SWITCH_THRESHOLD);
    }
  }
}

void JoystickController::update_last_active_positions(
  const std::vector<std::string> & controlled_joints)
{
  // This method should be called with the correct sensor context
  // For now, we'll use the first sensor as default
  if (sensorxel_joy_names_.empty()) {
    return;
  }

  const std::string & sensor_name = sensorxel_joy_names_[0];
  auto & last_active_positions = sensor_last_active_positions_[sensor_name];

  for (size_t i = 0; i < controlled_joints.size(); ++i) {
    const auto & joint_name = controlled_joints[i];
    auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
    if (it != current_joint_states_.name.end()) {
      size_t index = std::distance(current_joint_states_.name.begin(), it);
      if (i < last_active_positions.size()) {
        last_active_positions[i] = current_joint_states_.position[index];
      }
    }
  }
}

std::vector<double> JoystickController::calculate_joint_positions(
  const std::vector<std::string> & controlled_joints,
  const std::vector<double> & normalized_values,
  const std::string & sensor_name,
  bool swerve_mode,
  const JoystickValues & joystick_values) const
{
  std::vector<double> positions;

  for (size_t i = 0; i < controlled_joints.size(); ++i) {
    const auto & joint_name = controlled_joints[i];
    auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
    if (it != current_joint_states_.name.end()) {
      size_t index = std::distance(current_joint_states_.name.begin(), it);
      double current_position = current_joint_states_.position[index];
      double sensorxel_joy_value;

      if (swerve_mode) {
        // Use right joystick X-axis for lift control in swerve mode
        sensorxel_joy_value = (sensor_name ==
          constants::RIGHT_JOYSTICK_NAME) ? joystick_values.right_x : 0.0;
      } else {
        // Normal mode: use appropriate axis based on joint index
        sensorxel_joy_value = (state_interface_types_.size() > 1 &&
          i == 1) ? normalized_values[1] : normalized_values[0];
      }

      double new_position = current_position + sensorxel_joy_value *
        sensor_jog_scale_.at(sensor_name);
      positions.push_back(new_position);
    }
  }

  return positions;
}

void JoystickController::publish_joint_trajectory(
  const std::vector<std::string> & controlled_joints,
  const std::vector<double> & positions,
  const std::string & sensor_name)
{
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  trajectory_msg.header.stamp = rclcpp::Time(0);
  trajectory_msg.joint_names = controlled_joints;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration(0, 0);
  point.positions.reserve(positions.size());
  const auto min_it = sensor_min_positions_.find(sensor_name);
  const auto max_it = sensor_max_positions_.find(sensor_name);
  for (size_t i = 0; i < positions.size(); ++i) {
    double position = positions[i];
    if (min_it != sensor_min_positions_.end() && max_it != sensor_max_positions_.end() &&
      i < min_it->second.size() && i < max_it->second.size())
    {
      position = std::clamp(position, min_it->second[i], max_it->second[i]);
    }
    point.positions.push_back(position);
  }
  point.velocities.resize(positions.size(), 0.0);
  point.accelerations.resize(positions.size(), 0.0);

  trajectory_msg.points.push_back(point);

  auto joint_trajectory_publisher = sensor_joint_trajectory_publisher_[sensor_name];
  if (joint_trajectory_publisher) {
    joint_trajectory_publisher->publish(trajectory_msg);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
        "Joint trajectory publisher not found for sensor: %s", sensor_name.c_str());
  }
}

void JoystickController::publish_cmd_vel(bool swerve_mode, const JoystickValues & joystick_values)
{
  geometry_msgs::msg::Twist twist_msg;

  if (swerve_mode) {
    twist_msg.linear.x = -joystick_values.left_x / constants::LINEAR_X_SCALE;
    twist_msg.linear.y = joystick_values.left_y / constants::LINEAR_Y_SCALE;
    twist_msg.angular.z = -joystick_values.right_y / constants::ANGULAR_Z_SCALE;
  } else {
    // Zero twist when not in swerve mode
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(twist_msg);
}

void JoystickController::publish_joystick_values()
{
  auto sensorxel_joy_msg = std_msgs::msg::Float64MultiArray();
  for (const auto & sensorxel_joy_value : sensorxel_joy_values_) {
    sensorxel_joy_msg.data.insert(
      sensorxel_joy_msg.data.end(),
      sensorxel_joy_value.begin(),
      sensorxel_joy_value.end());
  }

  if (sensorxel_joy_publisher_.count("common") > 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Publishing joystick values to common topic");
    sensorxel_joy_publisher_["common"]->publish(sensorxel_joy_msg);
  }
}

void JoystickController::handle_tact_switches(
  bool left_tact_pressed, bool right_tact_pressed, const rclcpp::Time & current_time)
{
  // Create current state as bit pattern: left=bit1, right=bit0
  // 00 = neither pressed, 01 = right only, 10 = left only, 11 = both pressed
  uint8_t current_state = (left_tact_pressed ? 2 : 0) | (right_tact_pressed ? 1 : 0);
  uint8_t prev_state = (prev_left_tact_switch_ ? 2 : 0) | (prev_right_tact_switch_ ? 1 : 0);

  // Handle left tact switch press start
  if (left_tact_pressed && !prev_left_tact_switch_) {
    left_tact_press_start_time_ = current_time;
    left_tact_long_press_triggered_ = false;
  }

  // Handle right tact switch press start
  if (right_tact_pressed && !prev_right_tact_switch_) {
    right_tact_press_start_time_ = current_time;
    right_tact_long_press_triggered_ = false;
  }

  // Check for long press on left tact switch
  if (left_tact_pressed && !left_tact_long_press_triggered_) {
    auto press_duration = current_time - left_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "left_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Left tact switch long press triggered!");
      left_tact_long_press_triggered_ = true;
    }
  }

  // Check for long press on right tact switch
  if (right_tact_pressed && !right_tact_long_press_triggered_) {
    auto press_duration = current_time - right_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "right_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Right tact switch long press triggered!");
      right_tact_long_press_triggered_ = true;
    }
  }

  // Set flag when both buttons are pressed
  if (current_state == 3) {
    both_pressed_flag_ = true;
  }

  // Only trigger actions when reaching 00 state (no buttons pressed)
  if (current_state == 0 && prev_state != 0) {
    if (both_pressed_flag_) {
      // Mode change - both buttons were pressed at some point
      std_msgs::msg::String mode_msg;
      if (!params_.enable_swerve_mode) {
        current_mode_ = constants::ARM_CONTROL_MODE;
        RCLCPP_INFO(get_node()->get_logger(), "Swerve mode is disabled by configuration.");
      } else if (current_mode_ == constants::ARM_CONTROL_MODE) {
        current_mode_ = constants::SWERVE_MODE;
      } else {
        current_mode_ = constants::ARM_CONTROL_MODE;
      }
      mode_msg.data = current_mode_;
      mode_pub_->publish(mode_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Mode switched to: %s", current_mode_.c_str());

      // Reset flag after mode change
      both_pressed_flag_ = false;
    } else {
      // Individual button trigger - only if both were never pressed and not long press
      switch (prev_state) {
        case 1:  // 01 -> 00 (right button only was pressed)
          if (!right_tact_long_press_triggered_) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "right";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Right tact switch triggered!");
          }
          break;

        case 2:  // 10 -> 00 (left button only was pressed)
          if (!left_tact_long_press_triggered_) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "left";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Left tact switch triggered!");
          }
          break;
      }
    }

    // Reset long press flags when buttons are released
    if (prev_state == 1 || prev_state == 3) {  // Right button was pressed
      right_tact_long_press_triggered_ = false;
    }
    if (prev_state == 2 || prev_state == 3) {  // Left button was pressed
      left_tact_long_press_triggered_ = false;
    }
  }

  // Update previous state
  prev_left_tact_switch_ = left_tact_pressed;
  prev_right_tact_switch_ = right_tact_pressed;
  prev_tact_switch_ = (current_state == 3);
}

controller_interface::InterfaceConfiguration
JoystickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
JoystickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & sensorxel_joy_name : sensorxel_joy_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(sensorxel_joy_name + "/" + interface_type);
    }
  }

  return config;
}

void JoystickController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store current joint states
  current_joint_states_ = *msg;

  // initialize last_active_positions_ by sensor
  if (!has_joint_states_) {
    bool all_controlled_joints_found = true;
    for (const auto & sensor_name : sensorxel_joy_names_) {
      const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
      auto & last_active_positions = sensor_last_active_positions_[sensor_name];
      last_active_positions.resize(controlled_joints.size());
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
            joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          last_active_positions[i] = current_joint_states_.position[index];
        } else {
          all_controlled_joints_found = false;
        }
      }
    }
    if (!all_controlled_joints_found) {
      return;
    }
  }

  has_joint_states_ = true;
}

void JoystickController::base_motion_lock_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_locked = base_motion_locked_.exchange(msg->data);
  if (msg->data) {
    base_unlock_requires_center_.store(true);
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    if (!was_locked) {
      RCLCPP_INFO(get_node()->get_logger(), "Base motion locked for data recording.");
    }
  } else if (was_locked) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Recording ended. Center both joysticks to unlock base motion.");
  }
}

controller_interface::return_type JoystickController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!has_joint_states_) {
    return controller_interface::return_type::OK;
  }

  bool swerve_mode = params_.enable_swerve_mode &&
    (current_mode_ == constants::SWERVE_MODE);
  JoystickValues joystick_values;
  bool left_tact_switch_pressed = false;
  bool right_tact_switch_pressed = false;

  // Process each sensor
  for (size_t sensor_idx = 0; sensor_idx < sensorxel_joy_names_.size(); ++sensor_idx) {
    const auto & sensor_name = sensorxel_joy_names_[sensor_idx];
    RCLCPP_DEBUG(get_node()->get_logger(), "Processing sensor: %s", sensor_name.c_str());

    const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
    auto & last_active_positions = sensor_last_active_positions_[sensor_name];

    // Read and normalize sensor values
    std::vector<double> normalized_values = read_and_normalize_sensor_values(sensor_idx);
    if (!params_.enable_joystick_axes) {
      if (!normalized_values.empty()) {
        normalized_values[0] = 0.0;
      }
      if (normalized_values.size() > 1) {
        normalized_values[1] = 0.0;
      }
    }

    // Update joystick values
    update_joystick_values(sensor_name, normalized_values, joystick_values,
                          left_tact_switch_pressed, right_tact_switch_pressed);

    // Only axes mapped to this sensor's joints count as a jog command. In
    // swerve mode, the right X-axis exclusively controls the lift. Keeping
    // swerve mode active must not continuously replace the held target with
    // the gravity-displaced feedback position.
    bool joint_jog_active = false;
    if (swerve_mode) {
      joint_jog_active = sensor_name == constants::RIGHT_JOYSTICK_NAME &&
        std::abs(joystick_values.right_x) > 0.0;
    } else if (!controlled_joints.empty() && !normalized_values.empty()) {
      joint_jog_active = std::abs(normalized_values[0]) > 0.0;
      if (controlled_joints.size() > 1 && normalized_values.size() > 1) {
        joint_jog_active = joint_jog_active || std::abs(normalized_values[1]) > 0.0;
      }
    }

    auto & was_joint_jog_active = sensor_was_joint_jog_active_[sensor_name];

    // Capture the achieved pose once when the operator releases the axis.
    // Subsequent feedback movement must not move this hold target.
    if (was_joint_jog_active && !joint_jog_active && !current_joint_states_.name.empty() &&
      !controlled_joints.empty())
    {
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
            joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          if (i < last_active_positions.size()) {
            last_active_positions[i] = current_joint_states_.position[index];
          }
        }
      }
    }

    // Publish joint trajectory
    if (!current_joint_states_.name.empty() && !controlled_joints.empty()) {
      std::vector<double> positions;

      if (joint_jog_active) {
        positions = calculate_joint_positions(controlled_joints, normalized_values,
                                           sensor_name, swerve_mode, joystick_values);
        // Update last active positions with new positions
        for (size_t i = 0; i < positions.size() && i < last_active_positions.size(); ++i) {
          last_active_positions[i] = positions[i];
        }
      } else {
        positions = last_active_positions;
      }

      publish_joint_trajectory(controlled_joints, positions, sensor_name);
    }

    was_joint_jog_active = joint_jog_active;
    sensorxel_joy_values_[sensor_idx] = normalized_values;
  }

  // Publish cmd_vel
  const bool joysticks_centered =
    joystick_values.left_x == 0.0 && joystick_values.left_y == 0.0 &&
    joystick_values.right_x == 0.0 && joystick_values.right_y == 0.0;
  if (!base_motion_locked_.load() && base_unlock_requires_center_.load() && joysticks_centered) {
    base_unlock_requires_center_.store(false);
    RCLCPP_INFO(get_node()->get_logger(), "Base motion unlocked.");
  }
  const bool base_motion_enabled =
    !base_motion_locked_.load() && !base_unlock_requires_center_.load();
  publish_cmd_vel(swerve_mode && base_motion_enabled, joystick_values);

  // Publish joystick values
  publish_joystick_values();

  // Handle all tact switch functionality (mode switching and individual triggers)
  handle_tact_switches(left_tact_switch_pressed, right_tact_switch_pressed, time);

  RCLCPP_DEBUG(get_node()->get_logger(), "Joystick controller update completed");

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JoystickController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // Get sensorxel_joy sensor names from parameters
  sensorxel_joy_names_ = params_.joystick_sensors;
  n_sensorxel_joys_ = sensorxel_joy_names_.size();
  sensor_was_joint_jog_active_.clear();

  if (sensorxel_joy_names_.empty()) {
    RCLCPP_WARN(logger, "'joystick_sensors' parameter is empty.");
  }

  // Initialize the sensorxel_joy values vector with the correct size
  sensorxel_joy_values_.resize(n_sensorxel_joys_);
  for (auto & vec : sensorxel_joy_values_) {
    vec.resize(state_interface_types_.size(), 0.0);
  }
  // read parameters by sensor name
  for (const auto & sensor_name : sensorxel_joy_names_) {
    // controlled_joints
    std::string joints_param = sensor_name + "_controlled_joints";
    if (get_node()->has_parameter(joints_param)) {
      sensor_controlled_joints_[sensor_name] =
        get_node()->get_parameter(joints_param).as_string_array();
    }
    const std::string min_positions_param = sensor_name + "_min_positions";
    const std::string max_positions_param = sensor_name + "_max_positions";
    sensor_min_positions_[sensor_name] =
      get_node()->get_parameter(min_positions_param).as_double_array();
    sensor_max_positions_[sensor_name] =
      get_node()->get_parameter(max_positions_param).as_double_array();
    const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
    const auto & min_positions = sensor_min_positions_[sensor_name];
    const auto & max_positions = sensor_max_positions_[sensor_name];
    if (min_positions.empty() != max_positions.empty() ||
      (!min_positions.empty() &&
      (min_positions.size() != controlled_joints.size() ||
      max_positions.size() != controlled_joints.size())))
    {
      RCLCPP_ERROR(
        logger, "Position limits for sensor '%s' must both contain exactly %zu values.",
        sensor_name.c_str(), controlled_joints.size());
      return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < min_positions.size(); ++i) {
      if (min_positions[i] > max_positions[i]) {
        RCLCPP_ERROR(
          logger, "Minimum position exceeds maximum for sensor '%s', joint '%s'.",
          sensor_name.c_str(), controlled_joints[i].c_str());
        return CallbackReturn::ERROR;
      }
    }
    // reverse_interfaces
    std::string reverse_param = sensor_name + "_reverse_interfaces";
    if (get_node()->has_parameter(reverse_param)) {
      sensor_reverse_interfaces_[sensor_name] =
        get_node()->get_parameter(reverse_param).as_string_array();
    } else {
      // If parameter does not exist, initialize as empty vector
      sensor_reverse_interfaces_[sensor_name] = std::vector<std::string>();
    }
    // joint_trajectory_topic
    std::string topic_param = sensor_name + "_joint_trajectory_topic";
    if (get_node()->has_parameter(topic_param)) {
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s, value: %s", topic_param.c_str(),
          get_node()->get_parameter(topic_param).as_string().c_str());
      sensor_joint_trajectory_topic_[sensor_name] =
        get_node()->get_parameter(topic_param).as_string();
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s not found", topic_param.c_str());
    }
    // jog_scale
    std::string jog_scale_param = sensor_name + "_jog_scale";
    if (get_node()->has_parameter(jog_scale_param)) {
      sensor_jog_scale_[sensor_name] = get_node()->get_parameter(jog_scale_param).as_double();
    } else {
      // fallback: default jog scale
      sensor_jog_scale_[sensor_name] = constants::DEFAULT_JOG_SCALE;
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s not found, using default %.1f",
          jog_scale_param.c_str(), constants::DEFAULT_JOG_SCALE);
    }
  }

  // Create publisher for sensorxel_joy values (common topic)
  sensorxel_joy_publisher_["common"] =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/sensorxel_joy_values", rclcpp::SystemDefaultsQoS());

  // Create publisher for joint trajectory
  for (const auto & sensor_name : sensorxel_joy_names_) {
    RCLCPP_WARN(get_node()->get_logger(),
        "Creating joint trajectory publisher for sensor: %s, topic: %s", sensor_name.c_str(),
        sensor_joint_trajectory_topic_[sensor_name].c_str());
    sensor_joint_trajectory_publisher_[sensor_name] =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      sensor_joint_trajectory_topic_[sensor_name], rclcpp::SystemDefaultsQoS());
  }

  // Create subscriber for joint states
  RCLCPP_WARN(get_node()->get_logger(), "Creating joint states subscriber for topic: %s",
      params_.joint_states_topic.c_str());
  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));

  // Create publisher for mode
  mode_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/leader/joystick_controller_right/joystick_mode", 10);
  prev_tact_switch_ = false;

  // Create publisher for right tact switch trigger
  tact_trigger_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/leader/joystick_controller/tact_trigger", 10);
  prev_right_tact_switch_ = false;
  prev_left_tact_switch_ = false;
  both_pressed_flag_ = false;

  // Initialize long press variables
  left_tact_long_press_triggered_ = false;
  right_tact_long_press_triggered_ = false;
  left_tact_press_start_time_ = rclcpp::Time(0);
  right_tact_press_start_time_ = rclcpp::Time(0);

  // Create publisher for cmd_vel
  cmd_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  base_motion_lock_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/data/recording/base_motion_locked", 10,
    std::bind(
      &JoystickController::base_motion_lock_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  has_joint_states_ = false;
  current_mode_ = params_.enable_swerve_mode ?
    constants::SWERVE_MODE : constants::ARM_CONTROL_MODE;
  for (const auto & sensor_name : sensorxel_joy_names_) {
    sensor_was_joint_jog_active_[sensor_name] = false;
  }

  // Initialize state interface vector
  joint_state_interface_.resize(state_interface_types_.size());

  // Order all sensorxel_joy sensors in the storage
  for (size_t i = 0; i < state_interface_types_.size(); ++i) {
    const auto & interface = state_interface_types_[i];
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, sensorxel_joy_names_, interface, ordered_interfaces))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        n_sensorxel_joys_, interface.c_str(), ordered_interfaces.size());
      return CallbackReturn::ERROR;
    }
    joint_state_interface_[i] = ordered_interfaces;
  }

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "JoystickController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joystick_controller::JoystickController, controller_interface::ControllerInterface)
