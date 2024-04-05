// Copyright 2024 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__LIMITS_HANDLE_HPP
#define HARDWARE_INTERFACE__LIMITS_HANDLE_HPP

#include <cmath>
#include <map>
#include <optional>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace hardware_interface
{
const std::string get_component_interface_name(
  const std::string & prefix_name, const std::string & interface_name)
{
  return prefix_name + "/" + interface_name;
}

#define ROS2_CONTROL_THROW(message)                                                       \
  throw std::runtime_error(                                                               \
    ("File: '" + std::string(__FILE__) + "' Function/method: '" + std::string(__func__) + \
     "' Line: '" + std::to_string(__LINE__) + "' || " + std::string(message) + "\n"))

#define ROS2_CONTROL_ASSERT(condition, message) \
  if (!(condition))                             \
  {                                             \
    ROS2_CONTROL_THROW(message);                \
  }

struct JointInterfaceState
{
public:
  std::optional<double> position = std::nullopt;
  std::optional<double> velocity = std::nullopt;
  std::optional<double> acceleration = std::nullopt;
  std::optional<double> jerk = std::nullopt;
  std::optional<double> effort = std::nullopt;
};

class SaturationLimitHandle
{
public:
  SaturationLimitHandle() {}

  void init(
    std::vector<joint_limits::JointLimits> & limits,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_itf,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf)
  {
    limits_ = limits;
    logging_itf_ = logging_itf;
  }

  virtual bool configure(std::optional<std::vector<JointInterfaceState>> init_state = std::nullopt)
  {
    return true;
  }

  virtual bool enforce(
    const std::vector<JointInterfaceState> & state, std::vector<JointInterfaceState> & command,
    const rclcpp::Duration & dt)
  {
    const auto dt_seconds = dt.seconds();
    if (dt_seconds <= 0.0)
    {
      return false;
    }
    ROS2_CONTROL_ASSERT(
      (state.size() == command.size()), "The parsed state and command do not have the same size!");

    for (size_t index = 0; index < state.size(); index++)
    {
      if (limits_[index].has_position_limits && command[index].position)
      {
        // clamp input pos_cmd
        command[index].position = std::clamp(
          command[index].position.value(), limits_[index].min_position,
          limits_[index].max_position);
      }
    }
    return true;
  }

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double prev_cmd_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_itf_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;
  std::vector<joint_limits::JointLimits> limits_;
  std::unordered_map<std::string, std::reference_wrapper<const StateInterface>> actual_;
  std::unordered_map<std::string, std::reference_wrapper<CommandInterface>> reference_;
};
/*
class SaturationHandle
{
public:
  SaturationHandle(
    const std::string & prefix_name, const std::string & interface_name,
    std::map<std::string, StateInterface> & state_interface_map,
    std::map<std::string, CommandInterface> & command_interface_map)
  : prefix_name_(prefix_name),
    interface_name_(interface_name),
    prev_cmd_(std::numeric_limits<double>::quiet_NaN())
  {
  }

  virtual void enforce_limits(const rclcpp::Duration & period) = 0;

protected:
  std::string prefix_name_;
  std::string interface_name_;
  double prev_cmd_;

  std::unordered_map<std::string, std::reference_wrapper<const StateInterface>> actual_;
  std::unordered_map<std::string, std::reference_wrapper<CommandInterface>> reference_;
};

class PositionSaturationHandle : public SaturationHandle
{
public:
  PositionSaturationHandle(
    const std::string & prefix_name, const joint_limits::JointLimits & limits,
    std::map<std::string, StateInterface> & state_interface_map,
    std::map<std::string, CommandInterface> & command_interface_map)
  : SaturationHandle(prefix_name, HW_IF_POSITION, state_interface_map, command_interface_map),
    limits_(limits)
  {
    prev_cmd_ = 0.0;
    // TODO(saikishor): Check and handle the case when the position state interface might not exist
    actual_.insert(
      {HW_IF_POSITION, std::cref(state_interface_map
                                   .find(get_component_interface_name(prefix_name, HW_IF_POSITION))
                                   ->second)});
    reference_.insert(
      {HW_IF_POSITION, std::ref(command_interface_map
                                  .find(get_component_interface_name(prefix_name, HW_IF_POSITION))
                                  ->second)});
  }

  virtual void enforce_limits(const rclcpp::Duration & period) override
  {
    // Initially, the previous command might be NaN, set it to the current position value
    auto & ref_pos_itf = reference_[HW_IF_POSITION].get();
    if (!std::isfinite(prev_cmd_))
    {
      auto & act_pos_itf = actual_[HW_IF_POSITION].get();
      if (!actual_.empty())
      {
        prev_cmd_ = act_pos_itf.get_value();
      }
      else
      {
        prev_cmd_ = ref_pos_itf.get_value();
      }
    }

    // Check if the joint has joint limits, if not set the limits to maximum
    double min_pos_limit, max_pos_limit;
    if (limits_.has_position_limits)
    {
      min_pos_limit = limits_.min_position;
      max_pos_limit = limits_.max_position;
    }
    else
    {
      min_pos_limit = std::numeric_limits<double>::min();
      max_pos_limit = std::numeric_limits<double>::max();
    }

    // Evalute and clamp the position command to the maximum reachable value by hardware
    double min_pos, max_pos;
    if (limits_.has_velocity_limits)
    {
      const double delta_pos = limits_.max_velocity * period.seconds();
      min_pos = std::max(prev_cmd_ - delta_pos, min_pos_limit);
      max_pos = std::min(prev_cmd_ + delta_pos, max_pos_limit);
    }
    else
    {
      min_pos = min_pos_limit;
      max_pos = max_pos_limit;
    }

    // Saturate position command according to limits
    const double cmd = std::clamp(ref_pos_itf.get_value(), min_pos, max_pos);
    ref_pos_itf.set_value(cmd);
    prev_cmd_ = cmd;
  }

private:
  joint_limits::JointLimits limits_;
};

class VelocitySaturationHandle : public SaturationHandle
{
public:
  VelocitySaturationHandle(
    const std::string & prefix_name, const joint_limits::JointLimits & limits,
    std::map<std::string, StateInterface> & state_interface_map,
    std::map<std::string, CommandInterface> & command_interface_map)
  : SaturationHandle(prefix_name, HW_IF_VELOCITY, state_interface_map, command_interface_map),
    limits_(limits)
  {
    prev_cmd_ = 0.0;
    reference_[HW_IF_VELOCITY] =
      std::ref(command_interface_map[get_component_interface_name(prefix_name, HW_IF_VELOCITY)]);
  }

  virtual void enforce_limits(const rclcpp::Duration & period) override
  {
    if (limits_.has_velocity_limits)
    {
      // Velocity bounds
      double vel_low;
      double vel_high;

      auto & ref_vel_itf = reference_[HW_IF_VELOCITY].get();
      if (!std::isfinite(ref_vel_itf.get_value()))
      {
        ref_vel_itf.set_value(0.0);
      }
      if (!std::isfinite(prev_cmd_))
      {
        prev_cmd_ = 0.0;
      }

      if (limits_.has_acceleration_limits)
      {
        assert(period.seconds() > 0.0);
        const double dt = period.seconds();

        vel_low = std::max(prev_cmd_ - fabs(limits_.max_deceleration) * dt, -limits_.max_velocity);
        vel_high = std::min(prev_cmd_ + limits_.max_acceleration * dt, limits_.max_velocity);
      }
      else
      {
        vel_low = -limits_.max_velocity;
        vel_high = limits_.max_velocity;
      }

      // Saturate velocity command according to limits
      const double vel_cmd = std::clamp(ref_vel_itf.get_value(), vel_low, vel_high);
      ref_vel_itf.set_value(vel_cmd);
      prev_cmd_ = vel_cmd;
    }
  }

private:
  joint_limits::JointLimits limits_;
};

class EffortSaturationHandle : public SaturationHandle
{
public:
  EffortSaturationHandle(
    const std::string & prefix_name, const joint_limits::JointLimits & limits,
    std::map<std::string, StateInterface> & state_interface_map,
    std::map<std::string, CommandInterface> & command_interface_map)
  : SaturationHandle(prefix_name, HW_IF_EFFORT, state_interface_map, command_interface_map),
    limits_(limits)
  {
    prev_cmd_ = 0.0;
    reference_[HW_IF_EFFORT] =
      std::ref(command_interface_map[get_component_interface_name(prefix_name, HW_IF_EFFORT)]);
    // TODO(saikishor): Check and handle the case when the position (or) velocity (or) both state
    // interface might not exist
    actual_[HW_IF_POSITION] =
      std::cref(state_interface_map[get_component_interface_name(prefix_name, HW_IF_POSITION)]);
    actual_[HW_IF_VELOCITY] =
      std::cref(state_interface_map[get_component_interface_name(prefix_name, HW_IF_VELOCITY)]);
  }

  virtual void enforce_limits(const rclcpp::Duration & period) override
  {
    auto & ref_eff_itf = reference_[HW_IF_EFFORT].get();
    auto & act_pos_itf = actual_[HW_IF_POSITION].get();
    auto & act_vel_itf = actual_[HW_IF_VELOCITY].get();
    if (limits_.has_effort_limits)
    {
      double min_eff = -limits_.max_effort;
      double max_eff = limits_.max_effort;

      if (limits_.has_position_limits)
      {
        const double pos = act_pos_itf.get_value();
        if (pos < limits_.min_position)
        {
          min_eff = 0;
        }
        else if (pos > limits_.max_position)
        {
          max_eff = 0;
        }
      }

      if (limits_.has_velocity_limits)
      {
        const double vel = act_vel_itf.get_value();
        if (vel < -limits_.max_velocity)
        {
          min_eff = 0;
        }
        else if (vel > limits_.max_velocity)
        {
          max_eff = 0;
        }
      }
      // Saturate effort command according to limits
      const double eff_cmd = std::clamp(ref_eff_itf.get_value(), min_eff, max_eff);
      ref_eff_itf.set_value(eff_cmd);
    }
  }

private:
  joint_limits::JointLimits limits_;
};

class JointSaturationInterface
{
public:
  JointSaturationInterface(
    const std::string & joint_name, const joint_limits::JointLimits & limits,
    std::map<std::string, StateInterface> & state_interface_map,
    std::map<std::string, CommandInterface> & command_interface_map)
  {
    auto interface_exist =
      [&](const std::string & jnt_name, const std::string & interface_name, const auto & interfaces)
    {
      auto it = interfaces.find(get_component_interface_name(jnt_name, interface_name));
      return it != interfaces.end();
    };
    if (interface_exist(joint_name, HW_IF_POSITION, command_interface_map))
    {
      impl_.push_back(std::make_shared<PositionSaturationHandle>(
        joint_name, limits, state_interface_map, command_interface_map));
    }
    if (interface_exist(joint_name, HW_IF_VELOCITY, command_interface_map))
    {
      impl_.push_back(std::make_shared<VelocitySaturationHandle>(
        joint_name, limits, state_interface_map, command_interface_map));
    }
    if (interface_exist(joint_name, HW_IF_EFFORT, command_interface_map))
    {
      impl_.push_back(std::make_shared<EffortSaturationHandle>(
        joint_name, limits, state_interface_map, command_interface_map));
    }
  }

  void enforce_limits(const rclcpp::Duration & period)
  {
    for (auto & impl : impl_)
    {
      impl->enforce_limits(period);
    }
  }

private:
  std::vector<std::shared_ptr<SaturationHandle>> impl_;
};
*/
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__LIMITS_HANDLE_HPP
