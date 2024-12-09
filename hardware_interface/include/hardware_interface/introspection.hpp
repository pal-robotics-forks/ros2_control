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

/// \author Sai Kishor Kothakota

#ifndef HARDWARE_INTERFACE__INTROSPECTION_HPP_
#define HARDWARE_INTERFACE__INTROSPECTION_HPP_

#include <pal_statistics/pal_statistics_macros.hpp>

namespace hardware_interface
{
constexpr char DEFAULT_REGISTRY_KEY[] = "ros2_control";
constexpr char DEFAULT_INTROSPECTION_TOPIC[] = "~/introspection_data";

#define REGISTER_ROS2_CONTROL_INTROSPECTION_2_ARGS(ID, ENTITY)               \
  REGISTER_ENTITY(                                                           \
    hardware_interface::DEFAULT_REGISTRY_KEY, get_name() + "." + ID, ENTITY, \
    &stats_registrations_, false)

#define REGISTER_ROS2_CONTROL_INTROSPECTION_3_ARGS(ID, ENTITY, ENABLE)       \
  REGISTER_ENTITY(                                                           \
    hardware_interface::DEFAULT_REGISTRY_KEY, get_name() + "." + ID, ENTITY, \
    &stats_registrations_, ENABLE)

#ifndef GET_4TH_ARG
#define GET_4TH_ARG(arg1, arg2, arg3, arg4, ...) arg4
#endif

#define REGISTER_ROS2_CONTROL_INTROSPECTION_MACRO_CHOOSER(...) \
  GET_4TH_ARG(                                                 \
    __VA_ARGS__, REGISTER_ROS2_CONTROL_INTROSPECTION_3_ARGS,   \
    REGISTER_ROS2_CONTROL_INTROSPECTION_2_ARGS)

#define REGISTER_ROS2_CONTROL_INTROSPECTION(...) \
  REGISTER_ROS2_CONTROL_INTROSPECTION_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__INTROSPECTION_HPP_
