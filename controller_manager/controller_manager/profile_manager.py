#!/usr/bin/env python3
# Copyright 2025 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import errno
import os
import sys
import time
import warnings
import yaml

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller,
    set_controller_parameters,
    set_controller_parameters_from_param_files,
    bcolors,
)
from controller_manager_msgs.msg import ControllerManagerActivity, StringArray, Profile
from controller_manager_msgs.srv import SwitchController, SwitchProfiles, ListProfiles
from controller_manager.controller_manager_services import ServiceNotFoundError

from ament_index_python import get_resources, get_resource

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def first_match(iterable, predicate):
    return next((n for n in iterable if predicate(n)), None)


def combine_name_and_namespace(name_and_namespace):
    node_name, namespace = name_and_namespace
    return namespace + ("" if namespace.endswith("/") else "/") + node_name


def find_node_and_namespace(node, full_node_name):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return first_match(
        node_names_and_namespaces, lambda n: combine_name_and_namespace(n) == full_node_name
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(node_name, node_namespace)
    if not client_names_and_types:
        return False
    client_names, _ = zip(*client_names_and_types)
    return all(service in client_names for service in service_names)


def is_controller_loaded(
    node, controller_manager, controller_name, service_timeout=0.0, call_timeout=10.0
):
    controllers = list_controllers(
        node, controller_manager, service_timeout, call_timeout
    ).controller
    return any(c.name == controller_name for c in controllers)


"""
Profile manager for ros2_control controller manager.

Profiles are a way to group controllers that should be activated/deactivated together and have a semantic meaning together.
    For example, a "base_controller" profile could include all controllers needed to operate a mobile base
    (e.g., wheel PID controllers and a diff drive controller)
    while a "broadcasters" profile could include all the necessary state broadcasters.

Profile manager listens to the controller manager activity topic to monitor active controllers and provides a service to
    switch between profiles by activating/deactivating the corresponding controllers.

Each package can define its own profiles by providing a ros2_control_profile resource using ament_index_register_resource in their CMakeLists.txt.
    Apart from that, the users can define an overriding profile configurations in the $ROS_HOME/ros2_control_profiles/ directory.

The profile manager can be configured to understand the limitations of the system, like mutually exclusive interfaces in joints
    For instance, joint 1 and 2 can only exclusively have position or velocity or torque interfaces active at the same time.
    Maybe, joint 3 can have position and velocity interfaces active at the same time, but not torque.
    These limitations can be defined in a YAML file (non ROS2 parameter file) and passed as a parameter to the profile manager node.

This YAML could like this:

```yaml
    control_modes:
    # default for all joints
    default:
        allowed_mode_sets:
        - [position]
        - [velocity]
        - [torque]
        - [position, velocity]   # globally allowed combo

    # exceptions
    exceptions:
        joint3:
        allowed_mode_sets:
            - [position]           # only position

        joint5:
        allowed_mode_sets:
            - [position, velocity] # must have both

        "arm_*":
        allowed_mode_sets:
            - [position]
            - [position, velocity] # supports both single and combined

```

Based on the above configuration, the profile manager would ensure that when switching profiles
    the controllers being activated do not violate the defined control mode limitations, and when forced to do so,
    it would deactivate the necessary controllers to satisfy the constraints automatically.

"""


class ControllerManagerProfileService(Node):

    def __init__(self):
        super().__init__("profile_manager")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_srv = self.create_service(
            SwitchProfiles,
            "~/switch",
            self.switch_profile_callback,
            callback_group=self.callback_group,
        )
        self.active_profiles_pub = self.create_publisher(
            StringArray,
            "~/active_profiles",
            qos_profile=self.qos_profile,
            callback_group=self.callback_group,
        )

        self.list_profiles_srv = self.create_service(
            ListProfiles,
            "~/list_profiles",
            self.list_profiles_callback,
            callback_group=self.callback_group,
        )
        self.list_available_profiles_srv = self.create_service(
            ListProfiles,
            "~/list_available_profiles",
            self.list_available_profiles_callback,
            callback_group=self.callback_group,
        )
        self.list_active_profiles_srv = self.create_service(
            ListProfiles,
            "~/list_active_profiles",
            self.list_active_profiles_callback,
            callback_group=self.callback_group,
        )

        # Cache of controller states
        self.active_controllers = set()
        self.loaded_controllers = set()

        # Load profiles from config file
        self.load_profiles()

        # Subscribe to controller manager activity and create client to switch controllers
        self.cm_activity = self.create_subscription(
            ControllerManagerActivity,
            "/controller_manager/activity",
            self.activity_callback,
            self.qos_profile,
            callback_group=self.callback_group,
        )
        self.cm_switch_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self.callback_group,
        )

    def activity_callback(self, msg):
        self.active_controllers.clear()
        self.loaded_controllers.clear()
        for controller in msg.controllers:
            self.loaded_controllers.add(controller.name)
            if controller.state.label == "active":
                self.active_controllers.add(controller.name)

        profile_names = []
        for profile_name, profile in self.profiles.items():
            if not profile.controllers_list:
                continue
            if all(c in self.active_controllers for c in profile.controllers_list):
                profile_names.append(profile_name)

        profile_msg = StringArray()
        profile_msg.data = profile_names
        self.active_profiles_pub.publish(profile_msg)

    def load_profiles(self):
        self.profiles = {}
        resources = get_resources("ros2_control_profile")
        if not resources:
            self.get_logger().info("No ros2_control_profile resources found.")

        for resource_name, resource_path in resources.items():
            self.get_logger().info(
                f"Loading ros2_control_profile resource '{resource_name}' from '{resource_path}'"
            )

            try:
                content, _ = get_resource("ros2_control_profile", resource_name)
            except (LookupError, OSError):
                self.get_logger().warning(
                    f"Failed to load ros2_control_profile resource '{resource_name}'"
                )
                continue

            data = None
            # Check if content is a path relative to share
            yaml_path_candidate = os.path.join(
                resource_path, "share", resource_name, content.strip()
            )

            if os.path.exists(yaml_path_candidate):
                try:
                    with open(yaml_path_candidate) as f:
                        data = yaml.safe_load(f)
                    self.get_logger().info(f"Loaded profiles from file: {yaml_path_candidate}")
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to load YAML from file {yaml_path_candidate}: {e}"
                    )
            elif os.path.exists(content.strip()):
                # It's an absolute path
                try:
                    with open(content.strip()) as f:
                        data = yaml.safe_load(f)
                    self.get_logger().info(f"Loaded profiles from file: {content.strip()}")
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to load YAML from file {content.strip()}: {e}"
                    )
            else:
                # Try to load content as YAML directly
                try:
                    data = yaml.safe_load(content)
                    if not isinstance(data, dict):
                        data = None  # Not a profile config if reasonable
                except yaml.YAMLError:
                    pass

            if not data:
                self.get_logger().warning(
                    f"Could not load profiles from resource '{resource_name}'. Content is neither a valid path nor valid profile YAML."
                )
                continue

            for profile_name, info in data.items():
                if profile_name in self.profiles:
                    self.get_logger().warning(
                        f"Profile '{profile_name}' already exists. Skipping duplicate from package '{resource_name}'."
                    )
                    continue

                if "controllers_list" in info:
                    profile = Profile()
                    profile.name = profile_name
                    profile.controllers_list = info["controllers_list"]
                    profile.metadata = info.get("metadata", "")
                    self.profiles[profile_name] = profile
                    self.get_logger().info(f"Loaded profile '{profile_name}' from {resource_name}")
                else:
                    self.get_logger().warning(
                        f"Profile '{profile_name}' from '{resource_name}' missing 'controllers_list'"
                    )

        self.get_logger().info(f"Loaded {len(self.profiles)} profiles")

    def list_profiles_callback(self, request, response):
        response.profiles = list(self.profiles.values())
        return response

    def list_available_profiles_callback(self, request, response):
        available_profiles = []
        for profile in self.profiles.values():
            if all(c in self.loaded_controllers for c in profile.controllers_list):
                available_profiles.append(profile)
        response.profiles = available_profiles
        return response

    def list_active_profiles_callback(self, request, response):
        active_profiles = []
        for profile in self.profiles.values():
            if all(c in self.active_controllers for c in profile.controllers_list):
                active_profiles.append(profile)
        response.profiles = active_profiles
        return response

    def switch_profile_callback(self, request, response):
        cm_request = SwitchController.Request()
        cm_request.activate_controllers = (
            self.profiles[request.start[0]].controllers_list if request.start else []
        )
        cm_request.deactivate_controllers = (
            self.profiles[request.stop[0]].controllers_list if request.stop else []
        )
        cm_request.strictness = SwitchController.Request.STRICT
        cm_request.activate_asap = False
        cm_request.timeout = rclpy.duration.Duration(seconds=1.0).to_msg()
        future = self.cm_switch_client.call_async(cm_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response.ok = future.result().ok
        response.message = future.result().message
        if response.ok:
            self.get_logger().info(
                f"Successfully switched profile. Activated: {cm_request.activate_controllers}, Deactivated: {cm_request.deactivate_controllers}"
            )
        else:
            self.get_logger().error(f"Failed to switch profile. Message: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    profile_manager = ControllerManagerProfileService()
    try:
        rclpy.spin(profile_manager)
    except KeyboardInterrupt:
        pass
    profile_manager.destroy_node()
    rclpy.shutdown()
