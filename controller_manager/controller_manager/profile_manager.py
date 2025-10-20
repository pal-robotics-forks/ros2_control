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
from controller_manager_msgs.msg import ControllerManagerActivity, StringArray
from controller_manager_msgs.srv import SwitchController, SwitchProfiles
from controller_manager.controller_manager_services import ServiceNotFoundError

from filelock import Timeout, FileLock
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
        node_names_and_namespaces, lambda n: combine_name_and_namespace(
            n) == full_node_name
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(
        node_name, node_namespace)
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


class ControllerManagerProfileService(Node):

    def __init__(self):
        super().__init__('profile_manager')
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.start_srv = self.create_service(
            SwitchProfiles, '~/switch', self.switch_profile_callback, callback_group=self.callback_group)
        self.active_profiles_pub = self.create_publisher(
            StringArray, '~/active_profiles', qos_profile=self.qos_profile, callback_group=self.callback_group)

        # Load profiles from config file
        self.load_profiles()

        # Subscribe to controller manager activity and create client to switch controllers
        self.cm_activity = self.create_subscription(
            ControllerManagerActivity,
            '/controller_manager/activity',
            self.activity_callback,
            self.qos_profile,
            callback_group=self.callback_group)
        self.cm_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller', callback_group=self.callback_group)

    def activity_callback(self, msg):
        print("Current active controllers:")
        active_controllers = []
        for controllers in msg.controllers:
            print(f" - {controllers.name}: {controllers.state.label}")
            if controllers.state.label == "active":
                active_controllers.append(controllers.name)
        profile_names = []
        for profile_name, controllers in self.profiles.items():
            if all(controller in active_controllers for controller in controllers):
                profile_names.append(profile_name)

        profile_msg = StringArray()
        profile_msg.data = profile_names
        self.active_profiles_pub.publish(profile_msg)

    def load_profiles(self):
        self.profiles = {}
        self.profiles["broadcasters"] = ["joint_state_broadcaster"]
        self.profiles["base_controller"] = ["pid_controller_right_wheel_joint",
                                            "pid_controller_left_wheel_joint",
                                            "diffbot_base_controller"]
        print("Loaded profiles:")

    def switch_profile_callback(self, request, response):
        cm_request = SwitchController.Request()
        cm_request.activate_controllers = self.profiles[request.start[0]] if request.start else [
        ]
        cm_request.deactivate_controllers = self.profiles[request.stop[0]] if request.stop else [
        ]
        cm_request.strictness = SwitchController.Request.STRICT
        cm_request.activate_asap = False
        cm_request.timeout = rclpy.duration.Duration(seconds=1.0).to_msg()
        future = self.cm_switch_client.call_async(cm_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response.ok = future.result().ok
        response.message = future.result().message
        if response.ok:
            self.get_logger().info(
                f"Successfully switched profile. Activated: {cm_request.activate_controllers}, Deactivated: {cm_request.deactivate_controllers}")
        else:
            self.get_logger().error(
                f"Failed to switch profile. Message: {response.message}")
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
