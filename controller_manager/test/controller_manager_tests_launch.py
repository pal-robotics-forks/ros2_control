#!/usr/bin/env python3

# Copyright 2020 ROS2-Control Development Team (2020)
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

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node


def generate_test_description():
    test_executable = os.getenv('TEST_EXECUTABLE')

    robot_description_file = os.path.join(
        get_package_share_directory('test_robot_hardware'),
        'description',
        'rrbot_system_position_only.urdf'
        )
    with open(robot_description_file, 'r') as infile:
        descr = infile.read()

    params = {'robot_description': descr,
              'update_time_ms': 10} #ms

    test_node = Node(
        executable=test_executable,
        parameters=[params],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    return (
        LaunchDescription([
            test_node,
            # Start tests right away - no need to wait for anything
            ReadyToTest(),
        ]))

if __name__ == '__main__':
    sys.exit(main())
