# Copyright 2021 Apex.AI, Inc.
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
import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_CONTEXT
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# Generate traces for specified executables and RMWs

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build
RUNTIME = int('@RUNTIME@')


def generate_test_description():
    # replaced with cmake `configure_file` function
    rmw_impl = '@RMW_IMPLEMENTATION@'
    test_exe = '@TEST_EXECUTABLE@'
    test_exe_name = '@TEST_EXECUTABLE_NAME@'

    launch_description = LaunchDescription()

    # see https://github.com/ros2/launch/issues/417
    # have to use `SetEnvironmentVariable` to set env vars in launch description
    envvar_rcutils_action = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}')
    envvar_rmw_action = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', rmw_impl)
    envvar_rclassert_rmw_action = SetEnvironmentVariable(
        'RCL_ASSERT_RMW_ID_MATCHES', rmw_impl)

    node_under_test = Node(
        package='autoware_reference_system',
        executable=test_exe,
        output='screen'
    )

    trace_action = Trace(
        session_name='profile_callback_' + test_exe_name + '_' + str(RUNTIME) + 's',
        events_ust=[
            'lttng_ust_cyg_profile_fast:func_entry',
            'lttng_ust_cyg_profile_fast:func_exit',
            'lttng_ust_statedump:start',
            'lttng_ust_statedump:end',
            'lttng_ust_statedump:bin_info',
            'lttng_ust_statedump:build_id'
        ] + DEFAULT_EVENTS_ROS,
        events_kernel=[
            'sched_switch'
        ],
        context_names=[
            'ip'
        ] + DEFAULT_CONTEXT,
    )

    launch_description.add_action(envvar_rcutils_action)
    launch_description.add_action(envvar_rclassert_rmw_action)
    launch_description.add_action(envvar_rmw_action)
    launch_description.add_action(trace_action)
    launch_description.add_action(node_under_test)
    launch_description.add_action(
        launch_testing.actions.ReadyToTest()
    )
    return launch_description, locals()


class TestGenerateTracesAutowareReferenceSystem(unittest.TestCase):

    def test_generate_traces(self):
        global RUNTIME
        start_time = time.time()
        end_time = start_time + RUNTIME

        while time.time() < end_time:
            print('generating traces...')
            time.sleep(0.25)  # seconds

        self.assertTrue(True)
