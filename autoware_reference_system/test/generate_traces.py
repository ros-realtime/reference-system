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
import os
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
from launch_testing import post_shutdown_test
import launch_testing.actions
from launch_testing.asserts import assertExitCodes

from tracetools_launch.action import Trace
# from tracetools_trace.tools.names import DEFAULT_CONTEXT
# from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

# Generate traces for specified executables and RMWs

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build


def generate_test_description():
    env = os.environ.copy()
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

    # replaced with cmake `configure_file` function
    rmw_impl = '@RMW_IMPLEMENTATION@'
    test_exe = '@TEST_EXECUTABLE@'
    test_exe_name = '@TEST_EXECUTABLE_NAME@'
    timeout = '@TIMEOUT@'

    # specify rmw to use
    env['RCL_ASSERT_RMW_ID_MATCHES'] = rmw_impl
    env['RMW_IMPLEMENTATION'] = rmw_impl

    launch_description = LaunchDescription()
    proc_under_test = ExecuteProcess(
        cmd=[test_exe],
        name=test_exe_name,
        sigterm_timeout=timeout,
        output='screen',
        env=env,
    )

    trace_action = Trace(
        session_name='profile-' + test_exe_name + '-' + rmw_impl + '-' + str(timeout) + 's'
    )

    launch_description.add_action(trace_action)
    launch_description.add_action(proc_under_test)
    launch_description.add_action(
        launch_testing.actions.ReadyToTest()
    )
    return launch_description, locals()


class TestAutowareReferenceSystem(unittest.TestCase):

    def test_pubs_and_subs(self):
        print('test_pubs_and_subs')


@post_shutdown_test()
class TestAutowareReferenceSystemAfterShutdown(unittest.TestCase):

    def test_process_exit_codes(self):
        # Checks that all processes exited cleanly
        assertExitCodes(self.proc_info)
