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

# Generate traces for specified executables and RMWs

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build


def generate_test_description():
    env = os.environ.copy()
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
    # specify rmw to use
    env['RCL_ASSERT_RMW_ID_MATCHES'] = '@RMW_IMPLEMENTATION@'
    env['RMW_IMPLEMENTATION'] = '@RMW_IMPLEMENTATION@'

    launch_description = LaunchDescription()
    proc_under_test = ExecuteProcess(
        cmd=['@TEST_EXECUTABLE@'],
        name='@TEST_EXECUTABLE_NAME@',
        sigterm_timeout='@TIMEOUT@',
        output='screen',
        env=env,
    )
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
