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

import multiprocessing
import os
import platform
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
from launch_testing import post_shutdown_test
import launch_testing.actions
from launch_testing.asserts import assertExitCodes

import rclpy
from rclpy.context import Context
import rclpy.executors
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import reference_interfaces.msg

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build

platforms = {}

# TODO(flynneva): move this to its own file for portability
# can add more supported platforms here
platforms['rpi4-linux-rt'] = {
    'common-name': 'raspberrypi4',
    'machine': 'aarch64',
    'processor': 'aarch64',
    'system': 'Linux',
    'flavor': 'ubuntu',
    'cores': 4,
    'realtime': True
}


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


class TestReferenceSystemAutoware(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_node', context=cls.context)
        cls.msgs = []

        # TODO(flynneva): sweep over different QoS settings during testing?
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        cls.sub = cls.node.create_subscription(
            msg_type=reference_interfaces.msg.Message4kb,
            topic='/FrontLidarDriver',
            callback=cls._msg_received,
            qos_profile=qos_profile
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.sub)
        rclpy.shutdown(context=cls.context)

    @classmethod
    def _msg_received(self, msg):
        # Callback for ROS subscriber used in the test
        self.msgs.append(msg)

    def get_message(self):
        # Try up to 60sec to receive messages
        startlen = len(self.msgs)

        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)

        try:
            end_time = time.time() + 60.0
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                if startlen != len(self.msgs):
                    break

            self.assertNotEqual(startlen, len(self.msgs))
            return self.msgs[-1]
        finally:
            executor.remove_node(self.node)

    def test_msg_rate(self):
        # Receive messages for 5 seconds - make sure we don't get too many or too few
        RUNTIME = 5.0
        start_time = time.time()
        end_time = start_time + RUNTIME

        while time.time() < end_time:
            self.get_message()

        self.assertGreater(len(self.msgs), 0)
        self.assertGreater(len(self.msgs), RUNTIME)  # At least 1 message per second
        self.assertLess(len(self.msgs), RUNTIME * 15)  # Fewer than 15 messages per second

    def test_cpu_info(self):
        # get current system information
        system, node, release, version, machine, processor = platform.uname()
        platform_supported = False
        for pform in platforms:
            if(platforms[pform]['system'] == system):
                if(platforms[pform]['processor'] == processor):
                    platform_supported = True
                    self.assertEqual(multiprocessing.cpu_count(), platforms[pform]['cores'])
                    if(platforms[pform]['realtime']):
                        self.assertNotEqual(version.find('PREEMPT_RT'), -1)

        self.assertEqual(platform_supported, True)


@post_shutdown_test()
class TestReferenceSystemAutowareAfterShutdown(unittest.TestCase):

    def test_process_exit_codes(self):
        # Checks that all processes exited cleanly
        assertExitCodes(self.proc_info)
