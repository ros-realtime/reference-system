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
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

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
    env = os.environ.copy()
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

    # replaced with cmake `configure_file` function
    rmw_impl = '@RMW_IMPLEMENTATION@'
    test_exe = '@TEST_EXECUTABLE@'
    test_exe_name = '@TEST_EXECUTABLE_NAME@'
    timeout = int('@TIMEOUT@')

    # specify rmw to use
    env['RCL_ASSERT_RMW_ID_MATCHES'] = rmw_impl
    env['RMW_IMPLEMENTATION'] = rmw_impl

    launch_description = LaunchDescription()
    proc_under_test = ExecuteProcess(
        cmd=[test_exe],
        name=test_exe_name,
        sigterm_timeout=str(timeout),
        output='screen',
        env=env,
    )

    trace_callback_action = Trace(
        session_name='profile_' + test_exe_name + '_' + str(RUNTIME) + 's',
        events_ust=[
            'lttng_ust_cyg_profile_fast:func_entry',
            'lttng_ust_cyg_profile_fast:func_exit',
            'lttng_ust_statedump:start',
            'lttng_ust_statedump:end',
            'lttng_ust_statedump:bin_info',
            'lttng_ust_statedump:build_id',
        ] + DEFAULT_EVENTS_ROS,
        events_kernel=[
            'sched_switch',
        ],
        context_names=[
            'ip',
        ] + DEFAULT_CONTEXT,
    )

    trace_memalloc_action = Trace(
        session_name='profile_' + test_exe_name + '_' + str(RUNTIME) + 's',
        events_ust=[
            'lttng_ust_libc:malloc',
            'lttng_ust_libc:calloc',
            'lttng_ust_libc:realloc',
            'lttng_ust_libc:free',
            'lttng_ust_libc:memalign',
            'lttng_ust_libc:posix_memalign',
        ] + DEFAULT_EVENTS_ROS,
        events_kernel=[
            'kmem_mm_page_alloc',
            'kmem_mm_page_free',
        ]
    )

    launch_description.add_action(trace_callback_action)
    launch_description.add_action(trace_memalloc_action)
    launch_description.add_action(proc_under_test)
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