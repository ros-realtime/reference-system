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
import sys

from bokeh.io import output_file
from bokeh.layouts import layout
from bokeh.plotting import save

import callback_duration
import dropped_messages
import memory_usage

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

path = ''
pwd = ''
ros2_data_model = None
memory_data_model = None
ust_memory_usage_dfs = None
kernel_memory_usage_dfs = None
trace_type = None

SIZE_SUMMARY = 800
SIZE_SUBPLOT = 500

TRACE_CALLBACK = 'callback'
TRACE_MEMORY = 'memory'
TRACE_UNSUPPORTED = 'unsupported'
# TODO(flynneva): support path as just the `tracing` directory and loop over
# all subdirectories that have tracing data in them
TRACE_DIRECTORY = 'tracing'


def checkPath(p):
    # make sure directory exists
    if not os.path.isdir(p):
        # make sure given path isnt a file
        if not os.path.isfile(p):
            print('Given path does not exist: ' + p)
            sys.exit()
        else:
            # given path is a filename, remove filename but keep rest of path
            p = os.path.dirname(p)

    # make sure path ends in a `/`
    if (p[-1] != '/'):
        if not (p.find('.txt') >= 0):
            p += '/'
    # check if given path is a callback trace
    global trace_type
    if p.find(TRACE_CALLBACK) >= 0:
        trace_type = TRACE_CALLBACK
    elif p.find(TRACE_MEMORY) >= 0:
        trace_type = TRACE_MEMORY
    else:
        trace_type = TRACE_UNSUPPORTED

    global pwd
    pwd = os.path.basename(os.path.normpath(path))
    if (pwd.find('.txt') >= 0):
        print('removing filetype extension')
        pwd = pwd[:-4]

    return p


def initCallbackTraceData():
    events = load_file(path)
    handler = Ros2Handler.process(events)
    # handler.data.print_data()

    global ros2_data_model
    ros2_data_model = Ros2DataModelUtil(handler.data)


def initMemoryTraceData():
    # TODO(flynneva): implement once ros2_tracing memory & cpu usage is fully supported
    # see: https://github.com/ros-realtime/reference-system/pull/33
    return


def memory_report():
    fname = path + pwd + '_memory_and_cpu_usage_report'
    output_file(
        filename=fname + '.html',
        title='Memory Usage Report')

    mem_individual = memory_usage.individual(path + pwd + '.txt', size=SIZE_SUMMARY)

    report = layout([[*mem_individual]])

    save(report)
    # export_png(report, filename=fname + '.png')


def callback_report():
    fname = path + pwd + '_callback_duration_report'
    output_file(
        filename=fname + '.html',
        title='Callback Duration Report')

    duration_summary = callback_duration.summary(
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)
    duration_individual = callback_duration.individual(
        data_model=ros2_data_model,
        size=SIZE_SUBPLOT)

    report = layout([[duration_summary], *duration_individual])

    save(report)
    # export_png(report, filename=fname + '.png')


def dropped_messages_report():
    fname = path + pwd + '_dropped_messages_report'
    output_file(
        filename=fname + '.html',
        title='Dropped Messages Report')

    dropped_summary = dropped_messages.summary(
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)

    report = layout([[dropped_summary]])

    save(report)
    # export_png(report, filename=fname + '.png')


def generate_reports():
    if(trace_type == TRACE_CALLBACK):
        callback_report()
        dropped_messages_report()
    elif(trace_type == TRACE_MEMORY):
        memory_report()


if __name__ == '__main__':
    if(len(sys.argv) >= 2):
        path = sys.argv[1]
    else:
        path = '/home/ubuntu/.ros/tracing/profile'
    path = checkPath(path)

    if(trace_type == TRACE_CALLBACK):
        initCallbackTraceData()
    elif(trace_type == TRACE_MEMORY):
        initMemoryTraceData()
    elif(trace_type == TRACE_UNSUPPORTED):
        print('Trace files supplied have an unsupported profile name')
        print('Make sure they have either `callback` or `memory` in the directory name')
        sys.exit()

    generate_reports()
