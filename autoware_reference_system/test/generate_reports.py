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

from bokeh.io import export_png, output_file
from bokeh.layouts import layout
from bokeh.plotting import save

import callback_duration
import memory_usage

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.processor.memory_usage import KernelMemoryUsageHandler
from tracetools_analysis.processor.memory_usage import UserspaceMemoryUsageHandler
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.memory_usage import MemoryUsageDataModelUtil
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

path = ''
pwd = ''
ros2_data_model = None
memory_data_model = None
callback_symbols = None
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
    # make sure path ends in a `/`
    if (p[-1] != '/'):
        p += '/'
    # make sure directory exists
    if not os.path.isdir(p):
        print('Given path does not exist: ' + p)
        sys.exit()

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
    print(pwd)
    return p


def initCallbackTraceData():
    events = load_file(path)
    handler = Ros2Handler.process(events)
    handler.data.print_data()

    global ros2_data_model
    ros2_data_model = Ros2DataModelUtil(handler.data)

    global symbols
    symbols = ros2_data_model.get_callback_symbols()

    global callback_symbols
    callback_symbols = ros2_data_model.get_callback_symbols()


def initMemoryTraceData():
    events = load_file(path)
    ros2_handler = Ros2Handler.process(events)
    ust_memory_handler = UserspaceMemoryUsageHandler()
    kernel_memory_handler = KernelMemoryUsageHandler()

    # memory usage
    ros2_handler = Ros2Handler()
    mem_proc = Processor(
        ust_memory_handler,
        kernel_memory_handler,
        ros2_handler
    )
    mem_proc.process(events)

    global ros2_data_model
    global memory_data_model
    memory_data_model = MemoryUsageDataModelUtil(
        userspace=ust_memory_handler.data,
        kernel=kernel_memory_handler.data
    )
    ros2_data_model = Ros2DataModelUtil(ros2_handler.data)


def memory_report():
    fname = path + pwd + '_memory_usage_report'
    output_file(
        filename=fname + '.html',
        title='Memory Usage Report')

    report = memory_usage.summary(size=SIZE_SUMMARY)

    save(report)
    export_png(report, filename=fname + '.png')


def callback_report():
    fname = path + pwd + '_callback_duration_report'
    output_file(
        filename=fname + '.html',
        title='Callback Duration Report')

    duration_summary = callback_duration.summary(
        callback_symbols,
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)
    duration_individual = callback_duration.individual(
        callback_symbols,
        data_model=ros2_data_model,
        size=SIZE_SUBPLOT)

    report = layout([[duration_summary], *duration_individual])

    # show(report)
    save(report)
    export_png(report, filename=fname + '.png')


def generate_reports():
    if(trace_type == TRACE_CALLBACK):
        callback_report()
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
