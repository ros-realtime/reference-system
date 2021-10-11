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
import sys

from bokeh.io import output_file
from bokeh.layouts import layout
from bokeh.plotting import save

import callback_duration
from constants import SIZE_SUBPLOT, SIZE_SUMMARY
from constants import TRACE_CALLBACK, TRACE_MEMORY, TRACE_UNSUPPORTED
import dropped_messages
import memory_usage
from utils import checkPath, getDirPath, getFileName, getPWD, getTraceType, initDataModel


def memory_report(path):
    dirPath = getDirPath(path)
    name = getFileName(path)
    fname = dirPath + name + '_memory_and_cpu_usage_report'
    output_file(
        filename=fname + '.html',
        title='Memory Usage Report')

    print('Output report to ' + fname + '.html')
    mem_individual = memory_usage.individual(dirPath + name + '.txt', size=SIZE_SUMMARY)
    report = layout([[*mem_individual]])
    save(report)
    # export_png(report, filename=fname + '.png')


def callback_report(path, ros2_data_model):
    pwd = getPWD(path)
    fname = path + pwd + '_callback_duration_report'
    output_file(
        filename=fname + '.html',
        title='Callback Duration Report')

    print('Output report to ' + fname + '.html')
    duration_summary = callback_duration.summary(
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)
    duration_individual = callback_duration.individual(
        data_model=ros2_data_model,
        size=SIZE_SUBPLOT)

    report = layout([[duration_summary], *duration_individual])

    save(report)
    # export_png(report, filename=fname + '.png')


def dropped_messages_report(path, ros2_data_model):
    pwd = getPWD(path)
    fname = path + pwd + '_dropped_messages_report'
    output_file(
        filename=fname + '.html',
        title='Dropped Messages Report')

    print('Output report to ' + fname + '.html')
    dropped_msgs = dropped_messages.individual(
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)

    report = layout([[dropped_msgs]])

    save(report)
    # export_png(report, filename=fname + '.png')


def generate_reports(path, ros2_data_model):
    if(trace_type == TRACE_CALLBACK):
        callback_report(path, ros2_data_model)
        dropped_messages_report(path, ros2_data_model)
    elif(trace_type == TRACE_MEMORY):
        memory_report(path)


if __name__ == '__main__':
    path = 'path'
    if(len(sys.argv) >= 2):
        path = sys.argv[1]
    else:
        path = '/home/ubuntu/.ros/tracing/profile'
    path = checkPath(path)
    trace_type = getTraceType(path)
    if(trace_type == TRACE_CALLBACK):
        ros2_data_model = initDataModel(path)
    elif(trace_type == TRACE_MEMORY):
        ros2_data_model = None
    elif(trace_type == TRACE_UNSUPPORTED):
        print('Trace files supplied have an unsupported profile name')
        print('Make sure they have either `callback` or `memory` in the directory name')
        sys.exit()

    generate_reports(path, ros2_data_model)
