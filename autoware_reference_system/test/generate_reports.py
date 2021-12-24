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
from constants import TRACE_CALLBACK, TRACE_MEMORY, TRACE_STD
import dropped_messages
import memory_usage
# import std_latency

from trace_utils import initDataModel
from utils import checkDirPath, getDirPath, getFileName, getTraceType


def memory_report(wd, filename):
    fname = wd + filename + '_memory_and_cpu_usage_report'
    output_file(
        filename=fname + '.html',
        title='Memory Usage Report')

    print('Output report to ' + fname + '.html')
    mem_individual = memory_usage.individual(wd + filename + '.txt', size=SIZE_SUMMARY)
    report = layout([*mem_individual])
    save(report)
    # export_png(report, filename=fname + '.png')


def std_report(wd, filename):
    print('std report called')
    # dirPath = getDirPath(path)
    # std_summary, test_name = std_latency.individual(dirPath + 'streams.log', size=SIZE_SUMMARY)
    # fname = dirPath + test_name + '_latency_and_dropped_messages_report'
    # output_file(
    # filename=fname + '.html',
    # title='Latency and Dropped Messages Report (' + test_name + ')')
    # print('Output report to ' + fname + '.html')
    # report = layout([[*std_summary]])
    # save(report)


def callback_report(wd, filename, ros2_data_model):
    fname = wd + filename + '_callback_duration_report'
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


def dropped_messages_report(wd, filename, ros2_data_model):
    fname = wd + filename + '_tracing_latency_and_dropped_messages_report'
    output_file(
        filename=fname + '.html',
        title='ROS 2 Tracing Latency and Dropped Messages Report')

    print('Output report to ' + fname + '.html')
    dropped_msgs = dropped_messages.individual(
        data_model=ros2_data_model,
        size=SIZE_SUMMARY)

    report = layout([[dropped_msgs]])

    save(report)
    # export_png(report, filename=fname + '.png')


def generate_reports(wd, fname, trace_type, ros2_data_model):
    if(trace_type == TRACE_CALLBACK):
        callback_report(wd, fname, ros2_data_model)
        dropped_messages_report(wd, fname, ros2_data_model)
    elif(trace_type == TRACE_MEMORY):
        memory_report(wd, fname)
    elif(trace_type == TRACE_STD):
        std_report(wd, fname)


if __name__ == '__main__':
    path = 'path'
    if(len(sys.argv) >= 2):
        path = sys.argv[1]
    else:
        path = '/home/ubuntu/.ros/tracing/profile'
    # remove filename from path
    wd = getDirPath(path)
    # confirm directory exists
    checkDirPath(wd)
    # get filename
    fname = getFileName(path)
    # get trace type based on working directory path
    trace_type = getTraceType(wd)
    if(trace_type == TRACE_CALLBACK):
        ros2_data_model = initDataModel(path)
    elif(trace_type == TRACE_MEMORY):
        ros2_data_model = None
    elif(trace_type == TRACE_STD):
        ros2_data_model = None

    generate_reports(wd, fname, trace_type, ros2_data_model)
