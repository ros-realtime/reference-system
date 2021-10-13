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

from constants import SIZE_SUMMARY
from constants import TRACE_CALLBACK, TRACE_MEMORY, TRACE_UNSUPPORTED
import dropped_messages
import memory_usage
from utils import checkPath, getTraceType


def memory_summary_report(path, duration):
    fname = path + 'memory_and_cpu_usage_summary_report_' + duration + 's'
    output_file(
        filename=fname + '.html',
        title='Memory Usage Report')

    mem_summary = memory_usage.summary(path, duration=duration, size=SIZE_SUMMARY)

    report = layout([[*mem_summary]])
    save(report)
    # export_png(report, filename=fname + '.png')


def dropped_summary_report(path, duration):
    fname = path + 'latency_and_dropped_msgs_summary_report_' + duration + 's'
    output_file(
        filename=fname + '.html',
        title='Latency and Dropped Message Summary Report')

    dropped_summary = dropped_messages.summary(
        path=path,
        size=SIZE_SUMMARY,
        duration=duration)

    report = layout([dropped_summary])

    save(report)
    # export_png(report, filename=fname + '.png')


def generate_summary_reports(path, duration):
    trace_type = getTraceType(path)
    if(trace_type == TRACE_CALLBACK):
        print('dropped summary report')
        # dropped_summary_report(path, duration)
    elif(trace_type == TRACE_MEMORY):
        print('memory summary report')
        memory_summary_report(path, duration)
    elif(trace_type == TRACE_UNSUPPORTED):
        print('Trace type unsupported')


if __name__ == '__main__':
    if(len(sys.argv) >= 2):
        path = sys.argv[1]
        duration = sys.argv[2]
    else:
        path = '/home/ubuntu/.ros/tracing/profile'
    path = checkPath(path)

    generate_summary_reports(path, duration)
