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

from callback_duration import individual, summary

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

path = ''
pwd = ''
symbols = None
data_model = None

SIZE_SUMMARY = 800
SIZE_SUBPLOT = 500


def checkPath(p):
    # make sure path ends in a `/`
    if (p[-1] != '/'):
        p += '/'
    # make sure directory exists
    if not os.path.isdir(p):
        print('Given path does not exist: ' + p)
        sys.exit()

    global pwd
    pwd = os.path.basename(os.path.normpath(path))
    print(pwd)
    return p


def initTraceData():
    events = load_file(path)
    handler = Ros2Handler.process(events)
    handler.data.print_data()

    global data_model
    data_model = Ros2DataModelUtil(handler.data)

    global symbols
    symbols = data_model.get_callback_symbols()


def memory_usage_report():
    global path
    global pwd
    output_file(
        filename=path + pwd + '_memory_usage_report.html',
        title='Memory Usage Report')

    # duration_summary = summary(symbols, data_model=data_model, size=SIZE_SUMMARY)
    # duration_individual = individual(symbols, data_model=data_model, size=SIZE_SUMMARY)

    # report = layout([[duration_summary], *duration_individual])

    # show(report)
    # export_png(report, filename = path + 'callback_duration_report.png')


def callback_report():
    global path
    global pwd
    global data_model

    output_file(
        filename=path + pwd + '_callback_duration_report.html',
        title='Callback Duration Report')

    duration_summary = summary(symbols, data_model=data_model, size=SIZE_SUMMARY)
    duration_individual = individual(symbols, data_model=data_model, size=SIZE_SUBPLOT)

    report = layout([[duration_summary], *duration_individual])

    # show(report)
    save(report)
    export_png(report, filename=path + pwd + '_callback_duration_report.png')


def generate_reports():
    callback_report()


if __name__ == '__main__':
    if(len(sys.argv) >= 2):
        path = sys.argv[1]
    else:
        path = '/home/ubuntu/.ros/tracing/profile'
    path = checkPath(path)
    initTraceData()
    generate_reports()
