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
from collections import OrderedDict
import glob
import os
import re

from bokeh.models import ColumnDataSource
from bokeh.models.axes import LinearAxis
from bokeh.models.ranges import FactorRange, Range1d
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
import pandas as pd


def summary(path, size):
    data = parseLog(path)
    x = []
    latency = []
    dropped = []
    exes = set()
    rmws = set()
    for exe in data:
        exes.add(exe)
        for rmw in data[exe]:
            rmws.add(rmw)
            x.append((exe, rmw))
            latency.append(data[exe][rmw]['hot_path']['latency']['mean'])
            dropped.append(data[exe][rmw]['hot_path']['drops']['mean'])
    source = ColumnDataSource({
        'x': x,
        'latency': latency,
        'dropped': dropped})

    # initialize list of figures
    std_figs = []
    # initialize raw data figure
    summary_fig = figure(
        title='Latency and Dropped Messages Summary',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Average Latency (ms)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )

    summary_fig.vbar(
        width=0.2,
        x='x',
        top='latency',
        source=source,
        line_color='white',
        fill_color=factor_cmap('x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )

    # add extra y ranges
    summary_fig.extra_y_ranges = {
        'dropped': Range1d(
            start=0,
            end=max(dropped) + 5
        )
    }
    summary_fig.add_layout(
        LinearAxis(
            y_range_name='dropped',
            axis_label='Messages Dropped'),
        'right'
    )
    summary_fig.triangle(
        x='x',
        y='dropped',
        size=15,
        y_range_name='dropped',
        source=source,
        line_color='white',
        fill_color=factor_cmap('x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )

    summary_fig.y_range.start = 0
    summary_fig.x_range.range_padding = 0.1

    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Average Latency (ms)', '@{latency}{0.00}'),
        ('Messages Dropped', '@{dropped}{0.00}')
    ]
    summary_fig.add_tools(hover)

    std_figs = [summary_fig]
    return std_figs


def parseLog(path):
    basename = os.path.basename(path)
    # open file
    data = open(path).read().splitlines()
    # hold info on each test (start line and end line)
    log_map = {}
    in_test = False
    test_name = ''
    for index, line in enumerate(data):
        if line.find('generate_std_trace') > 0:
            if line.find('Start') > 0:
                search = ': generate_std_traces_'
                test_name = line[line.find(search) + len(search):line.find('.py')]
                rmw_idx = test_name.find('_rmw')
                exe = test_name[0:rmw_idx]
                rmw = test_name[rmw_idx + 1:test_name.rfind('_')]
                start_time = line[line.find('[') + 1:line.find(']') - 1]
                log_map[exe] = {}
                log_map[exe][rmw] = {'start': start_time, 'end': 0, 'hot_path': {'latency': {}, 'drops': {}}}
                in_test = True
            elif line.find('Passed') > 0:
                in_test = False
        # if within a test, add parse current line to dataframe
        if in_test:
            if line.find('hot path') > 0:
                if line.find('latency') > 0:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[exe][rmw]['end'] = end_time
                    log_map[exe][rmw]['hot_path']['latency'] = parseStats(line)
                elif line.find('drops') > 0:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[exe][rmw]['end'] = end_time
                    log_map[exe][rmw]['hot_path']['drops'] = parseStats(line)
    return log_map


def parseStats(line):
    # assume all stats in milliseconds
    stats = {
        'timestamp': 0.0,
        'min': 0.0,
        'max': 0.0,
        'mean': 0.0,
        'stddev': 0.0
    }
    stats['timestamp'] = float(line[line.find('[') + 1:line.find(']') - 1]) * 1000  # convert to ms
    parsed_stats = line.split(',')
    stats['min'] = parsed_stats[0][parsed_stats[0].find('min') + len('min='):]
    stats['max'] = parsed_stats[1][parsed_stats[1].find('max') + len('max='):]
    stats['mean'] = parsed_stats[2][parsed_stats[2].find('average') + len('average='):]
    stats['stddev'] = parsed_stats[3][parsed_stats[3].find('deviation') + len('deviation='):-1]
    for val in stats:
        if isinstance(stats[val], str):
            if stats[val].endswith('ms'):
                stats[val] = stats[val][:-2]
        stats[val] = float(stats[val])
    return stats