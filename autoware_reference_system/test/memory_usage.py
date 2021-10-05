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

from bokeh.models import ColumnDataSource
from bokeh.models.axes import LinearAxis
from bokeh.models.ranges import FactorRange, Range1d
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
import pandas as pd


def summary(path, duration, size):
    print('Parse all psrecord log files')
    data = []
    df = []
    df_summary = []
    exes = set()
    rmws = set()
    summary_data = {}

    for idx, fpath in enumerate(glob.glob(path + '*' + duration + '*.txt')):
        fname = os.path.basename(fpath)
        if (fname.find('.txt') >= 0):
            fname = fname[:-4]

        tmp_name = fname.find('_rmw')
        exe_name = fname[0:tmp_name]
        rmw_name = fname[tmp_name + 1:-(len(duration) + 2)]

        exes.add(exe_name)
        rmws.add(rmw_name)

        try:
            summary_data[exe_name]
        except KeyError:
            summary_data[exe_name] = {}
        try:
            summary_data[exe_name][rmw_name]
        except KeyError:
            summary_data[exe_name][rmw_name] = {}

        data.append(open(fpath).read().splitlines()[1:])
        data[idx] = [[float(element) for element in line.split()] for line in data[idx]]

        df.append(
            pd.DataFrame(
                data=data[idx],
                columns=[
                    'Elapsed Time',
                    'CPU (%)',
                    'Real (MB)',
                    'Virtual (MB)']))
        # df[idx] = df[idx].sample(200)
        df_summary.append(df[idx].describe().T.reset_index())
        avg_data = df_summary[idx]['mean'].tolist()
        summary_data[exe_name][rmw_name] = avg_data

    print(summary_data)
    # sort dict by key
    summary_data = OrderedDict(sorted(summary_data.items()))
    for exe in summary_data:
        summary_data[exe] = OrderedDict(sorted(summary_data[exe].items()))

    x = []
    cpu_usage = []
    real_mb = []
    virtual_mb = []
    for exe in summary_data:
        for rmw in summary_data[exe]:
            x.append((exe, rmw))
            cpu_usage.append(summary_data[exe][rmw][1])
            real_mb.append(summary_data[exe][rmw][2])
            virtual_mb.append(summary_data[exe][rmw][3])
    print(cpu_usage)
    source = ColumnDataSource({
        'x': x,
        'cpu_usage': cpu_usage,
        'real_mb': real_mb,
        'virtual_mb': virtual_mb})

    # initialize list of figures
    memory = []
    # initialize raw data figure
    summary_fig = figure(
        title='Memory and CPU Usage Summary',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Average CPU (%)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )

    summary_fig.vbar(
        width=0.2,
        x='x',
        top='cpu_usage',
        source=source,
        line_color='white',
        fill_color=factor_cmap('x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )

    # add extra y ranges
    summary_fig.extra_y_ranges = {
        'real_mb': Range1d(
            start=0,
            end=max(real_mb) + 5
        )
    }
    summary_fig.add_layout(
        LinearAxis(
            y_range_name='real_mb',
            axis_label='Real (MB)'),
        'right'
    )
    summary_fig.triangle(
        x='x',
        y='real_mb',
        size=15,
        y_range_name='real_mb',
        source=source,
        line_color='white',
        fill_color=factor_cmap('x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )

    summary_fig.legend.location = 'top_right'
    summary_fig.y_range.start = 0
    summary_fig.x_range.range_padding = 0.1

    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Mean CPU (%)', '@{cpu_usage}{0.00}'),
        ('Real (MB)', '@{real_mb}{0.00}'),
        ('Virtual (MB)', '@{virtual_mb}{0.00}')
    ]
    summary_fig.add_tools(hover)

    memory = [summary_fig]
    return memory


def individual(path, size):
    print('Parse psrecord log files')
    print(path)
    # open file
    data = open(path).read().splitlines()[1:]
    data = [[float(element) for element in line.split()] for line in data]
    df = pd.DataFrame(data=data, columns=['Elapsed Time', 'CPU (%)', 'Real (MB)', 'Virtual (MB)'])
    # add summary stats
    df_summary = df.describe().T.reset_index()
    # initialize list of figures
    memory = []
    source = ColumnDataSource(df)
    # colors
    colors = [
        '#158171',
        '#286f80',
        '#1bab78'
    ]
    # initialize raw data figure
    raw_data_fig = figure(
        title='Memory and CPU Usage Data',
        x_axis_label=f'Time (sec)',
        y_axis_label='CPU (%)',
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    # add CPU usage to figure
    raw_data_fig.line(
        x='Elapsed Time',
        y='CPU (%)',
        line_width=1,
        source=source,
        line_color=colors[0],
        alpha=0.8,
        legend_label='CPU (%)',
        muted_color=colors[0],
        muted_alpha=0.2
    )
    # legend attributes
    raw_data_fig.legend.location = 'top_right'
    raw_data_fig.legend.click_policy = 'hide'
    # add extra y ranges
    raw_data_fig.extra_y_ranges = {
        'Real (MB)': Range1d(
            start=0,
            end=df_summary.loc[2, 'max'] + 25
        )
    }
    raw_data_fig.add_layout(
        LinearAxis(
            y_range_name='Real (MB)',
            axis_label='Real (MB)'),
        'right'
    )
    # add Real memory usage to figure
    raw_data_fig.line(
        x='Elapsed Time',
        y='Real (MB)',
        y_range_name='Real (MB)',
        line_width=1,
        source=source,
        line_color=colors[1],
        alpha=0.8,
        legend_label='Real (MB)',
        muted_color=colors[1],
        muted_alpha=0.2
    )
    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('CPU (%)', '@{CPU (%)}{0.00}'),
        ('Real (MB)', '@{Real (MB)}{0.00}'),
        ('Virtual (MB)', '@{Virtual (MB)}{0.00}')
    ]
    raw_data_fig.add_tools(hover)
    # create summary table
    columns = [TableColumn(field=col, title=col) for col in df_summary.columns]
    summary_fig = DataTable(
        columns=columns,
        source=ColumnDataSource(df_summary),
        margin=(10, 10, 10, 10),
        height=140
    )
    # add figure and table to output
    memory.append([summary_fig, raw_data_fig])
    return memory
