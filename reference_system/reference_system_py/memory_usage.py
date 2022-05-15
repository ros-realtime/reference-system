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
import re

from bokeh.models import ColumnDataSource
from bokeh.models.axes import LinearAxis
from bokeh.models.ranges import FactorRange, Range1d
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.markups import Div
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap

import pandas as pd

from .constants import SIZE_AXIS_LABEL, SIZE_MAJOR_LABEL, SIZE_TITLE
from .constants import SIZE_TABLE_ROW, SIZE_TABLE_WIDTH
from .plot_utils import plot_barplot


def summary_from_directories(dirs, duration, size):
    data = []
    x = []
    raw_df = []
    summary_data = {
        'exe': [],
        'rmw': [],
        'type': [],
        'low': [],
        'mean': [],
        'high': [],
        'std_dev': [],
        'box_top': [],
        'box_bottom': []
    }
    dir_re = re.compile('([0-9]+)s/(rmw_.*)/([^/]+)$')
    for idx, directory in enumerate(dirs):
        match = dir_re.search(directory)
        if not match:
            raise ValueError(f'Given directory {directory} does not match naming requirements')
        extracted_duration, rmw, exe = match.groups()
        if int(extracted_duration) != duration:
            raise ValueError(
                f'Given directory {directory} does not have expected duration {duration}')
        fpath = directory + '/memory_log.txt'
        # open datafile and parse it into a usable structure
        data.append(open(fpath).read().splitlines()[1:])
        data[idx] = [[float(element) for element in line.split()] for line in data[idx]]
        # add raw data to dataframe
        raw_df.append(
            pd.DataFrame(
                data=data[idx],
                columns=[
                    'time',
                    'cpu',
                    'real',
                    'virtual']))
        # calculate statics from raw data
        df_summary = raw_df[idx].describe().T.reset_index()

        # add data to dictionary
        for index, row in df_summary.iterrows():
            summary_data['exe'].append(exe)
            summary_data['rmw'].append(rmw)
            summary_data['type'].append(row['index'])
            summary_data['low'].append(row['min'])
            summary_data['mean'].append(row['mean'])
            summary_data['high'].append(row['max'])
            summary_data['box_top'].append(row['mean'] + row['std'])
            summary_data['box_bottom'].append(row['mean'] - row['std'])
            summary_data['std_dev'].append(row['std'])

    df = pd.DataFrame.from_records(
        summary_data, columns=[
            'exe', 'rmw', 'type', 'low', 'mean', 'high', 'box_top', 'box_bottom', 'std_dev'])
    # sort by exe and rmw
    df = df.sort_values(['exe', 'rmw'], ascending=True)

    rmws = list(df.rmw.drop_duplicates())
    x = [tuple(x) for x in df[['rmw', 'exe']].drop_duplicates().to_records(index=False)]

    cpu = df.type == 'cpu'
    real = df.type == 'real'
    virtual = df.type == 'virtual'

    cpu_source = ColumnDataSource(df[cpu])
    real_source = ColumnDataSource(df[real])
    virtual_source = ColumnDataSource(df[virtual])
    # add exe and rmw list of tuples for x axis
    cpu_source.data['x'] = x
    real_source.data['x'] = x
    virtual_source.data['x'] = x
    fill_color = factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=0, end=1)

    # initialize cpu figure
    cpu_fig = figure(
        title='CPU Usage Over Time ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='CPU (%)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    plot_barplot(cpu_fig, cpu_source, fill_color=fill_color)

    # initialize real memory figure
    real_fig = figure(
        title='Real Memory Usage Over Time ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Real Memory Usage (MB)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    plot_barplot(real_fig, real_source, fill_color=fill_color)

    # initialize virtual memory figure
    virtual_fig = figure(
        title='Virtual Memory Usage Over Time ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Virtual Memory Usage (MB)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    plot_barplot(virtual_fig, virtual_source, fill_color=fill_color)

    # columns to use in all the tables below
    columns = [TableColumn(field=field, title=title)
               for field, title in [('exe', 'Benchmark'),
                                    ('rmw', 'RMW'),
                                    ('low', 'Min'),
                                    ('mean', 'Mean'),
                                    ('high', 'Max'),
                                    ('std_dev', 'Std. Dev.')]]

    # create CPU table
    cpu_table = DataTable(
            columns=columns,
            source=ColumnDataSource(df[cpu].round(decimals=2)),
            autosize_mode='fit_viewport',
            margin=(0, 10, 10, 10),
            height=(len(df[cpu].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)

    # create real memory table
    real_table = DataTable(
            columns=columns,
            source=ColumnDataSource(df[real]),
            margin=(0, 10, 10, 10),
            height=(len(df[real].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)

    # create virtual memory table
    virtual_table = DataTable(
            columns=columns,
            source=ColumnDataSource(df[virtual]),
            margin=(0, 10, 10, 10),
            height=(len(df[virtual].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)
    # add figures and tables to output
    memory_figs = {
        'cpu_table': cpu_table,
        'cpu_fig': cpu_fig,
        'real_mem_table': real_table,
        'real_mem_fig': real_fig,
        'virtual_mem_table': virtual_table,
        'virtual_mem_fig': virtual_fig
    }
    return memory_figs


def individual(path, size):
    basename = os.path.basename(path)[:-4]
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
        title='Memory and CPU Usage Data [' + basename + ']',
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
    raw_data_fig.title.text_font_size = SIZE_TITLE
    raw_data_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    raw_data_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    raw_data_fig.xaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    raw_data_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL

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
    table_title = Div(
        text='<b>Memory and CPU Usage Data: ' + basename + '</b>',
        width=1000,
        height=20,
        style={
            'font-size': SIZE_MAJOR_LABEL
        }
    )
    summary_fig = [
        table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df_summary),
            margin=(10, 10, 10, 10),
            height=140)]
    # add figure and table to output
    memory = [[summary_fig], [raw_data_fig]]
    return memory
