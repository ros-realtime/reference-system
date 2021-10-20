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
import glob
import os

from bokeh.models import ColumnDataSource
from bokeh.models.axes import LinearAxis
from bokeh.models.ranges import FactorRange, Range1d
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.markups import Div
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
from constants import SIZE_AXIS_LABEL, SIZE_CATEGORY_LABEL, SIZE_MAJOR_LABEL, SIZE_TITLE
import pandas as pd


def summary(path, duration, size):
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
        'top': [],
        'bottom': []
    }
    for idx, fpath in enumerate(glob.glob(path + '*' + duration + '*.txt')):
        fname = os.path.basename(fpath)
        if (fname.find('.txt') >= 0):
            fname = fname[:-4]
        # extract exe and rmw name
        tmp_name = fname.find('_rmw')
        exe = fname[0:tmp_name]
        rmw = fname[tmp_name + 1:-(len(duration) + 2)]
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
            summary_data['top'].append(row['75%'])
            summary_data['bottom'].append(row['25%'])
            summary_data['std_dev'].append(row['std'])

    df = pd.DataFrame.from_records(
        summary_data, columns=[
            'exe', 'rmw', 'type', 'low', 'mean', 'high', 'top', 'bottom', 'std_dev'])
    # sort by exe and rmw
    df = df.sort_values(['exe', 'rmw'], ascending=True)

    exes = []
    rmws = []
    for exe in df.exe:
        for rmw in df.rmw:
            # add exe and rmw to list
            if exe not in exes:
                exes.append(exe)
            if rmw not in rmws:
                rmws.append(rmw)
    for exe in exes:
        for rmw in rmws:
            x.append((exe, rmw))
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
    # initialize cpu figure
    cpu_fig = figure(
        title='CPU Usage Summary ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='CPU (%)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    cpu_fig.segment(
        x, df.high[cpu].values, x, df.low[cpu].values, color='black', line_width=2)
    cpu_fig.vbar(
        width=0.2,
        x='x',
        top='mean',
        source=cpu_source,
        line_color='black',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    cpu_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=cpu_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    cpu_fig.y_range.start = 0
    cpu_fig.x_range.range_padding = 0.1
    cpu_fig.title.text_font_size = SIZE_TITLE
    cpu_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    cpu_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    cpu_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    cpu_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    cpu_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # initialize real memory figure
    real_fig = figure(
        title='Real Memory Usage Summary ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Real Memory Usage (MB)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    real_fig.segment(
        x, df.high[real].values, x, df.low[real].values, color='black', line_width=2)
    real_fig.vbar(
        width=0.2,
        x='x',
        top='mean',
        source=real_source,
        line_color='black',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    real_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=real_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    real_fig.y_range.start = 0
    real_fig.x_range.range_padding = 0.1
    real_fig.title.text_font_size = SIZE_TITLE
    real_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    real_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    real_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    real_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    real_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # initialize virtual memory figure
    virtual_fig = figure(
        title='Virtual Memory Usage Summary ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Virtual Memory Usage (MB)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    virtual_fig.segment(
        x, df.high[virtual].values, x, df.low[virtual].values, color='black', line_width=2)
    virtual_fig.vbar(
        width=0.2,
        x='x',
        top='mean',
        source=virtual_source,
        line_color='black',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    virtual_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=virtual_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    virtual_fig.y_range.start = 0
    virtual_fig.x_range.range_padding = 0.1
    virtual_fig.title.text_font_size = SIZE_TITLE
    virtual_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    virtual_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    virtual_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    virtual_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    virtual_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # add cpu hover tool
    cpu_hover = HoverTool()
    cpu_hover.tooltips = [
        ('Average CPU Usage (%)', '@{mean}{0.00}'),
        ('Minimum CPU Usage (%)', '@{low}{0.00}'),
        ('Maximum CPU Usage (%)', '@{high}{0.00}')
    ]
    cpu_fig.add_tools(cpu_hover)

    # add real hover tool
    real_hover = HoverTool()
    real_hover.tooltips = [
        ('Average Real Memory Used (MB)', '@{mean}{0.00}'),
        ('Minimum Real Memory Used (MB)', '@{low}{0.00}'),
        ('Maximum Real Memory Used (MB)', '@{high}{0.00}')
    ]
    real_fig.add_tools(real_hover)

    # add virtual hover tool
    virtual_hover = HoverTool()
    virtual_hover.tooltips = [
        ('Average Virtual Memory Used (MB)', '@{mean}{0.00}'),
        ('Minimum Virtual Memory Used (MB)', '@{low}{0.00}'),
        ('Maximum Virtual Memory Used (MB)', '@{high}{0.00}')
    ]
    virtual_fig.add_tools(virtual_hover)

    # add cpu usage table
    columns = [TableColumn(field=col, title=col) for col in df]
    cpu_table_title = Div(
        text='<b>CPU Usage Statistics ' + str(duration) + 's</b>',
        width=1000,
        height=10
    )
    cpu_table = [
        cpu_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[cpu]),
            margin=(10, 10, 10, 10),
            height=(len(df[cpu].exe.values.tolist()) * 33),
            width=1000)]

    # add real table
    real_table_title = Div(
        text='<b>Real Memory Usage Statistics ' + str(duration) + 's</b>',
        width=1000,
        height=10
    )
    real_table = [
        real_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[real]),
            margin=(10, 10, 10, 10),
            height=(len(df[real].exe.values.tolist()) * 33),
            width=1000)]

    # add virtual table
    virtual_table_title = Div(
        text='<b>Virtual Memory Usage Statistics ' + str(duration) + 's</b>',
        width=1000,
        height=10
    )
    virtual_table = [
        virtual_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[virtual]),
            margin=(10, 10, 10, 10),
            height=(len(df[virtual].exe.values.tolist()) * 33),
            width=1000)]
    # add figures and tables to output
    memory_figs = [
        [cpu_table], [cpu_fig],
        [real_table], [real_fig],
        [virtual_table], [virtual_fig]
    ]
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
