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
from collections import defaultdict
import re

from bokeh.models import ColumnDataSource
from bokeh.models.ranges import FactorRange
from bokeh.models.widgets.markups import Div
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap

import pandas as pd

from .constants import SIZE_TABLE_ROW, SIZE_TABLE_WIDTH
from .plot_utils import plot_barplot


def summary_from_directories(dirs, duration, size):
    data, hot_path_name = parseLogSummaryFromFiles(
        [directory+'/std_output.log' for directory in dirs], duration)

    df_dict = {
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

    def add_row(exe, rmw, data_type, stats):
        df_dict['exe'].append(exe)
        df_dict['rmw'].append(rmw)
        df_dict['type'].append(data_type)
        for stat, value in stats.items():
            df_dict[stat].append(value)
        df_dict['box_top'].append(stats['mean'] + stats['std_dev'])
        df_dict['box_bottom'].append(stats['mean'] - stats['std_dev'])

    for (exe, rmw), results in data.items():
        stats = results['hot_path']['latency'][-1]
        add_row(exe, rmw, 'latency', stats)
        stats = results['hot_path']['dropped'][-1]
        add_row(exe, rmw, 'dropped', stats)

        stats_list = results['behavior_planner']['period']
        if stats_list:
            add_row(exe, rmw, 'period', stats_list[-1])

    df = pd.DataFrame.from_dict(df_dict)
    # sort by exe and rmw
    df = df.sort_values(['exe', 'rmw'], ascending=True)
    rmws = list(df.rmw.drop_duplicates())
    x = [tuple(x) for x in df[['rmw', 'exe']].drop_duplicates().to_records(index=False)]
    fill_color = factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=0, end=1)

    latency = df[df.type == 'latency']
    dropped = df[df.type == 'dropped']
    period = df[df.type == 'period']
    latency_source = ColumnDataSource(latency)
    dropped_source = ColumnDataSource(dropped)
    period_source = ColumnDataSource(period)
    # add exe and rmw list of tuples for x axis
    latency_source.data['x'] = x
    dropped_source.data['x'] = x
    period_source.data['x'] = x
    # initialize dict of figures
    std_figs = {}
    # initialize latency figure
    test_info = str(duration) + 's [' + hot_path_name + ']'
    columns = [TableColumn(field=field, title=title)
               for field, title in [('exe', 'Benchmark'),
                                    ('rmw', 'RMW'),
                                    ('low', 'Min'),
                                    ('mean', 'Mean'),
                                    ('high', 'Max'),
                                    ('std_dev', 'Std. Dev.')]]

    if not latency.empty:
        latency_fig = figure(
            title='Latency Summary ' + test_info,
            x_axis_label=f'Executors (with RMW)',
            y_axis_label='Latency (ms)',
            x_range=FactorRange(*x),
            plot_width=int(size * 2.0),
            plot_height=size,
            margin=(10, 10, 10, 10)
        )
        plot_barplot(latency_fig, latency_source, fill_color=fill_color)

        latency_table = DataTable(
                columns=columns,
                source=ColumnDataSource(latency.round(decimals=3)),
                autosize_mode='fit_viewport',
                margin=(0, 10, 10, 10),
                height=(len(latency.exe.values.tolist()) * SIZE_TABLE_ROW),
                width=SIZE_TABLE_WIDTH)
        std_figs['latency_table'] = latency_table
        std_figs['latency_fig'] = latency_fig

    if not dropped.empty:
        dropped_fig = figure(
            title='Dropped Messages Summary ' + test_info,
            x_axis_label=f'Executors (with RMW)',
            y_axis_label='Dropped Messages',
            x_range=FactorRange(*x),
            plot_width=int(size * 2.0),
            plot_height=size,
            margin=(10, 10, 10, 10)
        )
        plot_barplot(dropped_fig, dropped_source, fill_color=fill_color)

        dropped_table = DataTable(
                columns=columns,
                source=ColumnDataSource(dropped.round(decimals=1)),
                autosize_mode='fit_viewport',
                margin=(0, 10, 10, 10),
                height=(len(dropped.exe.values.tolist()) * SIZE_TABLE_ROW),
                width=SIZE_TABLE_WIDTH)
        std_figs['dropped_table'] = dropped_table
        std_figs['dropped_fig'] = dropped_fig

    if not period.empty:
        period_fig = figure(
            title='Behavior Planner Jitter Summary ' + str(duration) + 's',
            x_axis_label=f'Executors (with RMW)',
            y_axis_label='Period (ms)',
            x_range=FactorRange(*x),
            plot_width=int(size * 2.0),
            plot_height=size,
            margin=(10, 10, 10, 10)
        )
        plot_barplot(period_fig, period_source, fill_color=fill_color)

        period_table = DataTable(
                columns=columns,
                source=ColumnDataSource(period.round(decimals=3)),
                autosize_mode='fit_viewport',
                margin=(0, 10, 10, 10),
                height=(len(period.exe.values.tolist()) * SIZE_TABLE_ROW),
                width=SIZE_TABLE_WIDTH)
        std_figs['period_table'] = period_table
        std_figs['period_fig'] = period_fig

    return std_figs


def parse_stats_from_values(latency_, min_, max_, average_, deviation_):
    stats = {
        'low': float(min_),
        'high': float(max_),
        'mean': float(average_),
        'std_dev': float(deviation_)}

    return stats


def parseLogSummaryFromFiles(files, duration):
    hot_path_name = None

    # result maps each pair (exe, rmw) to lists of results corresponding to the runs
    results = defaultdict(lambda: {'hot_path': {'latency': [],
                                                'dropped': []},
                                   'behavior_planner': {'period': []}})

    hot_path_name_regex = re.compile(r'^ *hot path: *(.*)$')
    hot_path_latency_regex = re.compile(r'^ *hot path latency: *(.+)ms \[min=(.+)ms, ' +
                                        r'max=(.+)ms, average=(.+)ms, deviation=(.+)ms\]$')
    hot_path_drops_regex = re.compile(r'^ *hot path drops: *(.+) \[min=(.+), max=(.+), ' +
                                      r'average=(.+), deviation=(.+)\]$')
    behavior_planner_period_regex = re.compile(r'^ *behavior planner period: *(.+)ms \[' +
                                               r'min=(.+)ms, max=(.+)ms, average=(.+)ms, ' +
                                               r'deviation=(.+)ms\]$')

    rmw_regex = re.compile(r'^RMW Implementation: (rmw_.*)')
    filename_regex = re.compile(r'.*/([0-9]+)s/(rmw_.*)/(.*)/std_output.log')
    for count, file in enumerate(files):
        match = filename_regex.match(file)
        if not match:
            raise ValueError(f'File {file} does not conform to the naming scheme')

        extracted_duration, rmw, exe = match.groups()
        if int(extracted_duration) != duration:
            raise ValueError(f'File {file} does not match expected duration {duration}')
        with open(file) as fp:
            rmw_line, *data = fp.read().splitlines()

        match = rmw_regex.match(rmw_line)
        if match and rmw != match.groups()[0]:
            raise ValueError((f'{file}: mismatch between filename-rmw ("{rmw}")' +
                              f'and content-rmw("{match.groups()[0]}")'))

        if rmw not in file:
            raise ValueError(f'File {file} contains data from RMW {rmw}, contradicting its name')

        for line in data:
            match = hot_path_name_regex.match(line)
            if match:
                name, = match.groups()
                if hot_path_name is not None and hot_path_name != name:
                    raise ValueError('Two different hotpaths in a single summary: ' +
                                     f'{name} {hot_path_name}')
                hot_path_name = name
                continue
            match = hot_path_latency_regex.match(line)
            if match:
                results[(exe, rmw)]['hot_path']['latency'].append(
                    parse_stats_from_values(*match.groups()))
                continue
            match = hot_path_drops_regex.match(line)
            if match:
                results[(exe, rmw)]['hot_path']['dropped'].append(
                    parse_stats_from_values(*match.groups()))
                continue
            match = behavior_planner_period_regex.match(line)
            if match:
                results[(exe, rmw)]['behavior_planner']['period'].append(
                    parse_stats_from_values(*match.groups()))
                continue

    if hot_path_name is None:
        raise RuntimeError('No hot_path defined in experiment.')
    return results, hot_path_name
