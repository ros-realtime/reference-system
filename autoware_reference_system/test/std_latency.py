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
from bokeh.models import ColumnDataSource
from bokeh.models.ranges import FactorRange
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
import pandas as pd


def summary(path, size):
    data, test_name, hot_path_name = parseLog(path)
    x = []
    all_data = {
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
    exes = set()
    rmws = set()
    count = 0
    for results in data:
        exe = results[0]
        rmw = results[1]
        exes.add(exe)
        rmws.add(rmw)
        x.append((exe, rmw))
        for data_type in results[2]['hot_path']:
            all_data['exe'].append(exe)
            all_data['rmw'].append(rmw)
            all_data['type'].append(data_type)
            all_data['low'].append(results[2]['hot_path'][data_type]['low'])
            all_data['mean'].append(results[2]['hot_path'][data_type]['mean'])
            all_data['high'].append(results[2]['hot_path'][data_type]['high'])
            all_data['std_dev'].append(results[2]['hot_path'][data_type]['std_dev'])
            all_data['top'].append(results[2]['hot_path'][data_type]['top'])
            all_data['bottom'].append(results[2]['hot_path'][data_type]['bottom'])
            count += 1
    df = pd.DataFrame.from_records(
        all_data, columns=[
            'exe', 'rmw', 'type', 'low', 'mean', 'high', 'top', 'bottom', 'std_dev'])
    latency = df.type == 'latency'
    dropped = df.type == 'dropped'
    latency_source = ColumnDataSource(df[latency])
    dropped_source = ColumnDataSource(df[dropped])
    latency_source.data['x'] = x
    dropped_source.data['x'] = x
    # initialize list of figures
    std_figs = []
    # initialize raw data figure
    latency_fig = figure(
        title='Latency Summary [' + hot_path_name + ']',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Average Latency (ms)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    latency_fig.segment(
        x, df.high[latency].values, x, df.low[latency].values, color='black', line_width=4)
    latency_fig.vbar(
        width=0.2,
        x='x',
        top='top',
        bottom='bottom',
        line_color='white',
        source=latency_source,
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )

    # initialize raw data figure
    dropped_fig = figure(
        title='Dropped Messages Summary [' + hot_path_name + ']',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Dropped Messages',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    dropped_fig.segment(
        x, df.high[dropped].values, x, df.low[dropped].values, color='black', line_width=4)
    dropped_fig.vbar(
        width=0.2,
        x='x',
        top='top',
        bottom='bottom',
        source=dropped_source,
        line_color='white',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=3, end=4)
    )

    dropped_fig.y_range.start = 0
    dropped_fig.x_range.range_padding = 0.1

    # add hover tools
    latency_hover = HoverTool()
    latency_hover.tooltips = [
        ('Average Latency (ms)', '@{mean}{0.00}'),
        ('Minimum Latency (ms)', '@{low}{0.00}'),
        ('Maximum Latency (ms)', '@{high}{0.00}'),
    ]
    latency_fig.add_tools(latency_hover)
    hover = HoverTool()
    hover.tooltips = [
        ('Dropped Messages Average', '@{mean}{0.00}'),
        ('Dropped Messages Min', '@{low}{0.00}'),
        ('Dropped Messages Max', '@{high}{0.00}'),
    ]
    dropped_fig.add_tools(hover)

    # add tables
    # create tables
    columns = [TableColumn(field=col, title=col) for col in df[latency].columns]
    latency_table = DataTable(
        columns=columns,
        source=ColumnDataSource(df[latency]),
        margin=(10, 10, 10, 10),
        height=200,
        width=1000
    )
    dropped_table = DataTable(
        columns=columns,
        source=ColumnDataSource(df[dropped]),
        margin=(10, 10, 10, 10),
        height=200,
        width=1000
    )
    std_figs = [[latency_table], [latency_fig], [dropped_table], [dropped_fig]]
    return std_figs


def parseLog(path):
    # open file
    data = open(path).read().splitlines()
    # hold info on each test (start line and end line)
    log_map = {}
    in_test = False
    test_name = ''
    hot_path_name = ''
    log_map = []
    count = 0
    for index, line in enumerate(data):
        if line.find('generate_std_trace') > 0:
            if line.find('Start') > 0:
                search = ': generate_std_traces_'
                test_name = line[line.find(search) + len(search):line.find('.py')]
                rmw_idx = test_name.find('_rmw')
                exe = test_name[0:rmw_idx]
                rmw = test_name[rmw_idx + 1:test_name.rfind('_')]
                start_time = line[line.find('[') + 1:line.find(']') - 1]
                log_map.append(
                    [exe, rmw, {
                        'start': start_time,
                        'end': 0,
                        'hot_path': {
                            'latency': {},
                            'dropped': {}
                        }
                    }])
                in_test = True
            elif line.find('Passed') > 0:
                in_test = False
                count += 1
        # if within a test, add parse current line to dataframe
        if in_test:
            if line.find('hot path') > 0:
                if line.find('latency') > 0:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[count][2]['end'] = end_time
                    log_map[count][2]['hot_path']['latency'] = parseStats(line)
                elif line.find('drops') > 0:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[count][2]['end'] = end_time
                    log_map[count][2]['hot_path']['dropped'] = parseStats(line)
    return log_map, test_name, hot_path_name


def parseStats(line):
    # assume all stats in milliseconds
    stats = {
        'timestamp': 0.0,
        'low': 0.0,
        'high': 0.0,
        'mean': 0.0,
        'std_dev': 0.0,
        'top': 0.0,
        'bottom': 0.0
    }
    stats['timestamp'] = float(line[line.find('[') + 1:line.find(']') - 1]) * 1000  # convert to ms
    parsed_stats = line.split(',')
    stats['low'] = parsed_stats[0][parsed_stats[0].find('min') + len('min='):]
    stats['high'] = parsed_stats[1][parsed_stats[1].find('max') + len('max='):]
    stats['mean'] = parsed_stats[2][parsed_stats[2].find('average') + len('average='):]
    stats['std_dev'] = parsed_stats[3][parsed_stats[3].find('deviation') + len('deviation='):-1]
    for val in stats:
        if isinstance(stats[val], str):
            if stats[val].endswith('ms'):
                stats[val] = stats[val][:-2]
        stats[val] = float(stats[val])
    stats['top'] = stats['mean'] + stats['std_dev']
    stats['bottom'] = stats['mean'] - stats['std_dev']
    if stats['bottom'] < 0:
        stats['bottom'] = 0
    return stats
