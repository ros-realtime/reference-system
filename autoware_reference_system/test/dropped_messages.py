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
import random

from bokeh.models import ColumnDataSource
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.plotting import figure
import pandas as pd


def summary(data_model, size):
    callback_symbols = data_model.get_callback_symbols()
    colors = []  # Adds random colors for each callback
    color_i = 0
    earliest_date = None
    latest_date = None
    fname = ''
    dropped_data = []
    period_data = []
    for obj, symbol in callback_symbols.items():
        callback_df = data_model.get_callback_durations(obj)
        # get node information and filter out internal subscriptions
        owner_info = data_model.get_callback_owner_info(obj)
        if not owner_info or '/parameter_events' in owner_info:
            continue
        period = 0.0
        if not owner_info or 'period' in owner_info:
            # assume in milliseconds so convert to seconds
            period = float(owner_info[
                owner_info.find('period: ') + len('period: '):owner_info.rfind(' ')]) / 1000
        # add color to list if needed
        if(len(colors) <= color_i):
            colors.append('#%06X' % random.randint(0, 256**3-1))
        # get human readable name of callback
        substr = 'node: '
        index = str.find(owner_info, substr)
        if index >= 0:
            index += len(substr)
            fname = 'node_' + owner_info[index:(str.find(owner_info, ','))]
        substr = 'topic: /'
        index = str.find(owner_info, substr)
        if index >= 0:
            index += len(substr)
            fname += '_topic_' + owner_info[index:]
        # get first and last timestamp of data
        thefirstdate = callback_df.loc[:, 'timestamp'].iloc[0]
        if earliest_date is None or thefirstdate <= earliest_date:
            earliest_date = thefirstdate
        thelastdate = callback_df.loc[:, 'timestamp'].iloc[len(callback_df)-1]
        if latest_date is None or thelastdate >= latest_date:
            latest_date = thelastdate
        # add name of callback and count to list
        dropped_data.append([str(fname), float(len(callback_df)), 0.0, 0.0, colors[color_i]])
        if period != 0.0:
            period_data.append([str(fname), period, 0])  # set to 0 until we know runtime
        if len(callback_df) > 200:  # this is a hack specific for the autoware reference system
            period_data.append([str(fname), 0.0, 0])  # these callbacks are after behavior planner
        else:
            period
        color_i += 1

    dropped_df = pd.DataFrame(
        dropped_data, columns=['node_name', 'count', 'dropped', 'expected_count', 'color'])
    period_df = pd.DataFrame(
        period_data, columns=['node_name', 'period', 'expected_count'])
    # calculate run time for experiment
    approx_run_time = (latest_date - earliest_date).total_seconds()
    # calculate expected counts for each period
    mask = (period_df['period'] != 0)
    period_non_zero = period_df[mask]
    period_df.loc[mask, 'expected_count'] = approx_run_time / period_non_zero['period']
    # after the behavior planner node, all nodes are triggered on every message
    expected_count_sum = sum(period_df['expected_count'])
    mask = period_df['period'] == 0
    period_df.loc[mask, 'expected_count'] = expected_count_sum
    # calculate dropped messages
    for callback in dropped_df.node_name:
        if str(callback) in str(period_df.node_name):
            expected = float(
                period_df.loc[period_df.node_name == str(callback), 'expected_count'])
            count = float(
                dropped_df.loc[dropped_df.node_name == str(callback), 'count'])
        else:
            # assume has same frequence as Front Lidar Driver
            expected = float(
                period_df.loc[
                    period_df.node_name == str('node_FrontLidarDriver'), 'expected_count'])
            count = float(
                dropped_df.loc[dropped_df.node_name == str(callback), 'count'])
        dropped_df.loc[dropped_df.node_name == str(callback), 'dropped'] = abs(expected - count)
        dropped_df.loc[dropped_df.node_name == str(callback), 'expected_count'] = expected
    source = ColumnDataSource(dropped_df)
    max_dropped = max(dropped_df['dropped']) + 0.25
    dropped = figure(
        title='Dropped Messages Summary ({:.2f} s)'.format(float(approx_run_time)),
        y_axis_label='Node Name',
        x_axis_label='Dropped Messages',
        y_range=dropped_df['node_name'],
        x_range=(0, max_dropped),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )

    dropped.hbar(
        y='node_name',
        right='dropped',
        width=0.1,
        fill_color='color',
        source=source
    )
    # add legend
    # legend = Legend(items=[dropped_df['node_name'], v])
    # dropped.add_layout(legend, 'right')
    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Callback', '@node_name'),
        ('Dropped', '@dropped'),
        ('Expected', '@expected_count'),
        ('Received', '@count')
    ]
    dropped.add_tools(hover)

    # add summary table
    dropped_summary = dropped_df.describe().T.reset_index()
    columns = [TableColumn(field=col, title=col) for col in dropped_summary]
    summary_table = DataTable(
        columns=columns,
        source=ColumnDataSource(dropped_summary),
        margin=(10, 10, 10, 10),
        height=75,
        width=size
    )
    return [summary_table, dropped]
