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
from bokeh.models.axes import LinearAxis
from bokeh.models.ranges import Range1d
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.plotting import figure
import pandas as pd


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
        title='CPU and Memory Usage Data',
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
