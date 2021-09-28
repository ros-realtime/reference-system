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
from bokeh.models import DatetimeTickFormatter
from bokeh.models import Legend
from bokeh.models import NumeralTickFormatter
from bokeh.palettes import viridis
from bokeh.plotting import figure


def summary(memory_data_util, ros2_data_util, size):
    print('Getting memory usage by tid, this may take awhile...')
    ust_memory_usage_dfs = memory_data_util.get_absolute_userspace_memory_usage_by_tid()
    kernel_memory_usage_dfs = memory_data_util.get_absolute_kernel_memory_usage_by_tid()
    tids = ros2_data_util.get_tids()
    colours = viridis(len(tids) + 1)
    first_tid = min(tids)
    starttime = ust_memory_usage_dfs[first_tid]. \
        loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
    memory = figure(
        title='Memory usage per thread/node',
        x_axis_label=f'time ({starttime})',
        y_axis_label='memory usage',
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )

    i_colour = 0
    legend_it = []
    print('Generating memory usage figures')
    for tid in tids:
        name = str(tid) + ' ' + str(ros2_data_util.get_node_names_from_tid(tid))
        # Userspace
        ust = memory.line(
            x='timestamp',
            y='memory_usage',
            line_width=2,
            source=ColumnDataSource(ust_memory_usage_dfs[tid]),
            line_color=colours[i_colour],
        )
        legend_it.append((name + ' (ust)', [ust]))
        # Kernel
        ker = memory.line(
            x='timestamp',
            y='memory_usage',
            line_width=2,
            source=ColumnDataSource(kernel_memory_usage_dfs[tid]),
            line_color=colours[i_colour],
            line_dash='dotted',
        )
        i_colour += 1
        legend_it.append((name + ' (kernel)', [ker]))

    memory.title.align = 'center'
    memory.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])
    memory.yaxis[0].formatter = NumeralTickFormatter(format='0.0b')

    legend = Legend(
        items=legend_it,
        label_text_font_size='8pt',
        label_standoff=1,
        padding=1,
        spacing=1
    )
    legend.click_policy = 'mute'

    memory.add_layout(legend, 'right')
    return memory
