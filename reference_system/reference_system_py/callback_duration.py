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
from bokeh.models import DatetimeTickFormatter
from bokeh.models import Legend
from bokeh.models.tools import HoverTool
from bokeh.plotting import figure

import numpy as np
import pandas as pd


def summary(data_model, size):
    callback_symbols = data_model.get_callback_symbols()
    colours = []  # Adds random colours for each callback
    colour_i = 0
    earliest_date = None
    fname = ''
    for obj, symbol in callback_symbols.items():
        duration_df = data_model.get_callback_durations(obj)
        thedate = duration_df.loc[:, 'timestamp'].iloc[0]
        if earliest_date is None or thedate <= earliest_date:
            earliest_date = thedate

    starttime = earliest_date.strftime('%Y-%m-%d %H:%M')
    duration = figure(
        title='Callback Durations Summary',
        x_axis_label=f'start ({starttime})',
        y_axis_label='duration (ms)',
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )

    legend_it = []
    for obj, symbol in callback_symbols.items():

        # Filter out internal subscriptions and get node information
        owner_info = data_model.get_callback_owner_info(obj)
        if not owner_info or '/parameter_events' in owner_info:
            continue

        if(len(colours) <= colour_i):
            colours.append('#%06X' % random.randint(0, 256**3-1))

        duration_df = data_model.get_callback_durations(obj)
        source = ColumnDataSource(duration_df)
        duration.title.align = 'center'

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

        c = duration.line(
            x='timestamp',
            y='duration',
            line_width=2,
            source=source,
            line_color=colours[colour_i],
            alpha=0.8,
            muted_color=colours[colour_i],
            muted_alpha=0.2,
            name=fname
        )
        legend_it.append((fname, [c]))
        colour_i += 1
        # duration.legend.label_text_font_size = '11px'
        duration.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])

    legend = Legend(
        items=legend_it,
        label_text_font_size='8pt',
        label_standoff=1,
        padding=1,
        spacing=1
    )
    legend.click_policy = 'hide'

    duration.add_layout(legend, 'right')

    # add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Callback', '$name'),
        ('Duration', '@duration{0.000}' + 's'),
        ('Timestamp', '@timestamp{%S.%3Ns}')
    ]
    hover.formatters = {
        '@timestamp': 'datetime'
    }
    duration.add_tools(hover)

    return duration
    # show(duration)
    # export_png(duration, filename=path + 'callback_duration_summary.png')


def individual(data_model, size):
    # returns a list of individual plots for each callback symbol
    callback_symbols = data_model.get_callback_symbols()
    colours = []  # Adds random colours for each callback
    colour_i = 0
    fname = ''
    figs = []
    for obj, symbol in callback_symbols.items():
        owner_info = data_model.get_callback_owner_info(obj)
        if owner_info is None:
            owner_info = '[unknown]'

        # Filter out internal subscriptions
        if '/parameter_events' in owner_info:
            continue

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

        # Duration
        duration_df = data_model.get_callback_durations(obj)
        starttime = duration_df.loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
        source = ColumnDataSource(duration_df)
        duration = figure(
            title='Callback Duration Over time',
            x_axis_label=f'start ({starttime})',
            y_axis_label='duration (ms)',
            plot_width=int(size * 1.2),
            plot_height=size,
            margin=(10, 10, 10, 175)  # Top, R, Bottom, L
        )
        duration.title.align = 'center'

        if(len(colours) <= colour_i):
            colours.append('#%06X' % random.randint(0, 256**3-1))

        duration.line(
            x='timestamp',
            y='duration',
            legend_label=fname,
            line_width=2,
            source=source,
            line_color=colours[colour_i]
        )
        # duration.legend_label.text_font_size = '11px'
        duration.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])

        # add hover tool
        hover = HoverTool()
        hover.tooltips = [
            ('Duration', '@duration{0.000}' + 's'),
            ('Timestamp', '@timestamp{%S.%3Ns}')
        ]
        hover.formatters = {
            '@timestamp': 'datetime'
        }
        duration.add_tools(hover)

        # Histogram
        # (convert to milliseconds)
        dur_hist, edges = np.histogram(duration_df['duration'] * 1000 / np.timedelta64(1, 's'))
        duration_hist = pd.DataFrame({
            'duration': dur_hist,
            'left': edges[:-1],
            'right': edges[1:],
        })
        hist = figure(
            title='Frequency of Callback Duration',
            x_axis_label='duration (ms)',
            y_axis_label='frequency',
            plot_width=int(size * 1.2),
            plot_height=size,
            margin=(10, 10, 10, 25)  # Top, R, Bottom, L
        )
        hist.title.align = 'center'
        hist.quad(
            bottom=0,
            top=duration_hist['duration'],
            left=duration_hist['left'],
            right=duration_hist['right'],
            fill_color=colours[colour_i],
            line_color=colours[colour_i],
            legend_label=fname
        )

        colour_i += 1
        figs.append([duration, hist])
    return figs
