# Callback duration
#
# Get trace data using the provided launch file:
#    $ ros2 launch tracetools_analysis pingpong.launch.py
#    (wait a few seconds, then kill with Ctrl+C)
#
# (optional) convert trace data:
#    $ ros2 run tracetools_analysis convert ~/.ros/tracing/pingpong/ust
#
# OR
#
# Use the provided sample converted trace file, changing the path below to:
#    'sample_data/converted_pingpong'  path = '~/.ros/tracing/pingpong/ust'
path = '/home/ubuntu/.ros/tracing/autoware-multithreaded/'

#import sys
# Assuming a workspace with:
#   src/tracetools_analysis/
#   src/ros-tracing/ros2_tracing/tracetools_read/
#sys.path.insert(0, '../')
#sys.path.insert(0, '../../../ros-tracing/ros2_tracing/tracetools_read/')
import datetime as dt

from bokeh.plotting import figure
from bokeh.plotting import output_notebook
from bokeh.io import show
from bokeh.io import export_png
from bokeh.layouts import row
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter
from bokeh.models import PrintfTickFormatter
import numpy as np
import pandas as pd
import random

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil  # Process
events = load_file(path)
handler = Ros2Handler.process(events)
#handler.data.print_data()

data_util = Ros2DataModelUtil(handler.data)

callback_symbols = data_util.get_callback_symbols()

output_notebook()
psize = 450
colours = ['#29788E', '#DD4968', '#410967']  # Plot durations separately
colour_i = 0
for obj, symbol in callback_symbols.items():
    owner_info = data_util.get_callback_owner_info(obj)
    if owner_info is None:
        owner_info = '[unknown]'

    # Filter out internal subscriptions
    if '/parameter_events' in owner_info:
        continue

    # Duration
    duration_df = data_util.get_callback_durations(obj)
    starttime = duration_df.loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
    source = ColumnDataSource(duration_df)
    duration = figure(
        title=owner_info,
        x_axis_label=f'start ({starttime})',
        y_axis_label='duration (ms)',
        plot_width=psize, plot_height=psize,
    )
    duration.title.align = 'center'
    
    if(len(colours) <= colour_i):
        colours.append('#%06X' % random.randint(0, 256**3-1))
    
    duration.line(
        x='timestamp',
        y='duration',
        legend=str(symbol),
        line_width=2,
        source=source,
        line_color=colours[colour_i],
    )
    duration.legend.label_text_font_size = '11px'
    duration.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])

    # Histogram
    # (convert to milliseconds)
    dur_hist, edges = np.histogram(duration_df['duration'] * 1000 / np.timedelta64(1, 's'))
    duration_hist = pd.DataFrame({
        'duration': dur_hist, 
        'left': edges[:-1], 
        'right': edges[1:],
    })
    hist = figure(
        title='Duration histogram',
        x_axis_label='duration (ms)',
        y_axis_label='frequency',
        plot_width=psize, plot_height=psize,
    )
    hist.title.align = 'center'
    hist.quad(
        bottom=0,
        top=duration_hist['duration'], 
        left=duration_hist['left'],
        right=duration_hist['right'],
        fill_color=colours[colour_i],
        line_color=colours[colour_i],
    )

    colour_i += 1
    #show(row(duration, hist))  # Plot durations in one plot
    export_png(row(duration, hist), filename='durations_' + str(colour_i) + '.png')
    earliest_date = None
for obj, symbol in callback_symbols.items():
    duration_df = data_util.get_callback_durations(obj)
    thedate = duration_df.loc[:, 'timestamp'].iloc[0]
    if earliest_date is None or thedate <= earliest_date:
        earliest_date = thedate

starttime = earliest_date.strftime('%Y-%m-%d %H:%M')
duration = figure(
    title='Callback durations',
    x_axis_label=f'start ({starttime})',
    y_axis_label='duration (ms)',
    plot_width=psize, plot_height=psize,
)

colour_i = 0
for obj, symbol in callback_symbols.items():
    # Filter out internal subscriptions
    owner_info = data_util.get_callback_owner_info(obj)
    if not owner_info or '/parameter_events' in owner_info:
        continue

    duration_df = data_util.get_callback_durations(obj)
    source = ColumnDataSource(duration_df)
    duration.title.align = 'center'
    duration.line(
        x='timestamp',
        y='duration',
        legend=str(symbol),
        line_width=2,
        source=source,
        line_color=colours[colour_i],
    )
    colour_i += 1
    duration.legend.label_text_font_size = '11px'
    duration.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])

#show(duration)
export_png(duration, filename='duration_final.png')
