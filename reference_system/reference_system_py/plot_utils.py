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
import math

from bokeh.models.tools import HoverTool

from .constants import SIZE_AXIS_LABEL, SIZE_CATEGORY_LABEL, SIZE_MAJOR_LABEL, SIZE_TITLE


def plot_barplot(fig, data_source, fill_color='gray'):
    """
    Plot a barplot in `fig` using data from `data_source`.

    `data_source` is assumed to contain the following columns:
    - 'x' contains the x-coordinate
    - 'low' and 'high' contain the minimum/maximum
    - 'box_bottom' and 'box_top' contain the beginning/end of the box (mean +- stddev)
    - 'mean' contains the mean value
    """
    fig.segment(
        'x', 'box_bottom', 'x', 'low', color='black', line_width=2,
        source=data_source)
    fig.segment(
        'x', 'box_top', 'x', 'high', color='black', line_width=2,
        source=data_source)
    fig.vbar(
        width=0.2,
        x='x',
        top='box_top',
        bottom='box_bottom',
        line_color='black',
        line_width=1,
        source=data_source,
        fill_color=fill_color
    )
    fig.scatter(
        size=25,
        x='x',
        y='high',
        source=data_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=fill_color
    )
    fig.scatter(
        size=25,
        x='x',
        y='low',
        source=data_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=fill_color
    )
    fig.y_range.start = 0
    fig.x_range.range_padding = 0.1
    fig.title.text_font_size = SIZE_TITLE
    fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL
    fig.xaxis.major_label_orientation = math.pi/16

    yaxis_label = fig.yaxis[0].axis_label
    # Add hover tool
    hover = HoverTool()
    hover.tooltips = [
        ('Benchmark', '@{exe} [@{rmw}]'),
        ('Average '+yaxis_label, '@{mean}{0.00}'),
        ('Minimum '+yaxis_label, '@{low}{0.00}'),
        ('Maximum '+yaxis_label, '@{high}{0.00}'),
    ]
    fig.add_tools(hover)
