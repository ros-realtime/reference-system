# Key performance indicator report

<script type="text/javascript" src="https://cdn.bokeh.org/bokeh/release/bokeh-{{ bokeh_version|safe }}.min.js"></script>
<script type="text/javascript" src="https://cdn.bokeh.org/bokeh/release/bokeh-tables-{{ bokeh_version|safe }}.min.js"></script>

{{ bokeh_script|safe }}

{{ latency_table|safe }}
{{ latency_fig|safe }}

{{ dropped_table|safe }}
{{ dropped_fig|safe }}

{{ period_table|safe }}
{{ period_fig|safe }}