# Memory and CPU usage report

<script type="text/javascript" src="https://cdn.bokeh.org/bokeh/release/bokeh-{{ bokeh_version|safe }}.min.js"></script>
<script type="text/javascript" src="https://cdn.bokeh.org/bokeh/release/bokeh-tables-{{ bokeh_version|safe }}.min.js"></script>

{{ bokeh_script|safe }}

{{ cpu_table|safe }}
{{ cpu_fig|safe }}

{{ real_mem_table|safe }}
{{ real_mem_fig|safe }}

{{ virtual_mem_table|safe }}
{{ virtual_mem_fig|safe }}