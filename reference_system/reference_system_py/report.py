
from pathlib import Path

from . import callback_duration
from . import dropped_messages
from . import memory_usage
from . import std_latency
from .benchmark import get_benchmark_directory, get_benchmark_directories_below
from .benchmark import REFERENCE_SYSTEM_SHARE_DIR
from .constants import SIZE_SUBPLOT, SIZE_SUMMARY

from bokeh.embed import components
from bokeh.io import output_file as bokeh_output_file
from bokeh.layouts import layout as bokeh_layout
from bokeh.plotting import save as bokeh_save
import bokeh.util.version

from jinja2 import Environment, FileSystemLoader, select_autoescape

try:
    from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
    from trace_utils import initDataModel
    tracetools_available = True
except ModuleNotFoundError:
    tracetools_available = False


def generate_callback_report(executable, pkg, directory, runtime_sec, rmw):
    """Generate a per-executable report from the 'callback' trace file."""
    if not tracetools_available:
        raise RuntimeError('Unable to import tracetools_trace. Are the tracetools installed?')

    log_directory = get_benchmark_directory(directory, executable, runtime_sec, rmw)
    duration_output = log_directory/'callback_duration_report.html'
    dropped_msgs_output = log_directory/'tracing_latency_and_dropped_messages_report.html'
    data_model = initDataModel(log_directory/'callback_trace')

    bokeh_output_file(filename=duration_output,
                      title='Callback Duration Report')
    print('Output report to', duration_output)
    duration_summary = callback_duration.summary(
        data_model=data_model,
        size=SIZE_SUMMARY)
    duration_individual = callback_duration.individual(
        data_model=data_model,
        size=SIZE_SUBPLOT)
    report = bokeh_layout([[duration_summary], *duration_individual])
    bokeh_save(report)

    bokeh_output_file(filename=dropped_msgs_output,
                      title='ROS 2 Tracing Latency and Dropped Messages Report')
    print('Output report to', dropped_msgs_output)
    dropped_msgs = dropped_messages.individual(
        data_model=data_model,
        size=SIZE_SUMMARY)
    report = bokeh_layout([[dropped_msgs]])
    bokeh_save(report)


def generate_memory_report(executable, pkg, directory, runtime_sec, rmw):
    """Generate a per-executable report from the 'memory' trace file."""
    log_directory = get_benchmark_directory(directory, executable, runtime_sec, rmw)
    output = log_directory/'memory_and_cpu_usage_report.html'
    input_path = log_directory/'memory_log.txt'

    bokeh_output_file(filename=output,
                      title='Memory and CPU Usage Report')
    print('Output report to', output)
    mem_individual = memory_usage.individual(input_path,
                                             size=SIZE_SUMMARY)
    report = bokeh_layout([*mem_individual])
    bokeh_save(report)


def generate_report(trace_type, *args, **kwargs):
    if trace_type == 'memory':
        return generate_memory_report(*args, **kwargs)
    elif trace_type == 'callback':
        return generate_callback_report(*args, **kwargs)
    elif trace_type == 'std':
        return None  # No postprocessing needed for std trace
    else:
        raise ValueError(f'Invalid trace_type: {trace_type}')


def fill_in_template(template_file: Path, report_title: str, figures: dict()):
    """
    Fills in the given tempalte file with the supplied figures

    template_file: Full path to template file, including template file name
    report_title: Title of report
    figures: Figures to insert into report
    """
    # get template directory
    template_directory = template_file.parents[0]
    env = Environment(
        loader=FileSystemLoader(template_directory),
        autoescape=select_autoescape()
    )
    template = env.get_template(str(template_file.name))
    # get html and script from figures
    jinja_components = {}
    script, div = components(figures)
    jinja_components['bokeh_script'] = script
    jinja_components = {**jinja_components, **div}
    # add current bokeh CDN version to template variables
    jinja_components['bokeh_version'] = bokeh.util.version.base_version()

    return template.render(jinja_components, title=report_title)


def generate_summary_report(trace_type, pkg, directory, runtime_sec, template_file):
    """Generate a summary report for the given `trace_type`, using all traces under `directory`."""
    trace_dirs = get_benchmark_directories_below(directory, runtime_sec=runtime_sec)
    template_file_extension = template_file.split('.')[-1]

    if trace_type == 'memory':
        # use same filetype extension as input template file
        output_file = \
            f'{directory}/memory_and_cpu_usage_summary_report_{runtime_sec}s.' + \
            f'{template_file_extension}'
        report_title = 'Memory and CPU Usage Summary Report'
        mem_summary_figs = memory_usage.summary_from_directories(trace_dirs,
                                                            duration=runtime_sec,
                                                            size=SIZE_SUMMARY)

        # confirm template_file exists
        if not Path(template_file).exists():
            print(f'Template file was not found: {template_file}')
            template_file = REFERENCE_SYSTEM_SHARE_DIR / 'cfg/memory_report_template.md'
            template_file_extension = '.md'
            print(f'Falling back to default: {template_file}')
        output_report = fill_in_template(
            template_file, report_title, mem_summary_figs)

        with open(output_file, 'w') as report:
            report.write(output_report)

    elif trace_type == 'std':
        output_file = \
            f'{directory}/executor_kpi_summary_report_{runtime_sec}s.' + \
            f'{template_file_extension}'
        report_title = 'Executor Key Performance Indicator (KPI) Report'

        std_summary_figs = std_latency.summary_from_directories(trace_dirs,
                                                           duration=runtime_sec,
                                                           size=SIZE_SUMMARY)
        # confirm template_file exists
        if not Path(template_file).exists():
            print(f'Template file was not found: {template_file}')
            template_file = REFERENCE_SYSTEM_SHARE_DIR / 'cfg/std_report_template.md'
            template_file_extension = '.md'
            print(f'Falling back to default: {template_file}')
        output_report = fill_in_template(
            template_file, report_title, std_summary_figs)

        with open(output_file, 'w') as report:
            report.write(output_report)
    else:
        raise NotImplementedError(f'Unsupported trace type {trace_type}')

    print('Output report to', output_file)
