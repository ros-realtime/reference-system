#!/usr/bin/env python3
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

import argparse
import itertools

# Generates traces for specified executables and RMWs
from reference_system_py.benchmark import available_executables, generate_trace
from reference_system_py.benchmark import ROS_HOME, setup_benchmark_directory
from reference_system_py.report import generate_report, generate_summary_report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Benchmark an executor implementation.')
    parser.add_argument('runtimes',
                        help='comma-separated list of runtimes (in seconds)')
    parser.add_argument('executables',
                        help='comma-separated list of target executables')
    parser.add_argument('--trace_types',
                        help='comma-separated list of trace types (default: memory,std)',
                        default='memory,std')
    parser.add_argument('--rmws',
                        default='rmw_cyclonedds_cpp',
                        help='comma-separated list of rmw implementations')
    parser.add_argument('--logdir',
                        default=None,
                        help=('The directory where traces and results are placed (default: ' +
                              f'{ROS_HOME}/benchmark_<pkg>/<timestamp>)'))
    parser.add_argument('--plot_only',
                        default=False,
                        help='Only plot existing data, do not run new experiments.',
                        action='store_true')
    parser.add_argument('--template_file',
                        default='cfg/report_template.md',
                        help='Path to template report file to fill in')

    cmdline_args = parser.parse_args()
    common_args = {'pkg': 'autoware_reference_system',
                   'directory': cmdline_args.logdir}

    if common_args['directory'] is None:
        create_dir = (not cmdline_args.plot_only)
        common_args['directory'] = str(setup_benchmark_directory(pkg=common_args['pkg'],
                                                                 create=create_dir))

    runtimes = [int(runtime) for runtime in cmdline_args.runtimes.split(',')]
    exe_patterns, rmws, trace_types = (lst.split(',')
                                       for lst in [cmdline_args.executables,
                                                   cmdline_args.rmws,
                                                   cmdline_args.trace_types])

    exes = [exe for pattern in exe_patterns
            for exe in available_executables(pattern=pattern, pkg=common_args['pkg'])]
    for runtime, exe, rmw, trace_type in itertools.product(runtimes, exes, rmws, trace_types):
        if not cmdline_args.plot_only:
            generate_trace(trace_type, exe, rmw=rmw, runtime_sec=runtime, **common_args)
        generate_report(trace_type, exe, rmw=rmw, runtime_sec=runtime, **common_args)

    for trace_type, runtime in itertools.product(trace_types, runtimes):
        generate_summary_report(
            trace_type=trace_type,
            runtime_sec=runtime,
            template_file=cmdline_args.template_file,
            **common_args)
