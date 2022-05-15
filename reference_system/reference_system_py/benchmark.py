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

# Generates traces for specified executables and RMWs

import contextlib
import fcntl
import os
from pathlib import Path
import subprocess
import time

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import psutil

try:
    from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
    from trace_utils import initDataModel
    tracetools_available = True
except ModuleNotFoundError:
    tracetools_available = False


ROS_HOME = Path(os.environ.get('ROS_HOME', os.environ['HOME']+'/.ros'))
REFERENCE_SYSTEM_SHARE_DIR = Path(get_package_share_directory('reference_system'))


def available_executables(pkg, pattern='*'):
    prefix = get_package_prefix(pkg)
    return [path.stem for path in (Path(prefix)/'lib'/pkg).glob(pattern)]


def get_benchmark_directory(base_directory, executable, runtime_sec, rmw, create=False):
    """
    Return the directory to place measurements and reports for the given experiment.

    If `create` is True, the directory is created if it does not exist yet.
    """
    # Note: memory_usage.py and std_latency.py make assumptions about the directory format.
    # Do not change this without also changing these other files.
    directory = Path(base_directory)/f'{runtime_sec}s/{rmw}/{executable}/'
    if create:
        directory.mkdir(parents=True, exist_ok=True)
    return directory


def get_benchmark_directories_below(base_directory, runtime_sec=None):
    """Return all benchmark directories found below `base_directory`."""
    runtime_re = '*[0-9]' if runtime_sec is None else str(runtime_sec)
    return [str(directory) for directory in Path(base_directory).glob(f'*{runtime_re}s/rmw_*/*')]


@contextlib.contextmanager
def terminatingRos2Run(pkg, executable, rmw, env=os.environ, args=[], **kwargs):
    """
    Run the given executable (part of the given package) under the given rmw.

    The executable is automatically terminated upon exit from the context
    """
    env = env.copy()
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
    env['RMW_IMPLEMENTATION'] = rmw
    env['RCL_ASSERT_RMW_ID_MATCHES'] = rmw

    assert 'timeout' not in kwargs, ('terminatingRos2Run does not support the timeout argument;' +
                                     'use time.sleep in the with block instead')

    ros_executable = Path(get_package_prefix(pkg))/'lib'/pkg/executable
    cmdline = f'{ros_executable} {" ".join(args)}'
    process = subprocess.Popen(cmdline,
                               shell=True,
                               env=env,
                               **kwargs)
    shellproc = psutil.Process(process.pid)
    try:
        yield process
    finally:
        if process.poll() not in (None, 0):
            # Process terminated with an error
            raise RuntimeError(f'Command "{cmdline}" terminated with error: {process.returncode}')

        # The process returned by subprocess.Popen is the shell process, not the
        # ROS process. Terminating the former will not necessarily terminate the latter.
        # Terminate all the *children* of the shell process instead.
        children = shellproc.children()
        assert len(children) <= 1
        if children:
            rosproc = children[0]
            rosproc.terminate()


@contextlib.contextmanager
def roudi_daemon(env=os.environ, roudi_config_path=None):
    """
    Context manager that runs a RouDi instance for the duration of the context.

    The `env` parameter specifies environment variables for the RouDi process.
    The `roudi_config_path` parameter can be used to provide a RouDi toml configuration file.
    """
    if 'ICEORYX_HOME' not in env:
        raise RuntimeError('Cannot find ICEORYX_HOME in environment. ' +
                           'Is the iceoryx environment set up?')
    try:
        with open('/tmp/roudi.lock') as roudi_lock:
            try:
                fcntl.flock(roudi_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exception:
                raise RuntimeError('RouDi already running.') from exception
            finally:
                fcntl.flock(roudi_lock.fileno(), fcntl.LOCK_UN)
    except FileNotFoundError:
        pass  # If the file does not exist, everything is fine; roudi is not running.

    roudi_shell = subprocess.Popen(('$ICEORYX_HOME/bin/iox-roudi ' +
                                    ('' if roudi_config_path is None
                                     else f"-c '{roudi_config_path}'")),
                                   shell=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   env=env)

    try:
        yield roudi_shell
    finally:
        if roudi_shell.poll() is not None:
            raise RuntimeError(f'Roudi terminated with return code {roudi_shell.returncode}:\n' +
                               f'Output: {roudi_shell.stderr.read()}')

        roudi_process, = psutil.Process(roudi_shell.pid).children()
        roudi_process.terminate()
        roudi_process.wait()


def generate_callback_trace(executable, pkg, directory, runtime_sec, rmw):
    """
    Generate a tracefile for the given executable using the 'callback' method.

    The 'callback' method measures the executable using 'ros2 trace'
    """
    raise NotImplementedError('lttng currently does not work within ADE')

    if not tracetools_available:
        raise RuntimeError('Unable to import tracetools_trace. Are the tracetools installed?')

    log_directory = get_benchmark_directory(directory, executable, runtime_sec, rmw, create=True)

    kernel_events = []
    user_events = DEFAULT_EVENTS_ROS
    session = f'callback_trace_{pkg}_{executable}_{rmw}_{runtime_sec}s'
    tracer = subprocess.Popen(f'ros2 trace -s {session} ' +
                              f'-p {log_directory/"callback_trace"} ' +
                              f'-k {" ".join(kernel_events)} ' +
                              f'-u {" ".join(user_events)} ',
                              shell=True,
                              text=True,
                              stdin=subprocess.PIPE)
    try:
        with terminatingRos2Run(pkg, executable, rmw,
                                stdout=subprocess.DEVNULL):
            # transmit the key press that ros2 trace requires to start
            tracer.stdin.write('\n')
            time.sleep(runtime_sec)
    finally:
        tracer.terminate()


def generate_std_trace(executable, pkg, directory, runtime_sec, rmw):
    """
    Generate a tracefile for the given executable using the 'std' method.

    The 'std' method logs stdout of the executable.
    """
    log_directory = get_benchmark_directory(directory, executable, runtime_sec, rmw, create=True)

    logfile = log_directory/'std_output.log'
    with logfile.open('w', encoding='utf8') as logfd:
        with terminatingRos2Run(pkg, executable, rmw,
                                stdout=logfd,
                                text=True):
            time.sleep(runtime_sec)


def generate_memory_trace(executable, pkg, directory, runtime_sec, rmw):
    """
    Generate a tracefile for the given executable using the 'memory' method.

    The 'memory' method uses `psrecord` to profile memory and CPU usage.
    """
    log_directory = get_benchmark_directory(directory, executable, runtime_sec, rmw, create=True)
    logfile = log_directory/f'memory_log.txt'
    plotfile = log_directory/f'memory_log.png'

    psrecord_cmd = subprocess.run('command -v psrecord', shell=True, stdout=subprocess.DEVNULL)
    if psrecord_cmd.returncode != 0:
        raise RuntimeError('psrecord is not installed; install it with pip install -U psrecord')

    with terminatingRos2Run(pkg, executable, rmw,
                            stdout=subprocess.DEVNULL) as rosprocess:
        # Note: psrecord does provide a duration argument, but it just kills its subprocess
        #       using SIGKILL, which does not reliably terminate ROS programs.
        #       We therefore run psrecord inside the terminatingRos2Run instead.
        tracerprocess = subprocess.Popen(f'psrecord --include-children ' +
                                         f'--log {logfile} ' +
                                         f'--plot {plotfile} ' +
                                         f'{rosprocess.pid} ',
                                         shell=True)

        time.sleep(runtime_sec+0.5)
    tracerprocess.wait(20)


def generate_trace(trace_type, *args, **kwargs):
    if trace_type == 'memory':
        return generate_memory_trace(*args, **kwargs)
    elif trace_type == 'callback':
        return generate_callback_trace(*args, **kwargs)
    elif trace_type == 'std':
        return generate_std_trace(*args, **kwargs)
    else:
        raise ValueError(f'Invalid trace_type: {trace_type}')


def setup_benchmark_directory(pkg, create=False):
    base_dir = ROS_HOME/f'benchmark_{pkg}'
    if not create:
        latest = base_dir/'latest'
        if not latest.exists():
            raise FileNotFoundError(f'Benchmark directory {base_dir} does not exist')
        return latest

    base_dir.mkdir(exist_ok=True, parents=True)
    # Create a subdirectory with a timestamp and link 'latest' to it.
    timestamp = time.strftime('%Y-%m-%d-%H-%M-%S')
    benchmark_dir = base_dir/timestamp
    latest_symlink = base_dir/'latest'
    latest_symlink.unlink(missing_ok=True)
    latest_symlink.symlink_to(benchmark_dir)
    return benchmark_dir
