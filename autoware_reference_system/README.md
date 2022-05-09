# Profiling executors using the Autoware reference system

## Introduction

This tutorial incorporates [the open-sourced `autoware_reference_system`](
https://github.com/ros-realtime/reference-system) and can be used to fairly and repeatably test
the performance of the various executors available within the greater ROS 2 community.

The example simulates a real world scenario, [Autoware.Auto](
https://www.autoware.org/autoware-auto) and its LiDAR data pipeline, that can be used to evaluate
the performance of the executor. To this end, the example comes with built-in performance
measurements that make it easy to compare the performance between executor implementations
in a repeatable way.

![The Autoware reference system](../content/img/autoware_reference_system.svg)

## Quick Start

Some tools are provided in order to automate and standardize the report generation process for this
`autoware_reference_system`.

First, install and build the dependencies

```console
(ade) $ python3 -m pip install psrecord bokeh  # optional dependency: networkx
(ade) $ cd workspace
(ade) $ colcon build --packages-up-to autoware_reference_system
```

The easiest way to run the benchmarks is through the `ctest` interface. Rebuild the package
with the `RUN_BENCHMARK` option and run `colcon test`:

```console
(ade) $ colcon build --packages-select autoware_reference_system \
        --cmake-force-configure --cmake-args -DRUN_BENCHMARK=ON
(ade) $ colcon test --packages-select autoware_reference_system
```

After the tests have run, reports can be found as `.html` files  in
`$ROS_HOME/benchmark_autoware_reference_system/<timestamp>` (`$ROS_HOME` defaults to `~/.ros`).
The symlink `$ROS_HOME/benchmark_autoware_reference_system/latest` always points to the latest
results. Detailed reports to individual test runs can be found in subdirectories of the form
`<duration>/<middleware>/<executable>`.

More details on all the supported CMake arguments can be found in
[the supported CMake argument section](#supported-cmake-arguments) below.

By default the tests uses the default ROS 2 middleware set for the system.
To run the tests for all available RMWs, add the
`-DALL_RMWS=ON` CMake argument to the `colcon build` step.

The test duration can be configured through the `RUN_TIMES` variable in `CMakelists.txt`.
A separate set of tests is created for each chosen runtime.

## Test Results and Reports

Reports are automatically generated depending on which tests are run. The main test directory
(`$ROS_HOME/benchmark_autoware_reference_system/latest` by default) contains the *summary
reports*,
which aggregate metrics across all tested configurations.

Below this main test directory, each tested configuration has a subdirectory of the form
`<duration>/<middleware>/<executable name>`. This directory contains the raw trace data and
additional per-test reports in `.html` format.

## Tweaking the benchmark setup

To get more fine-grained control over the benchmarking process invoke the benchmark script
directly. To get a summary of the available options, call

```console
(ade) $ $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py --help
```

As an example, to run all benchmarks starting with `autoware_` and the
`autoware_default_multithreaded` benchmark for 15 seconds run

```console
(ade) $ $(ros2 pkg prefix --share autoware_reference_system)/scripts/benchmark.py \
        15 'autoware_*'
```

The `--logdir` option can be used to store the measurement results and reports in a custom
directory, without adding a timestamp. Note that this may overwrite existing measurement
results in the same directory.

## **Key Performance Indicators (KPIs)**

The performance measurement evaluates the executor using the following metrics. In general, the
lowest value within each KPI is considered to be the better performance.

- **CPU utilization**
    - In general a lower CPU utilization is better since it enables you to choose a smaller CPU or
    have more functionality on a larger CPU for other things.
- **Memory utilization**
    - In general a lower memory utilization is better since it enables you to choose a smaller
    memory or have more space for other things
- **Number of dropped sensor samples in transform nodes**
    - The nodes in the reference system always use the most recent sensor data
    (i.e., use a history depth of 1)
        - This is a common strategy in real-world settings, as old sensor data
        is much less valuable than new sensor data
        - For example an image from 30 seconds ago is much less helpful while driving down
        the road than an image from 0.1 second ago
    - Fusion nodes drop messages during normal operation if the inputs publish with different
    frequencies
    - In transform nodes, however, dropped messages indicate that the transform node cannot
    keep up with its input

- **Number of front LiDAR samples that did not trigger an update in the Object Collision Estimator**
    - The Front and Rear LiDARs have the same publishing frequency
    - This means Object Collision Estimator should run for every LiDAR sample
    - Count number of executions of Object Collision Estimator and Front LiDAR and report any difference
- **Worst-case Latency between Front LiDAR and the Object Collision Estimator**
    - For worst-case latency we want to identify obstacles in time
    (i.e. early enough that we can still emergency-brake).
- **Average Latency between Front LiDAR and Object Collision Estimator**
    - For average latency we want to identify obstacles as soon as possible so we can account for
    the obstacle in our planning.
- **The Behavior Planner should be as cyclical as possible**
    - The desired behavior of the Behavior Planner is to be as cyclical as possible, meaning it
    should be executed as close to its set frequency of _100ms_ as possible
    - Measure the jitter and drift over time of the timer callback

## Message Types

A **single message type** is used for the entire reference system when generating results in
order to simplify the setup as well as make it more repeatable and extensible.

This means **only one message type** from the list below is used during any given experimental
run for every node in the reference system.

1. **Message4kB**
    - reference message with a fixed size of 4 kilobytes (kB)

Other messages with different fixed sizes could be added here in the future.

When reporting results it is important to specify the message type used during the experiment, as
the message size impacts the metrics.

## Autoware Reference System

Built from [a handful of building-block node types](the-reference-system-design.md#base-node-types)
, each one of these nodes are meant to simulate a
real-world node from the Autoware.Auto project
LiDAR data pipeline.

Under each node type are the requirements used for this specific reference system,
`autoware_reference_system`. Future reference systems could have slightly different requirements
and still use the same building-block node types.

For simplicity, every node except for the command nodes only publishes one topic, and this topic
has the same name as the node that publishes it. However, each topic can be subscribed to by
multiple different nodes.

Also for simplicity, every node that does processing (aka number crunching) by default is
configured to do that processing for the same amount of time: roughly 10 milliseconds.
This processing time varies
drastically depending on what platform you are on since each node does a fixed amount of
actual work, not a fixed amount of time.  See [the Configuring Processing Time section
](#configure-processing-time) for more details.

1. [**Message Type**](#message-types)
    - all nodes use the same message type during any single test run
    - default message type:
        - Message4kB
    - to be implemented:
        - Message64kB
        - Message256kB
        - Message512kB
        - Message1024kB
        - Message5120kB
2. **Sensor Nodes**
    - all sensor node have a publishing rate (cycle time) of **100 milliseconds**
    - all sensor nodes publish the same message type
    - total of **5 sensor nodes**:

        - Front LiDAR Driver
        - Rear LiDAR Driver
        - Point Cloud Map
        - Visualizer
        - Lanelet2Map

3. **Transform Nodes**
    - all transform nodes have one subscriber and one publisher
    - all transform nodes start processing after a message is received
    - publishes message after processing is complete
    - total of **10 transform nodes:**
        - Front Points Transformer
        - Rear Points Transformer
        - Voxel Grid Downsampler
        - Point Cloud Map Loader
        - Ray Ground Filter
        - Object Collision Estimator
        - MPC Controller
        - Parking Planner
        - Lane Planner
4. **Fusion Nodes**
    - all fusion nodes have **two subscribers** and one publisher for this reference system
    - all fusion nodes start processing after a message is received **from all** subscriptions
    - all fusion nodes have a max input time difference between the first input received and last
    input received before publishing of **9999** seconds
    - publishes message after processing is complete
    - total of **5 fusion nodes:**
        - Point Cloud Fusion
        - NDT Localizer
        - Vehicle Interface
        - Lanelet2 Global Planner
        - Lanelet 2 Map Loader
5. **Cyclic Nodes**
    - for this reference system there is only **1 cyclic node**
    - this cyclic node has **6 subscribers**and one publisher
    - this cyclic node starts processing after a message is received **from any** single subscription
    - publishes message after processing is complete
6. **Command Nodes**
    - all command nodes have **1 subscriber** and zero publishers
    - all command nodes prints out the final latency statistics after a message is received on
    the specified topic
    - total of **2 command nodes:**
        - VehicleDBWSystem
        - IntersectionOutput
7. **Intersection Nodes**
    - for this reference system there is only EuclideanClusterDetector
    - this intersection node has **2 subscribers** and **2 publishers**
    - publishes message after processing is complete on the correspoding publisher

## Configure Processing Time

Many nodes in the reference system are actually performing some pseudo-work by finding prime
numbers up until some maximum value.  Depending on the platform, this maximum value will
need to be changed so that these nodes do not take an absurd amount of time.  This maximum value
should be chosen on a platform-by-platform basis so that the total run time of this work takes
some desired length of time.

In order to make finding this maximum value a bit easier across many different platforms a simple
**number_cruncher_benchmark** is provided that will loop
over various maximum values and spit out how long each one takes to run.  After running this
executable on your platform you should have a good idea what maximum value you should use in your
timing configuration so that each
node does some measurable work for some desired amount of time.

Here is an example output of the `number_cruncher_benchmark` run on a typical development
platform (Intel 9i7):

```console
ros2 run autoware_reference_system number_cruncher_benchmark
maximum_number     run time
          64       0.001609ms
         128       0.002896ms
         256       0.006614ms
         512       0.035036ms
        1024       0.050957ms
        2048       0.092732ms
        4096       0.22837ms
        8192       0.566779ms
       16384       1.48837ms
       32768       3.64588ms
       65536       9.6687ms
      131072       24.1154ms
      262144       62.3475ms
      524288       162.762ms
     1048576       429.882ms
     2097152       1149.79ms
```

Run the above command on your system, select your desired `run_time` and place the corresponding
`maximum_number` in the timing configuration file for the desired nodes.

## Supported CMake Arguments

- `RUN_BENCHMARK`
    - Tell CMake to build the benchmark tests that will check the reference system against its
    requirements before running a sweep of tests to generate trace files and reports
    - Without the `RUN_BENCHMARK` variable set to `ON` only the standard linter tests will be run
- `TEST_PLATFORM`
    - Test CMake to build the tests to check if the tests are being run from a
    supported platform or not
    - This flag can be omitted if you would like to run the tests on a development system before
    running them on a supported platform
    - The platform tests themselves can and should be improved going forward and are only some
    simple checks today (architecture, number of CPUs, PREEMPT_RT flag, etc.)
    - Set this to `ON` to check if the current platform is supported
- `SKIP_TRACING`
    - Set to `ON` to skip the `ros2_tracing` tests, aka the `callback` tests
    - This can greatly reduce the length of time the `colcon test` command takes to run
- `ALL_RMWS`
    - Set this to `ON` if you'd like to run tests on all available RMWs as well
    - Otherwise use only default RMW (first one listed by CMake function
    `get_available_rmw_implementations`)
    - Defaults to `OFF`

## Generating Node Graph Image

To generate the image shown above you can take advantage of [a program called `graphviz`
](https://graphviz.org/doc/info/command.html) that has a command line interface (CLI) command `dot`.

First, check out the provided `.dot` file
to get an idea of how the `dot` syntax works (feel free to modify it for your use case or
future _reference systems_).

To generate the `.dot` file into an `.svg` image, run the following command:

```console
dot -Tsvg autoware_reference_system.dot
```

_Note:_ you can change the generated image type to any of [the supported type parameters
](https://graphviz.org/docs/outputs/) if you would like a different filetype.

## Available benchmarks

The package comes with a set of benchmark executables. Many of these benchmarks distinguish
**hotpath nodes** from other nodes; these are the nodes involved in the latency KPI, starting with
the front/rear LiDAR  and ending in the ObjectCollisionEstimator. Within the hotpath nodes, the
benchmarks further distinguish between  *front/rear LiDAR nodes* (the LidarDriver and
PointsTransformer in front/rear, respectively) and  the *fusion chain nodes* (everything from
PointCloudFusion to ObjectCollisionEstimator).

Some benchmarks use real-time priorities and affinities. In this case, hotpath nodes run at
priority 1 on cores 1-3; the planner nodes runs at priority 30 on core 0; and all other nodes run
without real-time priority on all cores.

### ROS 2 benchmarks

- *autoware_default_singlethreaded*: All nodes are assigned to the same single-threaded ROS executor
- *autoware_default_multithreaded*: All nodes are assigned to the same multi-threaded ROS executor
- *autoware_default_staticsinglethreaded*: Like *autoware_default_singlethreaded*, but
   using the `StaticSingleThreadedExecutor`.
- *autoware_default_prioritized*: Separate executors for front LiDAR nodes, rear LiDAR nodes
fusion chain nodes, behavior planner, and everything else. Uses real-time priorities.
- *autoware_default_cbg*: Like *autoware_default_prioritized*, but uses the callback-group executor
  to remove the non-hotpath subscription in EuclideanClusterDetector from the executor for
  fusion chain  nodes.
