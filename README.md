# Overview [![CI status](https://github.com/ros-realtime/reference-system/actions/workflows/colcon-build.yml/badge.svg)](https://github.com/ros-realtime/reference-system/actions/workflows/colcon-build.yml)

With the distributed development of ROS across many different organizations it is sometimes hard to benchmark and concretely show how a certain change to a certain system improves or reduces the performance of that system.  For example did a change from one executor to another actually reduce the CPU or was it something else entirely?

In order to try and address this problem we at [Apex.AI](https://apex.ai) would like to propose a definition of a [_reference system_](#reference-system) that simulates a real world scenario - in this first case Autoware.Auto and its lidar data pipeline - that can be repeated no matter the underlying change of any piece of the full stack (i.e. executor, DDS or even RMW).

![Node graph of reference-system-autoware](content/img/autoware_reference_system.svg)

Future _reference systems_ could be proposed that are more complex using the same basic node building blocks within the `reference_system` package.

## Reference System

A _reference system_ is defined by:
- A [platform](#supported-platforms) is defined by:
    - Hardware (e.g. an off-the-shelf single-board computer, embedded ECU, etc.)
        - if there are multiple configurations available for such hardware, ensure it is specified
    - Operating System (OS) like RT linux, QNX, etc. along with any special configurations made
- for simplicity and ease of benchmarking, **all nodes must run on a single process**
- a fixed number of nodes
    - each node with:
        - a fixed number of publishers and subscribers
        - a fixed _processing time_ or a fixed _publishing rate_
- a fixed _message type_ of fixed size to be used for every _node_.

With these defined attributes the _reference system_ can be replicated across many different possible configurations to be used to benchmark each configuration against the other in a reliable and fair manner.

With this approach [portable and repeatable tests](#testing) can also be defined to reliably confirm if a given _reference system_ meets the requirements.

## Supported Platforms

To enable as many people as possible to replicate this reference system, the platform(s) were chosen to be easily accessible (inexpensive, high volume), have lots of documentation, large community use and will be supported well into the future.

Platforms were not chosen for performance of the reference system - we know we could run “faster” with a more powerful CPU or GPU but then it would be harder for others to validate findings and test their own configurations.  Accessibility is the key here and will be considered if more platforms want to be added to this benchmark list.

**Platforms:** 
 - [Raspberry Pi 4B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/):
    - 4 GB RAM version is the assumed default
        - other versions could also be tested / added by the community
    - [real-time linux kernel](https://github.com/ros-realtime/rt-kernel-docker-builder)

*Note: create an [issue](https://github.com/ros-realtime/reference-system-autoware/issues/) to add more platforms to the list, keeping in mind the above criteria*

## Concept Overview

Rather than trying to write code to cover all potential variations of executors, APIs, and future features we cannot even imagine today we have chosen instead to define what we call a “reference system” based on part of a real-world system, [Autoware.Auto](https://www.autoware.org/autoware-auto).

The above node graph can be boiled down to only a handful of node "types" that are replicated to make this complex system:

**Node Types:**

1. [**Sensor Node**](reference_system/include/reference_system/nodes/rclcpp/sensor.hpp)
    - input node to system
    - one publisher, zero subscribers
    - publishes message cyclically at some fixed frequency
2. [**Transform Node**](reference_system/include/reference_system/nodes/rclcpp/transform.hpp)
    - one subscriber, one publisher
    - starts processing for N milliseconds after a message is received
    - publishes message after processing is complete
3. [**Fusion Node**](reference_system/include/reference_system/nodes/rclcpp/fusion.hpp)
    - 2 subscribers, one publisher
    - starts processing for N milliseconds after a message is received **from all** subscriptions
    - publishes message after processing is complete
4. [**Cyclic Node**](reference_system/include/reference_system/nodes/rclcpp/cyclic.hpp)
    - N subscribers, one publisher
    - cyclically processes all received messages since the last cycle for N milliseconds
    - publishes message after processing is complete
5. [**Command Node**](reference_system/include/reference_system/nodes/rclcpp/command.hpp)
    - prints output stats everytime a message is received
6. [**Intersection Node**](reference_system/include/reference_system/nodes/rclcpp/intersection.hpp)
    - behaves like N transform nodes
    - N subscribers, N publisher bundled together in one-to-one connections
    - starts processing on connection where sample was received
    - publishes message after processing is complete

These basic building-block nodes can be mixed-and-matched to create quite complex systems that replicate real-world scenarios to benchmark different configurations against each other.

## Reference Systems Overview

The first reference system benchmark proposed is based on the *Autoware.Auto* lidar data pipeline as stated above and shown in the node graph image above as well.

1. [**Autoware Reference System**](autoware_reference_system/README.md)
    - ROS2:
        - Executors:
            - Default:
                - [Single Threaded](autoware_reference_system/src/ros2/executor/autoware_default_singlethreaded.cpp)
                - [Static Single Threaded](autoware_reference_system/src/ros2/executor/autoware_default_staticsinglethreaded.cpp)
                - [Multithreaded](autoware_reference_system/src/ros2/executor/autoware_default_multithreaded.cpp)

Results below show various characteristics of the same simulated system (Autoware.Auto).

To add your own executor to the list above follow the [*how to add your custom executor*](#how-to-implement-your-custom-executor) section below.

## Benchmark Results

Results will be added to different tagged releases along with the specific configurations ran during the tests.

## Testing

Tests can be written for the _reference system_ to check the platform as well as if all nodes, topics and other requirements are met before accepting test results.

Tests are provided to automatically generate results for you by running `colcon test` on a supported platform above.

**Note:** during the testing _trace data_ generated from `LTTng` will be placed in `colcon_ws/tracing`. If the tracing directory isnt there the tests will automatically generate it for you.

### Dependencies

Before running the tests there are a few prerequisites to complete:

- install LTTng and `ros2_tracing` [following the instructions in `ros2_tracing`](https://gitlab.com/ros-tracing/ros2_tracing#building)
   - _Note:_ if you are setting up [ a realtime linux kernel for a raspberry pi using this docker file](https://github.com/ros-realtime/rt-kernel-docker-builder#raspberry-pi-4-rt-linux-kernel), it should [already include LTTng](https://github.com/ros-realtime/rt-kernel-docker-builder/pull/18)
   - _Note:_ make sure to clone `ros2_tracing` into **the same workspace as where you put the `reference-system`**, the tests will not properly run if they are not in the same directory.
   - Check out the proper branch for `ros2_tracing` according to your ROS distro `git checkout $ROS_DISTRO`
- install dependencies using the following command from the `colcon_ws` directory:
    - `rosdep install --from-paths src --ignore-src -y`

Once the above steps are complete you sould be ready to run the tests and generate some results.

Source your ROS distribution as well as your `ros2_tracing` overlay, compile this repository using the proper CMake arguments and generate some test results:

```
# source your ROS distribution
source /opt/ros/galactic/setup.bash

# cd to your colcon_ws with this repo and `ros2_tracing` inside
cd /path/to/colcon_ws
# build packages with benchmark tests enabled
colcon build --cmake-args -DRUN_BENCHMARK=TRUE -DTEST_PLATFORM=TRUE

# IMPORTANT
# source the newly built workspace to make sure to use the updated tracetools package
source install/local_setup.bash
# run tests, generate traces and reports
colcon test --packages-select $(colcon list -n --base-paths src/reference-system/)
```

After the tests are complete you should now have a director called `tracing` in your `colcon_ws` top-level directory.
This directory should now hold tracing data and reports for all tests performed.

The `RUN_BENCHMARK` CMake variable will tell CMake to build the benchmark tests that will check the reference system against its requirements before running a sweep of tests to generate trace files and reports. Without the `RUN_BENCHMARK` variable set to `True` only the standard linter tests will be run.


## Contributing

If you see a missing configuration on the list above that you would like to see benchmarked against please follow the steps below to request it to be added.

- look over the open / closed [issues](https://github.com/ros-realtime/reference-system-autoware/issues/) to make sure there isn't already an open ticket for the configuration you are looking for
create `include/reference_system/MY_EXECUTOR_NAME_nodes`

## How to Implement Your Custom Executor 

1. Read over [the above documentation](#concept-overview) on the base node types
2. Review the base [`rclcpp nodes`](reference_system/include/reference_system/nodes/rclcpp) that are provided and determine if your executor can use them
3. If you cannot, implment your own version of each base node type and place the source in [with the other nodes in the `reference_system`.](reference_system/include/reference_system/nodes)
4. Add your new nodes as a seperate `node system` in [the `reference_system` package](reference_system/include/reference_system/system/systems.hpp)
5. Copy one of the provided example `.cpp` files from the [`src/ros2/executor`](autoware_reference_system/src/ros2/executor) directory and replace the `create_autoware_nodes` template type with your new `node system` which should be in the `system/systems.hpp` file already included
6. Add new `.cpp` source file as a new executable in the `CMakelist.txt`
7. Add new executable to test wtihin the `CMakelist.txt`
8. Build and run tests!


