# The `reference_system` package

The `reference_system` package provides reusable base components (nodes) that allow users
to construct any number of "reference systems" to replicate real-world scenarios on
real-world hardware.

The purpose of these components are not to provide the most efficient way of writing a node
but rather a reusable and repeatible way of writing a node so that it can be used to
reliably test various key performance indicators later on.

## Base node types

Most real-world systems can be boiled down to only a handful of base node "types" that are then
repeated to make the real-world system.
This does not cover _all_ possible node types, however it allows
for numerous complicated systems to be developed using the same base building blocks.

1. **Sensor Node**
    - input node to system
    - one publisher, zero subscribers
    - publishes message cyclically at some fixed frequency
2. **Transform Node**
    - one subscriber, one publisher
    - starts processing for N milliseconds after a message is received
    - publishes message after processing is complete
3. **Fusion Node**
    - 2 subscribers, one publisher
    - starts processing for N milliseconds after a message is received **from all** subscriptions
    - publishes message after processing is complete
4. **Cyclic Node**
    - N subscribers, one publisher
    - cyclically processes all received messages since the last cycle for N milliseconds
    - publishes message after processing is complete
5. **Command Node**
    - prints output stats everytime a message is received
6. **Intersection Node**
    - behaves like N transform nodes
    - N subscribers, N publisher bundled together in one-to-one connections
    - starts processing on connection where sample was received
    - publishes message after processing is complete

These basic building-block nodes can be mixed-and-matched to create quite complex systems that
replicate real-world scenarios to benchmark different configurations against each other.

New base node types can be added if necessary.

## Testing and Dependencies

Common benchmarking scripts are provided within the `reference_system/reference_system_py`
directory which is a python module itself.  The methods and tools provided there can assist
with running standardized benchmarking tests and with generating reports as well.  See
[the `autoware_reference_system` for an example](../autoware_reference_system/scripts/benchmark.py)

Unit and integration tests have also been written for the `reference_system` package and can be found
within [the `test` directory](test/test_reference_system_rclcpp.cpp). If a new system type is
to be added, new unit and integration tests should also be added as well.