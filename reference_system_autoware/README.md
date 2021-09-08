# reference_system_autoware

This file is meant to define the Autoware Reference System and all of its nodes, topics and message types.

![Node graph of reference-system-autoware](../content/img/dotgraph_autoware.svg)

## Message Types

A **single message type** is used for the entire _reference system_ when generating results in order to simplify the setup as well as make it more repeatible and extensible.

This means **only one _message type_** from the list below is used during any given experimental run for every node in the reference system.

1. [**Message1kB**](../reference_interfaces/msg/Message1kB.idl)
    - reference message with a fixed size of 1 kilobyte (kB)

Other messages with different fixed sizes could be added here in the future.

When reporting results it will be important to include the _message type_ used duing the experiement so that comparisons can be done "apples to apples" and not "apples to pears".

## Reference System Autoware

Built from [a handful of building-block node types](../README.md#concept-overview), each one of these nodes are meant to simulate a real-world node from the Autoware.Auto project lidar data pipeline.

Under each node type are the requirements used for _this specific reference system_, `reference_system_autoware`. Future reference systems could have slightly different requirements and still use the same building-block node types.

For simplicity's sake, every node except for the _command nodes_ only ever publishes one topic and this topic has the same name as the node that publishes it. However, each topic can be subscribed to by multiple different nodes.

1. [**Message Type**](#message-types)
    - all nodes use the same message type during any single test run
    - default _message type_:
        - [Message1kB](include/reference_system_autoware/types.hpp#L21)
2. [**Sensor Nodes**](reference_system_autoware/include/reference_system_autoware/node/sensor.hpp)
    - all _sensor nodes_ have a publishing rate (cycle time) of [**100 milliseconds**](include/reference_system_autoware/reference_system.hpp#L39)
    - all _sensor_nodes_ publish the same _message type_
    - total of **5 _sensor nodes_**:
        - [**Front Lidar Driver**](include/reference_system_autoware/reference_system.hpp#L40)
        - [**Rear Lidar Driver**](include/reference_system_autoware/reference_system.hpp#L45)
        - [**Point Cloud Map**](include/reference_system_autoware/reference_system.hpp#L50)
        - [**rviz2**](include/reference_system_autoware/reference_system.hpp#L55)
        - [**Lanelet2Map**](include/reference_system_autoware/reference_system.hpp#60)
3. [**Processing Nodes**](include/reference_system_autoware/node/processing.hpp)
    - all _processing nodes_ have one subscriber and one publisher
    - all _proccing nodes_ start processing for [**1000 milliseconds**](include/reference_system_autoware/reference_system.hpp#L66) after a message is received
    - publishes message after processing is complete
    - total of **10 _processing nodes_:**
        - [Front Points Transformer](include/reference_system_autoware/reference_system.hpp#L67)
        - [Rear Points Transformer](include/reference_system_autoware/reference_system.hpp#L74)
        - [Voxel Grid Downsampler](include/reference_system_autoware/reference_system.hpp#L79)
        - [Point Cloud Map Loader](include/reference_system_autoware/reference_system.hpp#L85)
        - [Ray Ground Filter](include/reference_system_autoware/reference_system.hpp#L91)
        - [Euclidean Cluster Detector](include/reference_system_autoware/reference_system.hpp#L97)
        - [Object Collision Estimator](include/reference_system_autoware/reference_system.hpp#L103)
        - [MPC Controller](include/reference_system_autoware/reference_system.hpp#L109)
        - [Parking Planner](include/reference_system_autoware/reference_system.hpp#L115)
        - [Lane Planner](include/reference_system_autoware/reference_system.hpp#L121)
4. [**Fusion Nodes**](include/reference_system_autoware/node/fusion.hpp)
    - all _fusion nodes_ have **two subscribers** and one publisher for this _reference system_
    - all _fusion nodes_ start processing for **100 milliseconds** after a message is received **from all** subscriptions
    - publishes message after processing is complete
    - total of **5 _fusion nodes_:**
        - [Point Cloud Fusion](include/reference_system_autoware/reference_system.hpp#L129)
        - [NDT Localizer](include/reference_system_autoware/reference_system.hpp#L136)
        - [Vehicle Interface](include/reference_system_autoware/reference_system.hpp#L143)
        - [Lanelet2 Global Planner](include/reference_system_autoware/reference_system.hpp#L150)
        - [Lanelet 2 Map Loader](include/reference_system_autoware/reference_system.hpp#L157)
5. [**Reactor Nodes**](include/reference_system_autoware/node/reactor.hpp)
    - for this _reference system_ there is onle [**1 _reactor node_**](include/reference_system_autoware/reference_system.hpp#L164)
    - this _reactor node_ has [**6 subscribers**](include/reference_system_autoware/reference_system.hpp#L168)and one publisher
    - this _reactor node_ starts processing for [**100 milliseconds**](include/reference_system_autoware/reference_system.hpp#L165) after a message is received **from any** single subscription
    - publishes message after processing is complete
6. [**Command Nodes**](include/reference_system_autoware/node/command.hpp)
    - for this _reference system_ there is onle [**1 _command node_**](include/reference_system_autoware/reference_system.hpp#L175)
    - this _command node_ has [**1 subscriber**](include/reference_system_autoware/reference_system.hpp#L176)and zero publishers
    - this _command node_ prints out the final latency statistics after a message is received on the specified topic
