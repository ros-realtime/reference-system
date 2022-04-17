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
import os
import time
import unittest

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

import rclpy.context

from ros2cli.node.direct import DirectNode
import ros2topic.api
import pytest

# Tests to check if executable complies with the requirements for
# the autoware_reference_system by checking number of nodes, publishers,
# subscribers and frequency of some topics

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build

checks = {'topic_exists': False, 'pubs_match': False, 'subs_match': False}

# define autoware_reference_system requirements for each topic
# NOTE: the pub/sub counts are for the topic, not the node itself
reference_system = {
    '/FrontLidarDriver': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/RearLidarDriver': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/PointCloudMap': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/Visualizer': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/Lanelet2Map': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/PointsTransformerFront': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/PointsTransformerRear': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/VoxelGridDownsampler': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/PointCloudMapLoader': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/EuclideanClusterDetector': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/ObjectCollisionEstimator': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/MPCController': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/ParkingPlanner': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/LanePlanner': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/PointCloudFusion': {'pub_count': 1, 'sub_count': 2, 'checks': checks.copy()},
    '/NDTLocalizer': {'pub_count': 1, 'sub_count': 2, 'checks': checks.copy()},
    '/VehicleInterface': {'pub_count': 1, 'sub_count': 1, 'checks': checks.copy()},
    '/Lanelet2GlobalPlanner': {'pub_count': 1, 'sub_count': 2, 'checks': checks.copy()},
    '/Lanelet2MapLoader': {'pub_count': 1, 'sub_count': 3, 'checks': checks.copy()},
    '/BehaviorPlanner': {'pub_count': 1, 'sub_count': 2, 'checks': checks.copy()}
    # does not exist as topic but only as a node
    # '/VehicleDBWSystem': {'pub_count': 0, 'sub_count': 0, 'checks': checks.copy()}
}


def generate_test_description(ready_fn):
    env = os.environ.copy()
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

    launch_description = LaunchDescription()

    context = rclpy.context.Context()
    rclpy.init(context=context)
    launch_description.add_action(
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()))

    proc_under_test = ExecuteProcess(
        cmd=['@TEST_EXECUTABLE@'],
        name='@TEST_EXECUTABLE_NAME@',
        output='screen',
        env=env,
    )
    launch_description.add_action(proc_under_test)
    return launch_description, locals()


class TestRequirementsAutowareReferenceSystem(unittest.TestCase):

    def test_pubs_and_subs(self):
        with DirectNode([]) as node:
            seen_topics = {}
            try:
                while True:
                    print('topic_monitor looping:')
                    for name in ros2topic.api.get_topic_names(node=node):
                        if name not in seen_topics:
                            seen_topics[name] = {'pub_count': 0, 'sub_count': 0}
                        publishers = node.count_publishers(name)
                        subscribers = node.count_subscribers(name)

                        if seen_topics[name]['pub_count'] < publishers:
                            seen_topics[name]['pub_count'] = publishers

                        if seen_topics[name]['sub_count'] < subscribers:
                            seen_topics[name]['sub_count'] = subscribers

                    if len(seen_topics) > 0:
                        print('Topic monitor data collected')
                        for name in reference_system:
                            if name in seen_topics.keys():
                                reference_system[name]['checks']['topic_exists'] = True

                                if(reference_system[name]['pub_count'] ==
                                   seen_topics[name]['pub_count']):
                                    reference_system[name]['checks']['pubs_match'] = True
                                else:
                                    print('[' + name + '::pubs] EXPECTED: ' +
                                          str(reference_system[name]['pub_count']))
                                    print('[' + name + '::pubs] RECEIVED: ' +
                                          str(seen_topics[name]['pub_count']))

                                if(reference_system[name]['sub_count'] ==
                                   seen_topics[name]['sub_count']):
                                    reference_system[name]['checks']['subs_match'] = True
                                else:
                                    print('[' + name + '::subs] EXPECTED: ' +
                                          str(reference_system[name]['sub_count']))
                                    print('[' + name + '::subs] RECEIVED: ' +
                                          str(seen_topics[name]['sub_count']))

                            self.assertTrue(all(reference_system[name]['checks'].values()))
                            print(
                                f'\t\t{name}: '
                                f"['topic_exists'="
                                f"{reference_system[name]['checks']['topic_exists']},"
                                f" 'pubs_match'="
                                f"{reference_system[name]['checks']['pubs_match']},"
                                f" 'subs_match'="
                                f"{reference_system[name]['checks']['subs_match']}]")
                        # exit while loop, data was collected
                        return
                    # slow down while loop
                    time.sleep(0.5)
            except SystemError:
                pass
            except KeyboardInterrupt:
                pass
