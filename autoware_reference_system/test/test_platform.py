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

import multiprocessing
import platform

# this file has @variables@ that are meant to be automatically replaced
# by values using the `configure_file` CMake function during the build

platforms = {}

# TODO(flynneva): move this to its own file for portability
# can add more supported platforms here
platforms['rpi4-linux-rt'] = {
    'common-name': 'raspberrypi4',
    'machine': 'aarch64',
    'processor': 'aarch64',
    'system': 'Linux',
    'flavor': 'ubuntu',
    'cores': 4,
    'realtime': True
}


def test_platform():
    # get current system information
    system, node, release, version, machine, processor = platform.uname()
    platform_supported = False
    for pform in platforms:
        if(platforms[pform]['system'] == system):
            if(platforms[pform]['processor'] == processor):
                platform_supported = True
                assert multiprocessing.cpu_count() == platforms[pform]['cores']
                if(platforms[pform]['realtime']):
                    assert version.find('PREEMPT_RT') != -1
    if platform_supported:
        print('platform supported')
        assert True
