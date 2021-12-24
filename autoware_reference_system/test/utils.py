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
import errno
import os

from constants import TRACE_CALLBACK, TRACE_MEMORY, TRACE_STD
from errors import UnsupportedTraceTypeError


def checkDirPath(path: str):
    # Check if directory exists, raise error directory does not exist
    if not os.path.isdir(path):
        raise NotADirectoryError(errno.ENOENT, os.strerror(errno.ENOENT), path)
    return


def getTraceType(path):
    # Return the trace type based on the given path
    if TRACE_CALLBACK in path:
        return TRACE_CALLBACK
    elif TRACE_MEMORY in path:
        return TRACE_MEMORY
    elif TRACE_STD in path:
        return TRACE_STD
    else:
        raise UnsupportedTraceTypeError(
            'The given path is not from a supported trace type: ' + path)


def getDirPath(path):
    # Remove filename from given path and return with trailing `/`
    return os.path.dirname(path) + '/'


def getFileName(path):
    return os.path.splitext(os.path.basename(path))[0]
