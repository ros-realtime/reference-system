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

# generate summary report from all trace data
function(generate_summary_report trace_type run_time)
  if(${trace_type} MATCHES "memory")
    set(TRACE_DIR "${ROS_HOME}/${trace_type}")
  else()
    set(TRACE_DIR "${ROS_HOME}/tracing")
  endif()

  add_test(
    NAME generate_summary_report_${trace_type}_${run_time}s
    COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_summary_reports.py ${TRACE_DIR} ${run_time}"
    COMMENT "Generate Summary Report"
  )
  set_tests_properties(generate_summary_report_${trace_type}_${run_time}s
    PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
endfunction()