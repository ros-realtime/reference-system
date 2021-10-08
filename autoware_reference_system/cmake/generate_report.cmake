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

# generate report from traces
function(generate_report target trace_type runtime)
    if(${trace_type} MATCHES "memory")
      set(TRACE_DIR "${ROS_HOME}/${trace_type}/${target}_${rmw_implementation}_${runtime}s.txt")
      add_test(
        NAME generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
        COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_reports.py ${TRACE_DIR}"
        COMMENT "Generate CPU and Memory Usage report from psrecord data"
      )
      set_tests_properties(generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
        PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
    else()
      set(TRACE_DIR "${ROS_HOME}/tracing/${trace_type}_${target}_${rmw_implementation}_${runtime}s")
      # future note: this will fail on windows
      add_test(
        NAME convert_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        COMMAND bash -c "ros2 run tracetools_analysis convert ${TRACE_DIR}"
      )
      set_tests_properties(convert_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
      add_test(
        NAME process_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        COMMAND bash -c "ros2 run tracetools_analysis process ${TRACE_DIR}"
      )
      set_tests_properties(process_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
      add_test(
        NAME generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
        COMMAND bash -c "python3 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/generate_reports.py ${TRACE_DIR}"
        COMMENT "Process converted traces to create data model"
      )
      set_tests_properties(generate_${trace_type}_report_${target}_${rmw_implementation}_${runtime}s
        PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
    endif()
endfunction()