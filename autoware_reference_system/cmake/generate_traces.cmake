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

# run target for n seconds
function(generate_traces target  trace_type runtime)
    set(TEST_EXECUTABLE ${target})
    set(TEST_EXECUTABLE_NAME ${target}_${rmw_implementation})
    set(TRACE_TYPE ${trace_type})
    set(RUNTIME ${runtime})
    set(RMW_IMPLEMENTATION ${rmw_implementation})
    # ensure timeout is longer than the test runtime
    math(EXPR timeout "${runtime} + 5")
    if(${trace_type} MATCHES "memory")
      set(MEMRECORD_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/scripts/benchmark_one.sh")
      set(MEMRECORD_LOG "${ROS_HOME}/${trace_type}")
      set(MEMRECORD_LOG_NAME "${target}_${rmw_implementation}_${runtime}s.txt")
      set(MEMRECORD_EXE "$(ros2 pkg prefix ${PROJECT_NAME})/lib/${PROJECT_NAME}/${target}")
      # run psrecord script instead of ros2_tracing for memory usage plots
      add_test(
        NAME generate_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        COMMAND
          bash -c
            "chmod u+x ${MEMRECORD_PATH};
            RMW_IMPLEMENTATION=${rmw_implementation} \
            ${MEMRECORD_PATH} \
            ${MEMRECORD_EXE} \
            ${runtime} \
            ${MEMRECORD_LOG} \
            ${MEMRECORD_LOG_NAME}"
      )
      set_tests_properties(generate_${trace_type}_trace_${target}_${rmw_implementation}_${runtime}s
        PROPERTIES TIMEOUT ${DEFAULT_TIMEOUT})
    else()
      # replaces all @var@ and ${var} within input file
      configure_file(
        test/generate_${trace_type}_traces.py
        generate_${trace_type}_traces_${target}_${rmw_implementation}_${runtime}s.py
        @ONLY
      )
      add_ros_test(
        ${CMAKE_CURRENT_BINARY_DIR}/generate_${trace_type}_traces_${target}_${rmw_implementation}_${runtime}s.py
        TIMEOUT ${timeout}  # seconds
      )
    endif()
    if(TARGET ${target})
      ament_target_dependencies(${target}
        "rclcpp" "reference_interfaces" "reference_system")
    endif()
endfunction()