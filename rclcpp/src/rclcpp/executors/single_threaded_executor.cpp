// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
    }
  }
}

#ifdef PICAS
#include <rclcpp/cb_sched.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include <linux/sched.h>

// for sched_deadline
#include <pthread.h>
#define gettid() syscall(__NR_gettid)
struct sched_attr {
    int32_t size;

    int32_t sched_policy;
    int64_t sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    int32_t sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    int32_t sched_priority;

    /* SCHED_DEADLINE (nsec) */
    int64_t sched_runtime;
    int64_t sched_deadline;
    int64_t sched_period;
};

long int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

long int sched_getattr(pid_t pid, struct sched_attr *attr, unsigned int size, unsigned int flags)
{
  return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

// Belows are the added code for ROS2-PiCAS
void
SingleThreadedExecutor::spin_cpu(int cpu)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Singlethreadedexecutor spin: in Thread ID %ld", gettid());
  if (cpu != 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_cpu has an error.");
    }
  }


  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
    }
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "singlethreadedexecutor spin in while loop.");
  }
}


void
SingleThreadedExecutor::spin_rt()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Singlethreadedexecutor spin: in Thread ID %ld", gettid());
  // Register executor as a real-time thread
  if (this->executor_priority != 0) {
    sched_param sch_params;
    sch_params.sched_priority = this->executor_priority;
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_rt thread has an error.");
    }
  }

  if (this->executor_cpu != 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(this->executor_cpu, &cpuset);
    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_rt thread has an error.");
    }
  }

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin_rt] Start spin loop. Initialize any_executable.");    
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin_rt] Done execution in any_executable.");
    } else {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin_rt] Nothing to execute in any_executable. Go to the next loop.");
    }
  }
}

void
SingleThreadedExecutor::spin_deadline(int rt_priority, int T, int budget)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Singlethreadedexecutor spin_deadline: in Thread ID %ld", gettid());

  
  // Register executor as a real-time thread
  if (rt_priority != 0) {

    struct sched_attr attr;
    long int ret;
    unsigned int flags = 0;

    attr.size = sizeof(attr);
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

     /* creates a 10ms/30ms reservation */
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = budget * 1000 * 1000;
    attr.sched_period  = T * 1000 * 1000;
    attr.sched_deadline= T * 1000 * 1000;

    ret = sched_setattr(0, &attr, flags);
    if (ret < 0) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_deadline thread has an error.");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin_deadline thread is on CPU %d.", sched_getcpu());      
  
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
      
    } 
  }
}
#endif

