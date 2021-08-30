#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "reference_system_autoware/types.hpp"

namespace node {
struct SensorSettings {
  std::string node_name;
  std::string topic_name;
  std::chrono::nanoseconds cycle_time;
};

class Sensor : public rclcpp::Node {
 public:
  Sensor(const SensorSettings& settings) : Node(settings.node_name) {
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 10);
    timer_ = this->create_wall_timer(settings.cycle_time,
                                     [this] { timer_callback(); });
  }

 private:
  void timer_callback() {
    auto message = publisher_->borrow_loaned_message();
    int64_t timestamp_in_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    int64_t accumulated_latency_in_ns = 0;

    message.get().data[0] = accumulated_latency_in_ns;
    message.get().data[1] = timestamp_in_ns;
    publisher_->publish(std::move(message));
  }

 private:
  publisher_t publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace node
