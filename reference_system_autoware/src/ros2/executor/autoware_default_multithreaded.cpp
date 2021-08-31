#include "reference_system_autoware/reference_system.hpp"

int main(int argc, char* argv[]) {
  create_and_start_reference_system<rclcpp::executors::MultiThreadedExecutor>(argc, argv);

  return 0;
}
