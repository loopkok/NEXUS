#ifndef XHAND_CONTROL_ROS2_COMMAND_PACKET_HPP_
#define XHAND_CONTROL_ROS2_COMMAND_PACKET_HPP_
#include "data_type.hpp"

namespace xhand_control_ros2 {
inline HandCommand_t empty_command_packet() {
  // Reserved force words travel over RS485 even in position mode.
  HandCommand_t command{};
  for (size_t i = 0; i < command.finger_command.size(); ++i) {
    command.finger_command[i].id = i;
  }
  return command;
}
}  // namespace xhand_control_ros2
#endif
