#include "xhand_control_ros2/command_packet.hpp"
#include <cstring>
#include <stdexcept>
#include <iostream>

int main() {
  HandCommand_t packet;
  std::memset(&packet, 0xA5, sizeof(packet));
  packet = xhand_control_ros2::empty_command_packet();
  for (size_t i = 0; i < packet.finger_command.size(); ++i) {
    auto &joint = packet.finger_command[i];
    joint.position = 0.1f;
    joint.kp = 80;
    joint.tor_max = 400;
    joint.mode = 3;
    if (joint.id != i || joint.res0 || joint.res1 || joint.res2 || joint.res3
        || joint.ki || joint.kd) {
      throw std::runtime_error("nonzero reserved/default field in XHand wire packet");
    }
  }
  std::cout << "PASS: all 12 joint IDs and RS485 reserved/default words deterministic\n";
}
