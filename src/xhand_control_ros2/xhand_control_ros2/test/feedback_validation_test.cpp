#include "xhand_control_ros2/feedback_validation.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition) {
  if (!condition) throw std::runtime_error("feedback validation regression");
}

int main() {
  HandState_t state{};
  require(xhand_control_ros2::feedback_is_usable(0, state));
  for (int error : {1501050, 1501006, 1501070}) {
    require(!xhand_control_ros2::feedback_is_usable(error, state));
  }
  for (size_t i = 0; i < state.finger_state.size(); ++i) {
    state.finger_state[i].position = std::numeric_limits<float>::quiet_NaN();
    require(!xhand_control_ros2::feedback_is_usable(0, state));
    state.finger_state[i].position = std::numeric_limits<float>::infinity();
    require(!xhand_control_ros2::feedback_is_usable(0, state));
    state.finger_state[i].position = 0;
    require(xhand_control_ros2::feedback_is_usable(0, state));
  }
  std::cout << "PASS: SDK errors and all 12 non-finite joint positions rejected; valid recovery accepted\n";
}
