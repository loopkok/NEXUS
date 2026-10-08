#ifndef XHAND_CONTROL_ROS2_FEEDBACK_VALIDATION_HPP_
#define XHAND_CONTROL_ROS2_FEEDBACK_VALIDATION_HPP_

#include <cmath>
#include "data_type.hpp"

namespace xhand_control_ros2 {
// SDK failures may return a zero-filled or cached state. Never stamp those as
// a new observation. Keep this predicate independent of ROS for fault tests.
inline bool feedback_is_usable(int sdk_error, const HandState_t &state) {
  if (sdk_error != 0) {
    return false;
  }
  for (const auto &joint : state.finger_state) {
    if (!std::isfinite(joint.position)) {
      return false;
    }
  }
  return true;
}
}  // namespace xhand_control_ros2
#endif
