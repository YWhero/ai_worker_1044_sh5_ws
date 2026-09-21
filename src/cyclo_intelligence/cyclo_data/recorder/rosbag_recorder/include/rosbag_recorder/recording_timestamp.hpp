// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0.

#ifndef ROSBAG_RECORDER__RECORDING_TIMESTAMP_HPP_
#define ROSBAG_RECORDER__RECORDING_TIMESTAMP_HPP_

#include <cstdint>
#include <optional>

namespace rosbag_recorder
{

// Opt-in simulation recordings follow rosbag2 --use-sim-time: wait for a
// nonzero active ROS clock, then stamp the bag timeline with that clock.
inline std::optional<int64_t> recording_timestamp(
  bool record_with_ros_clock, bool ros_clock_active,
  int64_t ros_now_ns, int64_t rmw_source_ns)
{
  if (!record_with_ros_clock) {
    return rmw_source_ns;
  }
  if (!ros_clock_active || ros_now_ns <= 0) {
    return std::nullopt;
  }
  return ros_now_ns;
}

}  // namespace rosbag_recorder

#endif  // ROSBAG_RECORDER__RECORDING_TIMESTAMP_HPP_
