// Pure C++ regression: no ROS daemon, DDS, simulator, or recording required.
#ifdef NDEBUG
#undef NDEBUG  // Keep checks active when the recorder is built in Release mode.
#endif
#include <cassert>
#include <cstdint>
#include "rosbag_recorder/recording_timestamp.hpp"

int main()
{
  using rosbag_recorder::recording_timestamp;
  constexpr int64_t wall_ns = 1789703146162515456;
  constexpr int64_t sim_ns = 60225003141;
  assert(recording_timestamp(false, false, 0, wall_ns) == wall_ns);
  assert(recording_timestamp(false, true, sim_ns, wall_ns) == wall_ns);
  assert(recording_timestamp(false, false, 0, 0) == 0);
  assert(!recording_timestamp(true, false, sim_ns, wall_ns));
  assert(!recording_timestamp(true, true, 0, wall_ns));
  assert(!recording_timestamp(true, true, -1, wall_ns));
  assert(recording_timestamp(true, true, sim_ns, wall_ns) == sim_ns);
  assert(recording_timestamp(true, true, sim_ns, 0) == sim_ns);
  // A zero-header LG2 action still gets the same timeline as simulated state.
  assert(recording_timestamp(true, true, sim_ns + 8333333, wall_ns) == sim_ns + 8333333);
  // A later episode after a simulator reset uses its new clock, no wall fallback.
  assert(recording_timestamp(true, true, 1, wall_ns) == 1);
}
