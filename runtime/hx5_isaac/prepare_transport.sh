#!/usr/bin/env bash
set -euo pipefail
PROFILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=cyclo_intelligence_1044_hx5_isaac
PATCH_FILE="$PROFILE/../hx5_sim/rmw_zenoh_readiness.patch"
IMAGE="$(docker inspect --format '{{.Image}}' "$CONTAINER")"
PATCH_SHA="$(sha256sum "$PATCH_FILE" | cut -d ' ' -f 1)"
BUILD_KEY="$IMAGE:$PATCH_SHA"

if docker exec "$CONTAINER" sh -c \
  'test -f /workspace/ros_transport/install/rmw_zenoh_cpp/lib/librmw_zenoh_cpp.so && test "$(cat /workspace/ros_transport/build-key 2>/dev/null)" = "$1"' sh "$BUILD_KEY"; then
  printf '%s\n' 'Isaac ROS transport is current.'
else

docker exec "$CONTAINER" mkdir -p /workspace/ros_transport
docker exec -i "$CONTAINER" sh -c 'cat > /workspace/ros_transport/readiness.patch' < "$PATCH_FILE"
docker exec -i "$CONTAINER" bash --noprofile --norc -s -- "$BUILD_KEY" <<'BUILD'
set -e
PREFIX=/workspace/ros_transport
REVISION=41f316772ba1210a79e715d61951ddf50857b640
ARCHIVE_SHA=5ee6997a74a3794275741a647b32ec1921f38bc39ab7c588916a316eee69ad3a
VERSION="$(dpkg-query -W -f='${Version}' ros-jazzy-rmw-zenoh-cpp)"
if [ "${VERSION%%-*}" != 0.2.10 ]; then
  printf 'This transport patch requires rmw_zenoh_cpp 0.2.10, found %s. Review before rebuilding.\n' "$VERSION" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
cd "$PREFIX"
if ! printf '%s  upstream.tar.gz\n' "$ARCHIVE_SHA" | sha256sum -c --status; then
  curl -fL --retry 2 --connect-timeout 10 --max-time 120 \
    "https://codeload.github.com/ros2/rmw_zenoh/tar.gz/$REVISION" -o upstream.tar.gz.part
  printf '%s  upstream.tar.gz.part\n' "$ARCHIVE_SHA" | sha256sum -c
  mv upstream.tar.gz.part upstream.tar.gz
fi
mkdir -p src
tar -xzf upstream.tar.gz --strip-components=1 -C src "rmw_zenoh-$REVISION/rmw_zenoh_cpp"
patch --batch --forward -p1 -d src < readiness.patch
MAKEFLAGS=-j4 colcon build --packages-select rmw_zenoh_cpp \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --allow-overriding rmw_zenoh_cpp
printf '%s\n' "$1" > build-key
printf '%s\n' 'HX5 simulation ROS service transport is ready.'
BUILD
fi

# Build the opt-in simulation-clock recorder in the Isaac-only overlay. Keep the
# image's physical/Gazebo recorder and its timestamp defaults intact.
RECORDER_SOURCE="$PROFILE/../../src/cyclo_intelligence/cyclo_data/recorder/rosbag_recorder"
RECORDER_SHA="$(python3 - "$RECORDER_SOURCE" <<'HASH'
import hashlib
from pathlib import Path
import sys
root = Path(sys.argv[1])
paths = [root / 'CMakeLists.txt', root / 'package.xml']
for directory in ('include', 'src', 'srv', 'msg'):
    paths.extend(path for path in (root / directory).rglob('*') if path.is_file())
digest = hashlib.sha256()
for path in sorted(paths):
    digest.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes())
print(digest.hexdigest())
HASH
)"
RECORDER_KEY="$IMAGE:$RECORDER_SHA"
if docker exec "$CONTAINER" sh -c \
  'test -x /workspace/ros_transport/install/rosbag_recorder/lib/rosbag_recorder/service_bag_recorder && test "$(cat /workspace/ros_transport/recorder-build-key 2>/dev/null)" = "$1"' sh "$RECORDER_KEY"; then
  printf '%s\n' 'Isaac simulation-clock recorder is current.'
  exit 0
fi
docker exec -i "$CONTAINER" bash --noprofile --norc -s -- "$RECORDER_KEY" <<'RECORDER_BUILD'
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros_transport/install/local_setup.bash
cd /workspace/ros_transport
ln -sfn /root/ros2_ws/src/cyclo_intelligence/cyclo_data/recorder/rosbag_recorder src/rosbag_recorder
MAKEFLAGS=-j4 colcon build --packages-select rosbag_recorder \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --allow-overriding rosbag_recorder
printf '%s\n' "$1" > recorder-build-key
printf '%s\n' 'Isaac simulation-clock recorder is ready.'
RECORDER_BUILD
