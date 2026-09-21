#!/usr/bin/env bash
set -euo pipefail
PROFILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=cyclo_intelligence_1044_hx5_sim
PATCH_FILE="$PROFILE/rmw_zenoh_readiness.patch"
IMAGE="$(docker inspect --format '{{.Image}}' "$CONTAINER")"
PATCH_SHA="$(sha256sum "$PATCH_FILE" | cut -d ' ' -f 1)"
BUILD_KEY="$IMAGE:$PATCH_SHA"

if docker exec "$CONTAINER" sh -c \
  'test -f /workspace/ros_transport/install/rmw_zenoh_cpp/lib/librmw_zenoh_cpp.so && test "$(cat /workspace/ros_transport/build-key 2>/dev/null)" = "$1"' sh "$BUILD_KEY"; then
  exit 0
fi

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
