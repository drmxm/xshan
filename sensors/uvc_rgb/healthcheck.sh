#!/usr/bin/env bash
set -eo pipefail

: "${ROS_DISTRO:=humble}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

: "${IMAGE_TOPIC:=/sensors/uvc/image_raw}"
: "${TIMEOUT_SECS:=8}"     # a touch longer for discovery
: "${WAIT_LIST_SECS:=8}"   # how long to wait for topic to appear

# 0) Node should exist (name contains "gscam")
if ! ros2 node list | grep -q "gscam"; then
  echo "[health] gscam node not found yet"
  exit 1
fi

# 1) Wait briefly for the topic to be listed
end=$((SECONDS + WAIT_LIST_SECS))
while (( SECONDS < end )); do
  if ros2 topic list | grep -qx "${IMAGE_TOPIC}"; then
    found=1; break
  fi
  sleep 0.5
done
if [[ -z "${found:-}" ]]; then
  echo "[health] topic ${IMAGE_TOPIC} not advertised yet"
  exit 1
fi

# 2) Read one message with SensorData QoS (BestEffort/Volatile)
#    Explicit msg type avoids introspection delays.
if ! timeout "${TIMEOUT_SECS}" ros2 topic echo -n 1 "${IMAGE_TOPIC}" sensor_msgs/msg/Image \
      --qos-reliability best_effort \
      --qos-durability volatile \
      --qos-history keep_last \
      --qos-depth 1 >/dev/null 2>&1; then
  echo "[health] no messages received on ${IMAGE_TOPIC} within ${TIMEOUT_SECS}s (QoS=SensorData)"
  exit 1
fi

echo "[health] OK"
exit 0
