#!/usr/bin/env bash
set -eo pipefail

: "${ROS_DISTRO:=humble}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# Topics (override via compose if needed)
: "${IMAGE_TOPIC:=/camera/image_raw}"          # default matches what your camera actually publishes
: "${ANNOTATED_TOPIC:=/perception/image_annotated}"
: "${DETECTIONS_TOPIC:=/perception/detections}"

# Use the same RMW everywhere
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION

exec python3 /detector_node.py
