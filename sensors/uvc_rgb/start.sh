#!/usr/bin/env bash
set -eo pipefail

: "${ROS_DISTRO:=humble}"
# Relax nounset only while sourcing ROS to avoid AMENT_TRACE_SETUP_FILES issues
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# ---------- Defaults ----------
: "${CAMERA_DEVICE:=/dev/video0}"
: "${CAM_MODE:=AUTO}"    # AUTO | H264 | MJPEG | YUY2
: "${WIDTH:=1280}"
: "${HEIGHT:=720}"
: "${FPS:=30}"
: "${CAMERA_NAME:=arducam_rgb}"
: "${FRAME_ID:=uvc_rgb_frame}"
: "${IMAGE_TOPIC:=/sensors/uvc/image_raw}"
: "${INFO_TOPIC:=/sensors/uvc/camera_info}"
: "${INFO_URL:=file:///work/config/uvc_rgb_1280x720.yaml}"
export ROS_IMAGE_TRANSPORT="${ROS_IMAGE_TRANSPORT:-raw}"

# Bounded latency
QUEUE_OPTS="queue max-size-buffers=1 max-size-time=$(( 1000000000 / FPS * 3 )) leaky=downstream"

die() { echo "ERROR: $*" >&2; exit 1; }

have_plugin() { gst-inspect-1.0 "$1" >/dev/null 2>&1; }

pick_nvconv() {
  if have_plugin nvvideoconvert; then echo nvvideoconvert; else echo nvvidconv; fi
}

pick_mjpegdec() {
  if have_plugin nvjpegdec; then echo nvjpegdec; else echo jpegdec; fi
}

pick_h264dec() {
  if have_plugin nvv4l2decoder; then echo "nvv4l2decoder enable-max-performance=1"; else echo avdec_h264; fi
}

wait_for_device() {
  local dev="$1" tries=30
  while (( tries-- > 0 )); do
    [[ -e "$dev" ]] && return 0
    echo "[start] waiting for $dev ..."
    sleep 1
  done
  die "Device $dev not present"
}

detect_cam_mode() {
  if v4l2-ctl --device="$CAMERA_DEVICE" --list-formats-ext 2>/dev/null | grep -q "H264"; then
    echo "H264"; return
  fi
  if v4l2-ctl --device="$CAMERA_DEVICE" --list-formats-ext 2>/dev/null | grep -q -E "MJPG|MJPEG"; then
    echo "MJPEG"; return
  fi
  if v4l2-ctl --device="$CAMERA_DEVICE" --list-formats-ext 2>/dev/null | grep -q -E "YUYV|YUY2"; then
    echo "YUY2"; return
  fi
  echo "MJPEG"
}

build_pipeline() {
  local mode="${1^^}"
  local NVCONV; NVCONV="$(pick_nvconv)"
  local MJPEGDEC; MJPEGDEC="$(pick_mjpegdec)"
  local H264DEC;  H264DEC="$(pick_h264dec)"

  case "$mode" in
    H264)
      # v4l2src → h264parse → NV HW decoder → NVMM → NV convert → BGRx → CPU convert → BGR
      cat <<EOF
v4l2src device=${CAMERA_DEVICE} io-mode=2 do-timestamp=true !
video/x-h264,stream-format=byte-stream,alignment=au,framerate=${FPS}/1 !
h264parse ! ${H264DEC} !
${QUEUE_OPTS} !
video/x-raw(memory:NVMM) !
${NVCONV} ! video/x-raw,format=BGRx,framerate=${FPS}/1 !
videoconvert ! video/x-raw,format=BGR,framerate=${FPS}/1
EOF
      ;;
    MJPEG)
      # v4l2src → jpegparse → NV HW jpegdec → NVMM → NV convert → BGRx → CPU convert → BGR
      cat <<EOF
v4l2src device=${CAMERA_DEVICE} io-mode=2 do-timestamp=true !
image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 !
jpegparse ! ${MJPEGDEC} !
${QUEUE_OPTS} !
video/x-raw(memory:NVMM) !
${NVCONV} ! video/x-raw,format=BGRx,framerate=${FPS}/1 !
videoconvert ! video/x-raw,format=BGR,framerate=${FPS}/1
EOF
      ;;
    YUY2)
      # v4l2src → (NV or CPU) convert → BGR
      if have_plugin "$(pick_nvconv)"; then
        cat <<EOF
v4l2src device=${CAMERA_DEVICE} io-mode=2 do-timestamp=true !
video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 !
${QUEUE_OPTS} !
${NVCONV} ! video/x-raw,format=BGRx,framerate=${FPS}/1 !
videoconvert ! video/x-raw,format=BGR,framerate=${FPS}/1
EOF
      else
        cat <<EOF
v4l2src device=${CAMERA_DEVICE} io-mode=2 do-timestamp=true !
video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 !
${QUEUE_OPTS} !
videoconvert ! video/x-raw,format=BGR,framerate=${FPS}/1
EOF
      fi
      ;;
    *)
      die "Unknown CAM_MODE '$CAM_MODE' (use AUTO|H264|MJPEG|YUY2)"
      ;;
  esac
}

wait_for_device "${CAMERA_DEVICE}"

if [[ "${CAM_MODE^^}" == "AUTO" ]]; then
  CAM_MODE="$(detect_cam_mode)"
  echo "[start] AUTO mode selected → ${CAM_MODE}"
fi

# Build pipeline if not provided
if [[ -z "${GSCAM_CONFIG:-}" ]]; then
  echo "[start] Building pipeline for CAM_MODE=${CAM_MODE}"
  export GSCAM_CONFIG
  GSCAM_CONFIG="$(build_pipeline "${CAM_MODE}")"
fi

# Export gscam vars (topics & metadata)
export GSCAM_CAMERA_NAME="${CAMERA_NAME}"
export GSCAM_FRAME_ID="${FRAME_ID}"

echo "================= gscam launch config ================="
echo "DEVICE        : ${CAMERA_DEVICE}"
echo "MODE          : ${CAM_MODE}"
echo "RES/FPS       : ${WIDTH}x${HEIGHT}@${FPS}"
echo "FRAME_ID      : ${FRAME_ID}"
echo "IMAGE_TOPIC   : ${IMAGE_TOPIC}"
echo "INFO_TOPIC    : ${INFO_TOPIC}"
echo "INFO_URL      : ${INFO_URL}"
echo "======================================================="
echo "${GSCAM_CONFIG}" | sed 's/^/[GSCAM] /'
echo "======================================================="

# Nice to reduce jitter
if command -v renice >/dev/null 2>&1; then renice -n -5 $$ >/dev/null 2>&1 || true; fi

# Launch gscam (let gscam create its own appsink)
exec ros2 run gscam gscam_node \
  --ros-args \
  -r /image_raw:="${IMAGE_TOPIC}" \
  -r /camera_info:="${INFO_TOPIC}" \
  -p frame_id:="${FRAME_ID}" \
  -p camera_name:="${CAMERA_NAME}" \
  -p camera_info_url:="${INFO_URL}"
