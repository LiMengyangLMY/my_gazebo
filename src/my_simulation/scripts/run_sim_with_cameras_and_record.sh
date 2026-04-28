#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_ROOT="/home/lmy/catkin_ws"
ROS_SETUP="/opt/ros/noetic/setup.bash"
CATKIN_SETUP="${WORKSPACE_ROOT}/devel/setup.bash"
LOCAL_RECORD_DIR="${WORKSPACE_ROOT}/recordings_tmp"
EXPORT_DIR="/mnt/hgfs/sharedFolder"
SIM_LAUNCH_WAIT_SEC=10
VIEW_LAUNCH_GAP_SEC=2
WINDOW_WAIT_TIMEOUT_SEC=30
RECORD_START_DELAY_SEC=3
FFMPEG_FRAME_RATE=25
CAMERA_WARMUP_SEC=1
POST_WARMUP_SETTLE_SEC=1

mkdir -p "${LOCAL_RECORD_DIR}"
mkdir -p "${EXPORT_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PREFIX="${1:-tennis_pickup_demo_${TIMESTAMP}}"
LOCAL_PREFIX="${LOCAL_RECORD_DIR}/${OUTPUT_PREFIX}"
EXPORT_PREFIX="${EXPORT_DIR}/${OUTPUT_PREFIX}"

source "${ROS_SETUP}"
source "${CATKIN_SETUP}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg 未安装，无法自动录像。"
  exit 1
fi

if ! command -v xwininfo >/dev/null 2>&1; then
  echo "xwininfo 未安装，无法定位窗口。"
  exit 1
fi

if ! command -v xprop >/dev/null 2>&1; then
  echo "xprop 未安装，无法按进程定位窗口。"
  exit 1
fi

if ! command -v rosrun >/dev/null 2>&1; then
  echo "rosrun 不可用，无法启动 image_view。"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY 未设置，无法打开图像窗口或录制桌面。"
  exit 1
fi

ROS_PIDS=()
VIEW_PIDS=()
FFMPEG_PIDS=()
FFMPEG_RAW_FILES=()
FFMPEG_LOG_FILES=()
FFMPEG_TAGS=()
FINAL_FILES=()
LAST_VIEW_PID=""
LAST_VIEW_WIN_ID=""

list_client_window_ids() {
  xprop -root _NET_CLIENT_LIST 2>/dev/null | grep -o '0x[0-9a-f]\+'
}

find_window_id_by_pattern() {
  local pattern="$1"
  xwininfo -root -tree | grep -F "${pattern}" | head -n 1 | awk '{print $1}'
}

wait_for_window_by_pattern() {
  local pattern="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    local window_id
    window_id="$(find_window_id_by_pattern "${pattern}" || true)"
    if [[ -n "${window_id}" ]]; then
      echo "${window_id}"
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "未能在 ${timeout_sec}s 内找到窗口: ${pattern}" >&2
      xwininfo -root -tree | tail -n 50 >&2 || true
      return 1
    fi
    sleep 1
  done
}

window_pid_matches() {
  local window_id="$1"
  local target_pid="$2"
  local pid_text
  pid_text="$(xprop -id "${window_id}" _NET_WM_PID 2>/dev/null || true)"
  [[ -n "${pid_text}" ]] && grep -Eq "= ${target_pid}$" <<< "${pid_text}"
}

wait_for_window_by_pid() {
  local target_pid="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    while IFS= read -r window_id; do
      [[ -z "${window_id}" ]] && continue
      if window_pid_matches "${window_id}" "${target_pid}"; then
        echo "${window_id}"
        return 0
      fi
    done < <(list_client_window_ids)

    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "未能在 ${timeout_sec}s 内识别到 PID=${target_pid} 对应的窗口。" >&2
      xwininfo -root -tree | tail -n 80 >&2 || true
      return 1
    fi
    sleep 1
  done
}

launch_viewer() {
  local topic="$1"
  local role="$2"
  local node_name="${role}_camera_view"
  local window_name="${role}_camera_window"

  rosrun image_view image_view \
    image:="${topic}" \
    __name:="${node_name}" \
    _autosize:=true \
    _window_name:="${window_name}" >/tmp/image_view_${role}.log 2>&1 &
  local viewer_pid=$!
  VIEW_PIDS+=("${viewer_pid}")
  sleep 1

  if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
    echo "${role} 图像窗口进程启动失败。" >&2
    cat /tmp/image_view_${role}.log >&2 || true
    return 1
  fi

  LAST_VIEW_PID="${viewer_pid}"
  LAST_VIEW_WIN_ID="$(wait_for_window_by_pid "${viewer_pid}" "${WINDOW_WAIT_TIMEOUT_SEC}")"
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if rostopic list 2>/dev/null | grep -Fx "${topic}" >/dev/null 2>&1; then
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "未能在 ${timeout_sec}s 内等到话题: ${topic}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if rosservice list 2>/dev/null | grep -Fx "${service_name}" >/dev/null 2>&1; then
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "未能在 ${timeout_sec}s 内等到服务: ${service_name}" >&2
      return 1
    fi
    sleep 1
  done
}

call_rosservice() {
  local service_name="$1"
  rosservice call "${service_name}" "{}" >/dev/null
}

schedule_window_recording() {
  local window_id="$1"
  local raw_file="$2"
  local start_delay="$3"
  local tag="$4"
  local log_file="${LOCAL_PREFIX}_${tag}_record.log"

  (
    sleep "${start_delay}"
    exec ffmpeg -y \
      -nostdin \
      -loglevel info \
      -f x11grab \
      -framerate "${FFMPEG_FRAME_RATE}" \
      -window_id "${window_id}" \
      -i "${DISPLAY}" \
      -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" \
      -c:v libx264 \
      -crf 23 \
      -preset veryfast \
      -pix_fmt yuv420p \
      -an \
      "${raw_file}" >"${log_file}" 2>&1
  ) &

  FFMPEG_PIDS+=($!)
  FFMPEG_RAW_FILES+=("${raw_file}")
  FFMPEG_LOG_FILES+=("${log_file}")
  FFMPEG_TAGS+=("${tag}")
}

check_recorders_alive() {
  local failed=0

  for idx in "${!FFMPEG_PIDS[@]}"; do
    local pid="${FFMPEG_PIDS[$idx]}"
    local tag="${FFMPEG_TAGS[$idx]}"
    local log_file="${FFMPEG_LOG_FILES[$idx]}"

    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      echo "录像进程提前退出: ${tag}" >&2
      if [[ -f "${log_file}" ]]; then
        cat "${log_file}" >&2 || true
      fi
      failed=1
    fi
  done

  return "${failed}"
}

media_file_valid() {
  local file_path="$1"

  if [[ ! -s "${file_path}" ]]; then
    return 1
  fi

  if command -v ffprobe >/dev/null 2>&1; then
    ffprobe -v error -show_entries format=format_name -of default=nw=1:nk=1 "${file_path}" >/dev/null 2>&1
    return $?
  fi

  return 0
}

stop_ffmpeg_cleanly() {
  for pid in "${FFMPEG_PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
    fi
  done
  for pid in "${FFMPEG_PIDS[@]}"; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}

stop_process_list() {
  local -n ref="$1"
  for pid in "${ref[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -INT "${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  for pid in "${ref[@]}"; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}

remux_and_export() {
  local suffixes=("gazebo" "left" "right")
  for suffix in "${suffixes[@]}"; do
    local raw_file="${LOCAL_PREFIX}_${suffix}.mkv"
    local final_file="${EXPORT_PREFIX}_${suffix}.mp4"
    local log_file="${LOCAL_PREFIX}_${suffix}_record.log"

    if [[ -f "${raw_file}" ]]; then
      if ! media_file_valid "${raw_file}"; then
        echo "跳过无效录像文件: ${raw_file}" >&2
        if [[ -f "${log_file}" ]]; then
          echo "对应录制日志: ${log_file}" >&2
          cat "${log_file}" >&2 || true
        fi
        continue
      fi

      ffmpeg -y \
        -i "${raw_file}" \
        -c copy \
        -movflags +faststart \
        "${final_file}" >/tmp/"${OUTPUT_PREFIX}_${suffix}_remux.log" 2>&1 || {
          echo "转换失败: ${raw_file} -> ${final_file}" >&2
          cat /tmp/"${OUTPUT_PREFIX}_${suffix}_remux.log" >&2 || true
          continue
        }
      FINAL_FILES+=("${final_file}")
    fi
  done
}

cleanup() {
  local exit_code=$?
  echo
  echo "正在停止仿真、图像窗口和录像..."

  stop_ffmpeg_cleanly
  stop_process_list VIEW_PIDS
  stop_process_list ROS_PIDS

  remux_and_export

  if [[ ${#FINAL_FILES[@]} -gt 0 ]]; then
    echo "导出的视频文件："
    for file in "${FINAL_FILES[@]}"; do
      echo "  ${file}"
    done
  fi

  exit ${exit_code}
}

trap cleanup EXIT INT TERM

echo "启动 Gazebo 仿真（先暂停物理引擎）..."
roslaunch my_simulation spawn_car.launch paused:=true &
ROS_PIDS+=($!)

echo "等待仿真节点与双目话题就绪..."
sleep "${SIM_LAUNCH_WAIT_SEC}"
wait_for_service "/gazebo/unpause_physics" "${WINDOW_WAIT_TIMEOUT_SEC}"
wait_for_service "/gazebo/pause_physics" "${WINDOW_WAIT_TIMEOUT_SEC}"
wait_for_topic "/stereo_camera/left/image_raw" "${WINDOW_WAIT_TIMEOUT_SEC}"
wait_for_topic "/stereo_camera/right/image_raw" "${WINDOW_WAIT_TIMEOUT_SEC}"

echo "预热双目相机，先短暂恢复仿真以产生图像帧..."
call_rosservice "/gazebo/unpause_physics"
sleep "${CAMERA_WARMUP_SEC}"
call_rosservice "/gazebo/pause_physics"
sleep "${POST_WARMUP_SETTLE_SEC}"

echo "打开左目图像窗口..."
launch_viewer "/stereo_camera/left/image_raw" "left"
LEFT_WIN_ID="${LAST_VIEW_WIN_ID}"

sleep "${VIEW_LAUNCH_GAP_SEC}"

echo "打开右目图像窗口..."
launch_viewer "/stereo_camera/right/image_raw" "right"
RIGHT_WIN_ID="${LAST_VIEW_WIN_ID}"

sleep "${VIEW_LAUNCH_GAP_SEC}"

echo "等待窗口出现..."
GAZEBO_WIN_ID="$(find_window_id_by_pattern "gzclient" || true)"
if [[ -z "${GAZEBO_WIN_ID}" ]]; then
  GAZEBO_WIN_ID="$(wait_for_window_by_pattern "Gazebo" "${WINDOW_WAIT_TIMEOUT_SEC}")"
fi

echo "三个窗口已就绪，${RECORD_START_DELAY_SEC}s 后同步开始录像..."
schedule_window_recording "${GAZEBO_WIN_ID}" "${LOCAL_PREFIX}_gazebo.mkv" "${RECORD_START_DELAY_SEC}" "gazebo"
schedule_window_recording "${LEFT_WIN_ID}" "${LOCAL_PREFIX}_left.mkv" "${RECORD_START_DELAY_SEC}" "left"
schedule_window_recording "${RIGHT_WIN_ID}" "${LOCAL_PREFIX}_right.mkv" "${RECORD_START_DELAY_SEC}" "right"

sleep $((RECORD_START_DELAY_SEC + 1))

if ! check_recorders_alive; then
  echo "至少有一路录像没有成功启动，终止本次任务。" >&2
  exit 1
fi

echo "开始录像，恢复仿真物理引擎..."
call_rosservice "/gazebo/unpause_physics"

echo "系统已启动。"
echo "录像已开始，小车会在恢复仿真后再等待 3 秒开始运动。"
echo "按 Ctrl+C 可同时停止仿真并结束录像。"
echo "导出目录: ${EXPORT_DIR}"
echo "输出前缀: ${OUTPUT_PREFIX}"

wait
