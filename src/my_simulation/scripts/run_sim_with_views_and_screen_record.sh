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
CAMERA_STREAM_SETTLE_SEC=2
FFMPEG_FRAME_RATE=25
MANUAL_START_DELAY_SEC=5

mkdir -p "${LOCAL_RECORD_DIR}"
mkdir -p "${EXPORT_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
STRATEGY="${2:-segment}"
SCENE_KEY="${3:-demo4}"
OUTPUT_PREFIX="${1:-${SCENE_KEY}_${STRATEGY}_pickup_screen_${TIMESTAMP}}"
SESSION_LOCAL_DIR="${LOCAL_RECORD_DIR}/${OUTPUT_PREFIX}"
SESSION_EXPORT_DIR="${EXPORT_DIR}/${OUTPUT_PREFIX}"
RAW_FILE="${SESSION_LOCAL_DIR}/screen.mkv"
FINAL_FILE="${SESSION_EXPORT_DIR}/screen.mp4"
RECORD_LOG_FILE="${SESSION_LOCAL_DIR}/screen_record.log"
CMD_VEL_FILE="${SESSION_EXPORT_DIR}/cmd_vel.csv"
ODOM_FILE="${SESSION_EXPORT_DIR}/odom.csv"

source "${ROS_SETUP}"
source "${CATKIN_SETUP}"

mkdir -p "${SESSION_LOCAL_DIR}"
mkdir -p "${SESSION_EXPORT_DIR}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg 未安装，无法录屏。"
  exit 1
fi

if ! command -v ffprobe >/dev/null 2>&1; then
  echo "ffprobe 未安装，无法校验录屏文件。"
  exit 1
fi

if ! command -v xprop >/dev/null 2>&1; then
  echo "xprop 未安装，无法按进程定位窗口。"
  exit 1
fi

if ! command -v xrandr >/dev/null 2>&1; then
  echo "xrandr 未安装，无法获取屏幕分辨率。"
  exit 1
fi

if ! command -v rosrun >/dev/null 2>&1; then
  echo "rosrun 不可用，无法启动 image_view。"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY 未设置，无法打开图形界面或录屏。"
  exit 1
fi

if [[ "${STRATEGY}" != "point" && "${STRATEGY}" != "segment" ]]; then
  echo "策略参数无效: ${STRATEGY}，只支持 point 或 segment。"
  exit 1
fi

resolve_scene_file() {
  case "${SCENE_KEY}" in
    demo4)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/demo_4.yaml"
      ;;
    sparse10)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_10_sparse.yaml"
      ;;
    half15)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_15_half_random.yaml"
      ;;
    turncmp5)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_turn_compare_5_random.yaml"
      ;;
    turncmp)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_turn_compare.yaml"
      ;;
    cluster15)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_15_clustered_half.yaml"
      ;;
    cluster30)
      echo "${WORKSPACE_ROOT}/src/my_simulation/config/scenes/scene_30_clustered.yaml"
      ;;
    *)
      if [[ -f "${SCENE_KEY}" ]]; then
        echo "${SCENE_KEY}"
      else
        return 1
      fi
      ;;
  esac
}

SCENE_FILE="$(resolve_scene_file)" || {
  echo "场景参数无效: ${SCENE_KEY}，支持 demo4、sparse10、half15、turncmp5、turncmp、cluster15、cluster30，或直接传入 scene yaml 路径。"
  exit 1
}

ROS_PIDS=()
VIEW_PIDS=()
FFMPEG_PID=""
LAST_VIEW_WIN_ID=""
LAST_VIEW_PID=""

list_client_window_ids() {
  xprop -root _NET_CLIENT_LIST 2>/dev/null | grep -o '0x[0-9a-f]\+'
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

call_trigger_rosservice() {
  local service_name="$1"
  rosservice call "${service_name}" >/dev/null
}

wait_for_true_bool_topic() {
  local topic="$1"
  while true; do
    local output
    output="$(rostopic echo -n 1 "${topic}" 2>/dev/null || true)"
    if grep -Eq 'data: *(true|True|1)$' <<< "${output}"; then
      return 0
    fi
    sleep 1
  done
}

start_topic_recorder() {
  local topic="$1"
  local output_file="$2"

  rostopic echo -p "${topic}" >"${output_file}" 2>"${output_file}.log" &
  local recorder_pid=$!
  ROS_PIDS+=("${recorder_pid}")
  sleep 1

  if ! kill -0 "${recorder_pid}" >/dev/null 2>&1; then
    echo "话题记录进程启动失败: ${topic}" >&2
    cat "${output_file}.log" >&2 || true
    return 1
  fi
}

get_screen_geometry() {
  local geometry
  geometry="$(xrandr --current 2>/dev/null | awk '/\*/ {print $1; exit}')"
  if [[ -z "${geometry}" ]]; then
    echo "无法获取当前屏幕分辨率。" >&2
    return 1
  fi
  echo "${geometry}"
}

start_screen_recording() {
  local geometry="$1"

  ffmpeg -y \
    -nostdin \
    -loglevel info \
    -f x11grab \
    -framerate "${FFMPEG_FRAME_RATE}" \
    -video_size "${geometry}" \
    -i "${DISPLAY}.0+0,0" \
    -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" \
    -c:v libx264 \
    -crf 23 \
    -preset veryfast \
    -pix_fmt yuv420p \
    -an \
    "${RAW_FILE}" >"${RECORD_LOG_FILE}" 2>&1 &

  FFMPEG_PID=$!
  sleep 1

  if ! kill -0 "${FFMPEG_PID}" >/dev/null 2>&1; then
    echo "整屏录制进程启动失败。" >&2
    cat "${RECORD_LOG_FILE}" >&2 || true
    return 1
  fi
}

stop_ffmpeg_cleanly() {
  if [[ -n "${FFMPEG_PID}" ]] && kill -0 "${FFMPEG_PID}" >/dev/null 2>&1; then
    kill -INT "${FFMPEG_PID}" >/dev/null 2>&1 || true
    wait "${FFMPEG_PID}" >/dev/null 2>&1 || true
  fi
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

export_video_if_valid() {
  if [[ ! -s "${RAW_FILE}" ]]; then
    echo "录屏文件为空，跳过导出: ${RAW_FILE}" >&2
    [[ -f "${RECORD_LOG_FILE}" ]] && cat "${RECORD_LOG_FILE}" >&2 || true
    return
  fi

  if ! ffprobe -v error -show_entries format=format_name -of default=nw=1:nk=1 "${RAW_FILE}" >/dev/null 2>&1; then
    echo "录屏文件无效，跳过导出: ${RAW_FILE}" >&2
    [[ -f "${RECORD_LOG_FILE}" ]] && cat "${RECORD_LOG_FILE}" >&2 || true
    return
  fi

  ffmpeg -y \
    -i "${RAW_FILE}" \
    -c copy \
    -movflags +faststart \
    "${FINAL_FILE}" >"${SESSION_LOCAL_DIR}/screen_remux.log" 2>&1 || {
      echo "导出 mp4 失败: ${FINAL_FILE}" >&2
      cat "${SESSION_LOCAL_DIR}/screen_remux.log" >&2 || true
      return
    }

  echo "导出的视频文件: ${FINAL_FILE}"
}

cleanup() {
  local exit_code=$?
  echo
  echo "正在停止仿真、图像窗口和录屏..."

  stop_ffmpeg_cleanly
  stop_process_list VIEW_PIDS
  stop_process_list ROS_PIDS
  export_video_if_valid

  exit ${exit_code}
}

trap cleanup EXIT INT TERM

echo "启动 Gazebo 仿真（先暂停物理引擎）..."
roslaunch my_simulation spawn_car.launch \
  paused:=false \
  wait_for_manual_start:=true \
  startup_delay_sec:=0.0 \
  artifacts_output_dir:="${SESSION_EXPORT_DIR}" \
  strategy:="${STRATEGY}" \
  scene_file:="${SCENE_FILE}" &
ROS_PIDS+=($!)

echo "等待仿真节点与调试图像话题就绪..."
sleep "${SIM_LAUNCH_WAIT_SEC}"
wait_for_topic "/stereo/debug_image" "${WINDOW_WAIT_TIMEOUT_SEC}"
wait_for_service "/ball_pickup_controller/start" "${WINDOW_WAIT_TIMEOUT_SEC}"
wait_for_topic "/ball_pickup_controller/pickup_complete" "${WINDOW_WAIT_TIMEOUT_SEC}"
sleep "${CAMERA_STREAM_SETTLE_SEC}"

echo "打开左右拼接调试窗口..."
launch_viewer "/stereo/debug_image" "stereo_debug"

SCREEN_GEOMETRY="$(get_screen_geometry)"

echo "窗口已准备好。"
echo "请先手动摆好 Gazebo 和拼接调试窗口。"
echo "准备开始时，按回车键：脚本会先倒计时 ${MANUAL_START_DELAY_SEC} 秒，再同时开始整屏录制和捡球。"
read -r

echo "开始倒计时..."
for remaining in $(seq "${MANUAL_START_DELAY_SEC}" -1 1); do
  echo "  ${remaining}"
  sleep 1
done

echo "开始整屏录制..."
start_screen_recording "${SCREEN_GEOMETRY}"

echo "开始记录转向与里程信息..."
start_topic_recorder "/cmd_vel" "${CMD_VEL_FILE}"
start_topic_recorder "/odom" "${ODOM_FILE}"

echo "发送开始捡球信号。"
call_trigger_rosservice "/ball_pickup_controller/start"

echo "录屏已开始，小车已开始捡球。按 Ctrl+C 可停止并自动导出 mp4。"
echo "当前策略: ${STRATEGY}"
echo "当前场景: ${SCENE_KEY}"
echo "结果目录: ${SESSION_EXPORT_DIR}"
echo "输出文件前缀: ${OUTPUT_PREFIX}"
echo "转向命令记录: ${CMD_VEL_FILE}"
echo "里程姿态记录: ${ODOM_FILE}"

echo "等待最后一个球完成拾取..."
wait_for_true_bool_topic "/ball_pickup_controller/pickup_complete"
echo "检测到最后一个球已拾取，正在结束录制并导出结果..."
