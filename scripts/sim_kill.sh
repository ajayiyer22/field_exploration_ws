#!/usr/bin/env bash
# Stop processes left over from earlier runs of this workspace's simulation.
#   ./scripts/sim_kill.sh          stop them
#   ./scripts/sim_kill.sh --list   only show what would be stopped
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SELF=$$

PATTERNS=(
    '(^|/)gz sim'
    'gz-sim(-server|-gui)?'
    'ruby .*/gz sim'
    '/opt/ros/jazzy/lib/ros_gz_bridge/'
    '/opt/ros/jazzy/lib/ros_gz_sim/'
    '/opt/ros/jazzy/lib/robot_state_publisher/'
    '/opt/ros/jazzy/lib/controller_manager/'
    '/opt/ros/jazzy/lib/robot_localization/'
    '/opt/ros/jazzy/lib/twist_mux/'
    '/opt/ros/jazzy/lib/imu_filter_madgwick/'
    '/opt/ros/jazzy/lib/nav2_'
    '/opt/ros/jazzy/lib/rviz2/'
    '/opt/ros/jazzy/lib/clearpath_'
    "${WS}/install/"
    'search_bringup .*\.launch\.py'
    'clearpath_gz robot_spawn\.launch\.py'
    'rmf_demos_gz simulation\.launch\.xml'
)

collect() {
    for pattern in "${PATTERNS[@]}"; do
        pgrep -f -- "$pattern" || true
    done | sort -u | grep -v -x -e "$SELF" -e "$PPID" || true
}

pids="$(collect)"
if [[ -z "$pids" ]]; then
    echo "No simulation processes running."
    exit 0
fi
if [[ "${1:-}" == "--list" ]]; then
    ps -o pid,cmd -p $pids
    exit 0
fi
echo "Stopping $(echo "$pids" | wc -w) processes"
kill -INT $pids 2>/dev/null || true
for _ in $(seq 1 20); do
    sleep 0.25
    remaining="$(collect)"
    [[ -z "$remaining" ]] && break
done
remaining="$(collect)"
if [[ -n "$remaining" ]]; then
    kill -KILL $remaining 2>/dev/null || true
fi
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi
