#!/usr/bin/env bash
# Play all recorded rosbag episodes sequentially for RViz2 inspection.
# Usage: ./play_all_bags.sh [--dir <rosbags-dir>] [--rate <rate>] [--loop]
#                          [--topics <topic1> <topic2> ...]
#
# --dir defaults to the sibling rosbags/ dir. Point it elsewhere when
# the episodes live on another disk, e.g. the merged pnp_bottle_bin tree:
#   ./play_all_bags.sh --dir /media/$USER/<uuid>/pnp_bottle_bin_all
# The directory must contain recorded_bags_meta.yaml plus the session subdirs.
#
# Controls while a bag is playing:
#   Space  - pause / resume
#   s      - step one frame (when paused)
#   q      - quit current bag and move to next
#   Ctrl+C - abort everything

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Script lives in scripts/; default output root is the sibling rosbags/ dir.
ROSBAGS_DIR="$(cd "$SCRIPT_DIR/../rosbags" && pwd)"

# ── defaults ──────────────────────────────────────────────────────────────────
RATE=1.0
LOOP=false
# Default 1000 covers rate 1-5; raise it when high rates starve the reader.
QUEUE_SIZE=1000
EXTRA_ARGS=()

# ── argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dir)    ROSBAGS_DIR="$2"; shift 2 ;;
        --rate)   RATE="$2";  shift 2 ;;
        --queue)  QUEUE_SIZE="$2"; shift 2 ;;
        --loop)   LOOP=true;  shift   ;;
        --topics) shift
                  TOPICS_LIST=()
                  while [[ $# -gt 0 && "$1" != --* ]]; do
                      TOPICS_LIST+=("$1"); shift
                  done
                  # ros2 bag play's --topics is nargs='+': one flag, all topics.
                  if [[ ${#TOPICS_LIST[@]} -gt 0 ]]; then
                      EXTRA_ARGS+=(--topics "${TOPICS_LIST[@]}")
                  fi ;;
        -h|--help)
            echo "Usage: $0 [--dir <rosbags-dir>] [--rate <rate>] [--loop]" \
                 "[--queue <size>] [--topics <t1> <t2> ...]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

META_FILE="$ROSBAGS_DIR/recorded_bags_meta.yaml"

if [[ ! -d "$ROSBAGS_DIR" ]]; then
    echo "[ERROR] Rosbags directory not found: $ROSBAGS_DIR" >&2
    exit 1
fi
if [[ ! -f "$META_FILE" ]]; then
    echo "[ERROR] recorded_bags_meta.yaml not found in: $ROSBAGS_DIR" >&2
    echo "        Pass --dir <path> to point at the tree holding the episodes." >&2
    exit 1
fi

# ── ROS environment ───────────────────────────────────────────────────────────
# Source ROS 2 if not already sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
    for candidate in /opt/ros/*/setup.bash; do
        # shellcheck disable=SC1090
        source "$candidate" && break
    done
fi

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ── collect episodes in order ─────────────────────────────────────────────────
# Parse tasks_list order from YAML (preserves the order written in the file).
# Task-name agnostic: reads the consecutive "- <dir>" entries of the
# tasks_list block, whatever the task is called.
mapfile -t TASK_DIRS < <(
    awk '
        /^[[:space:]]+tasks_list:/ { in_list = 1; next }
        in_list && /^[[:space:]]+-[[:space:]]/ { sub(/.*-[[:space:]]*/, ""); print; next }
        in_list { exit }
    ' "$META_FILE"
)

declare -a EPISODES=()
for task in "${TASK_DIRS[@]}"; do
    task_path="$ROSBAGS_DIR/$task"
    if [[ ! -d "$task_path" ]]; then
        echo "[WARN] Task directory not found, skipping: $task_path"
        continue
    fi
    # Read episode list for this task from metadata
    in_task=false
    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]+${task}: ]]; then
            in_task=true; continue
        fi
        if $in_task; then
            # Stop at the next task key. Keys may start with a digit when
            # sessions are merged (e.g. "01/throw_the_..."), not just a letter.
            if [[ "$line" =~ ^[[:space:]]{4}[a-z0-9] && ! "$line" =~ episode ]]; then
                in_task=false; continue
            fi
            # Inline flow style: episodes_list: [episode_a, episode_b, ...]
            if [[ "$line" =~ episodes_list:[[:space:]]*\[([^]]*)\] ]]; then
                flow="${BASH_REMATCH[1]//,/ }"
                for ep in $flow; do
                    [[ "$ep" =~ ^episode_[0-9_]+$ ]] || continue
                    ep_path="$task_path/$ep"
                    mcap_file=$(find "$ep_path" -maxdepth 1 -name "*.mcap" 2>/dev/null | head -1)
                    if [[ -n "$mcap_file" ]]; then
                        EPISODES+=("$mcap_file")
                    else
                        echo "[WARN] No .mcap found in: $ep_path"
                    fi
                done
                continue
            fi
            # Block style: one "- episode_..." per line
            if [[ "$line" =~ -[[:space:]]+(episode_[0-9_]+) ]]; then
                ep="${BASH_REMATCH[1]}"
                ep_path="$task_path/$ep"
                mcap_file=$(find "$ep_path" -maxdepth 1 -name "*.mcap" 2>/dev/null | head -1)
                if [[ -n "$mcap_file" ]]; then
                    EPISODES+=("$mcap_file")
                else
                    echo "[WARN] No .mcap found in: $ep_path"
                fi
            fi
        fi
    done < "$META_FILE"
done

TOTAL=${#EPISODES[@]}
if [[ $TOTAL -eq 0 ]]; then
    echo "[ERROR] No .mcap episodes found under $ROSBAGS_DIR"
    exit 1
fi

# ── pretty header ─────────────────────────────────────────────────────────────
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          ROS 2 Rosbag Playback — SOBIT HOME                 ║"
echo "╠══════════════════════════════════════════════════════════════╣"
printf "║  Episodes found : %-42s ║\n" "$TOTAL"
printf "║  Rosbags dir    : %-42s ║\n" "$(basename "$ROSBAGS_DIR")"
printf "║  Playback rate  : %-42s ║\n" "$RATE"
printf "║  Queue size     : %-42s ║\n" "$QUEUE_SIZE"
printf "║  Loop mode      : %-42s ║\n" "$LOOP"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Controls: [Space] pause  [s] step  [q] next bag  [^C] quit ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ── trap for clean exit ───────────────────────────────────────────────────────
ABORT=false
cleanup() {
    echo -e "\n[INFO] Aborting playback."
    ABORT=true
}
trap cleanup INT TERM

# ── playback loop ─────────────────────────────────────────────────────────────
round=1
while true; do
    [[ $LOOP == true ]] && echo "[INFO] === Loop round $round ==="

    for i in "${!EPISODES[@]}"; do
        $ABORT && exit 0

        mcap="${EPISODES[$i]}"
        ep_name="$(basename "$(dirname "$mcap")")"
        task_name="$(basename "$(dirname "$(dirname "$mcap")")")"
        num=$((i + 1))

        echo "──────────────────────────────────────────────────────────────"
        echo "[${num}/${TOTAL}] Task   : $task_name"
        echo "         Episode: $ep_name"
        echo "         File   : $(basename "$mcap")"
        echo ""

        # Run in foreground so ros2 bag play receives stdin and keyboard controls work.
        # Ctrl+C sends SIGINT to the whole process group; ros2 bag play exits, then
        # the loop continues to the next episode (ABORT flag exits if pressed again).
        ros2 bag play "$mcap" \
            --rate "$RATE" \
            --read-ahead-queue-size "$QUEUE_SIZE" \
            "${EXTRA_ARGS[@]}" || true

        # Brief pause between episodes so RViz2 can settle
        sleep 1
    done

    [[ $LOOP == false ]] && break
    ((round++))
done

echo ""
echo "[INFO] All $TOTAL episodes played. Done."
