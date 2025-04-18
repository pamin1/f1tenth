#!/usr/bin/env bash
set -euo pipefail

# ./optmization.sh f1tenth localization_launch.py my_map
if [[ $# -lt 3 ]]; then
  echo "Usage: $0 <ros2_package> <launch_file> <output_map_name>"
  exit 1
fi

PKG="$1"
LAUNCH="$2"
OUTNAME="$3"

# Function that will be called on Ctrl+C
on_sigint() {
  echo -e "\n↪ Caught Ctrl+C — saving map to '${OUTNAME}.pgm'/.yaml…"
  ros2 run nav2_map_server map_saver_cli -f "src/f1tenth/VipPathOptimization/maps/${OUTNAME}"
  echo "✔ Map saved; exiting."
  python3 /home/nvidia/team2TEMP/src/f1tenth/VipPathOptimization/MapTrack.py test
  exit 0
}

# Register the trap for SIGINT (Ctrl+C)
trap on_sigint SIGINT

echo "▶ Launching '${PKG}/${LAUNCH}.py' (Ctrl+C to save)…"
# Run it in the foreground so we can intercept Ctrl+C
ros2 launch "${PKG}" "${LAUNCH}.py"

# If the launch ever exits on its own (e.g. you shut it down another way),
# you can optionally still save here or just exit:
echo "⚠️  Launch exited; saving map just in case…"
ros2 run nav2_map_server map_saver_cli -f "src/f1tenth/VipPathOptimization/maps/${OUTNAME}"
echo "✔ Done."

