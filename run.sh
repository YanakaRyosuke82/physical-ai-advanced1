#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source /home/ubuntu/pai_ws/install/setup.bash
exec python3 main.py "$@"
