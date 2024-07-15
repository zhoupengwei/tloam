#!/bin/bash
set -e

Xvfb :99 -screen 0 1024x768x24 &
export DISPLAY=:99

# setup tloam environment
source "/tloam_ws/devel/setup.bash" --
exec "$@"