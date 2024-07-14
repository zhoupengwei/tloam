#!/bin/bash
set -e

# setup tloam environment
source "/tloam_ws/devel/setup.bash" --
exec "$@"