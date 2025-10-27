#!/bin/bash
# Wrapper script to set LD_LIBRARY_PATH before running the controller

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set the AprilTag library path
export LD_LIBRARY_PATH="${SCRIPT_DIR}/../../../apriltag/build:${LD_LIBRARY_PATH}"

# Run the Python controller
exec python3 -u "${SCRIPT_DIR}/cswd_controller.py" "$@"
