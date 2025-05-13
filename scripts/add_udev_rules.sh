#!/bin/bash

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

sudo mkdir -p /etc/udev/rules.d
sudo cp $SCRIPT_DIR/99_grippers.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger
