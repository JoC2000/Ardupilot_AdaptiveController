#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

#Initialize submodule
git -C "$ROOT_DIR" submodule update --init --recursive

#Apply custom controller files
cp -r "$ROOT_DIR/src/"* "$ROOT_DIR/ardupilot/"
echo "Custom controller files applied to ardupilot"

#Install ardupilot dependencies
cd "$ROOT_DIR/ardupilot"
./Tools/environment_install/install-prereqs-ubuntu.sh -y

echo "Dependencies installed. Reload your shell before building:"
echo "source ~/.profile"