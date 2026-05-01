#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$SCRIPT_DIR

# Create our installation directory

INSTALL_DIR="$HOME/production/crash-imu"

mkdir -p $INSTALL_DIR

# Copy over the needed files

cd $REPO_ROOT
cp *.py \
   README.md \
   requirements.txt \
   setup.sh \
   $INSTALL_DIR

# Now run the setup script

cd $INSTALL_DIR
./setup.sh
