#!/bin/bash
set -e

echo "[1/4] Installing apt packages..."
sudo apt update
sudo apt install -y \
    python3-smbus \
    python3-smbus2 \
    python3-gps \
    python3-matplotlib \
    python3-pip \
    gpsd \
    gpsd-clients \
    alsa-utils

echo "[2/4] Installing Python packages..."
pip3 install --user --break-system-packages mcap || pip3 install --user mcap

echo "[3/4] Enabling I2C..."
sudo raspi-config nonint do_i2c 0

echo "[4/4] Enabling gpsd..."
sudo systemctl enable --now gpsd

echo
echo "Setup complete."
echo "Run: python3 segway_behavior_classifier.py"
