#!/bin/bash
set -e

echo "[1/5] Installing apt packages..."
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

echo "[2/5] Installing Python packages..."
pip3 install --user --break-system-packages mcap || pip3 install --user mcap

echo "[3/5] Enabling I2C..."
sudo raspi-config nonint do_i2c 0

echo "[4/5] Enabling gpsd..."
sudo systemctl enable --now gpsd

echo "[5/5] Installing crash-detector systemd service..."
SERVICE_SRC="$(dirname "$(readlink -f "$0")")/crash-detector.service"
sed -e "s|__WORKDIR__|$(pwd)|g" -e "s|__USER__|$USER|g" "$SERVICE_SRC" \
    | sudo tee /etc/systemd/system/crash-detector.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now crash-detector.service

echo
echo "Setup complete."
echo "Service status: sudo systemctl status crash-detector"
echo "Run manually:   python3 segway_behavior_classifier.py"
