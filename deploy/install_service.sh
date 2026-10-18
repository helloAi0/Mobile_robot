#!/usr/bin/env bash
# Installs the systemd service and the udev rules.   sudo bash deploy/install_service.sh [workspace]   (default ~/dev_ws)
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
USER_NAME="${SUDO_USER:-$USER}"
WS="${1:-/home/$USER_NAME/dev_ws}"
HERE="$(cd "$(dirname "$0")" && pwd)"
sed -e "s#@USER@#$USER_NAME#" -e "s#@WS@#$WS#" "$HERE/mobile_robot.service" > /etc/systemd/system/mobile_robot.service
[ -f /etc/default/mobile_robot ] || cp "$HERE/mobile_robot.env" /etc/default/mobile_robot
cp "$HERE/99-mobile-robot.rules" /etc/udev/rules.d/99-mobile-robot.rules
usermod -aG dialout "$USER_NAME"
udevadm control --reload; udevadm trigger
systemctl daemon-reload
echo "Installed. Edit /etc/default/mobile_robot and deploy/99-mobile-robot.rules for your hardware, then:"
echo "  sudo systemctl enable --now mobile_robot ; journalctl -u mobile_robot -f"
