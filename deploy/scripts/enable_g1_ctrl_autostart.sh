#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/deploy/robots/g1_29dof/build"
G1_CTRL="${BUILD_DIR}/g1_ctrl"
SERVICE_NAME="unitree-g1-ctrl-autostart.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
JOYSTICK_ARGS="--custom-joystick"

if [[ ! -x "${G1_CTRL}" ]]; then
  echo "Error: ${G1_CTRL} does not exist or is not executable. Build g1_ctrl first."
  exit 1
fi

if [[ -n "${JOYSTICK_DEVICE}" ]]; then
  JOYSTICK_ARGS="--custom-joystick ${JOYSTICK_DEVICE}"
fi

if [[ "${EUID}" -ne 0 ]]; then
  SUDO=sudo
else
  SUDO=
fi

${SUDO} tee "${SERVICE_PATH}" >/dev/null <<EOF
[Unit]
Description=Unitree G1 controller autostart
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
WorkingDirectory=${BUILD_DIR}
ExecStart=${G1_CTRL} --network ${NETWORK_INTERFACE} ${JOYSTICK_ARGS}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

${SUDO} systemctl daemon-reload
${SUDO} systemctl enable --now "${SERVICE_NAME}"

echo "Enabled ${SERVICE_NAME}"
echo "Edit ${SERVICE_PATH} to change the network interface or joystick device, then run this script again."
