#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: enable_g1_ctrl_autostart.sh [--delay-seconds N] [--joystick-type TYPE]

Options:
  --delay-seconds N  Wait N seconds before starting g1_ctrl (default: 0)
  --joystick-type T   Custom joystick mapping: xbox, ps5, beitong20 (default: xbox)
  -h, --help         Show this help message

Environment variables:
  NETWORK_INTERFACE   Network interface to use (default: eth0)
  JOYSTICK_DEVICE     Optional joystick device path
  JOYSTICK_TYPE       Custom joystick mapping: xbox, ps5, beitong20
EOF
}

STARTUP_DELAY_SECONDS="${STARTUP_DELAY_SECONDS:-0}"
JOYSTICK_TYPE="${JOYSTICK_TYPE:-beitong20}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/deploy/robots/g1_29dof/build"
G1_CTRL="${BUILD_DIR}/g1_ctrl"
SERVICE_NAME="unitree-g1-ctrl-autostart.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
JOYSTICK_ARGS="--custom-joystick"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --delay-seconds)
      if [[ $# -lt 2 ]]; then
        echo "Error: --delay-seconds requires a value."
        usage
        exit 1
      fi
      STARTUP_DELAY_SECONDS="$2"
      shift 2
      ;;
    --joystick-type)
      if [[ $# -lt 2 ]]; then
        echo "Error: --joystick-type requires a value."
        usage
        exit 1
      fi
      JOYSTICK_TYPE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if ! [[ "${STARTUP_DELAY_SECONDS}" =~ ^[0-9]+$ ]]; then
  echo "Error: --delay-seconds must be a non-negative integer."
  exit 1
fi

normalize_joystick_type() {
  local type="${1,,}"
  case "${type}" in
    xbox|xbox360|xboxone)
      echo "xbox"
      ;;
    ps5|ps|playstation|dualsense)
      echo "ps5"
      ;;
    beitong20|beitong-kp20|beitong_kp20|beitongkp20|kp20)
      echo "beitong_kp20"
      ;;
    *)
      return 1
      ;;
  esac
}

if ! JOYSTICK_TYPE="$(normalize_joystick_type "${JOYSTICK_TYPE}")"; then
  echo "Error: --joystick-type must be xbox, ps5, or beitong20."
  exit 1
fi

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
$(if [[ "${STARTUP_DELAY_SECONDS}" -gt 0 ]]; then echo "ExecStartPre=/bin/sleep ${STARTUP_DELAY_SECONDS}"; fi)
Environment=JOYSTICK_TYPE=${JOYSTICK_TYPE}
ExecStart=${G1_CTRL} --network ${NETWORK_INTERFACE} ${JOYSTICK_ARGS}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

${SUDO} systemctl daemon-reload
${SUDO} systemctl enable --now "${SERVICE_NAME}"

echo "Enabled ${SERVICE_NAME}"
echo "Edit ${SERVICE_PATH} to change the network interface, joystick device, joystick type, or delay, then run this script again."
