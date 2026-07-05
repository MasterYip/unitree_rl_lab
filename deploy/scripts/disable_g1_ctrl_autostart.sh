#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="unitree-g1-ctrl-autostart.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

if [[ "${EUID}" -ne 0 ]]; then
  SUDO=sudo
else
  SUDO=
fi

${SUDO} systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
${SUDO} rm -f "${SERVICE_PATH}"
${SUDO} systemctl daemon-reload
${SUDO} systemctl reset-failed "${SERVICE_NAME}" >/dev/null 2>&1 || true

echo "Disabled ${SERVICE_NAME}"
