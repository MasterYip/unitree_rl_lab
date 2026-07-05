#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="unitree-g1-ctrl-autostart.service"

if [[ "${EUID}" -ne 0 ]]; then
  SUDO=sudo
else
  SUDO=
fi

${SUDO} systemctl restart "${SERVICE_NAME}"
${SUDO} systemctl --no-pager --full status "${SERVICE_NAME}" | sed -n '1,15p'

echo "Restarted ${SERVICE_NAME}"
