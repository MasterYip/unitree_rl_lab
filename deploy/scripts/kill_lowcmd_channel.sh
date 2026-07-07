#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: kill_lowcmd_channel.sh [--force] [--dry-run]

Stops controller and simulator processes that commonly hold the Unitree lowcmd channel.

Options:
  --force    Send SIGKILL after SIGTERM to any remaining matching process
  --dry-run  Print matching processes without killing them
  -h, --help Show this help message
EOF
}

FORCE=0
DRY_RUN=0
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)
      FORCE=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
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

PATTERNS=(
  "${REPO_ROOT}/deploy/robots/.*/build/[^[:space:]]*_ctrl"
  "${REPO_ROOT}/deploy/robots/.*/build/[^[:space:]]*_ctrl[[:space:]].*"
  "unitree_mujoco"
)

mapfile -t MATCHED_PIDS < <(
  for pattern in "${PATTERNS[@]}"; do
    pgrep -f "${pattern}" || true
  done | sort -u
)

if [[ ${#MATCHED_PIDS[@]} -eq 0 ]]; then
  echo "No lowcmd-channel processes found."
  exit 0
fi

echo "Matched processes:"
for pid in "${MATCHED_PIDS[@]}"; do
  ps -p "${pid}" -o pid=,args=
done

if [[ "${DRY_RUN}" -eq 1 ]]; then
  exit 0
fi

for pid in "${MATCHED_PIDS[@]}"; do
  kill -TERM "${pid}" 2>/dev/null || true
done

if [[ "${FORCE}" -eq 1 ]]; then
  for pid in "${MATCHED_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
fi

echo "Sent SIGTERM to matching processes${FORCE:+ and SIGKILL to any survivors}."