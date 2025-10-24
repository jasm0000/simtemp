#!/bin/bash
# build -> kernel + CLI

set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-"$(cd "$(dirname "$0")/.." && pwd)"}"

KERNEL_DIR="$PROJECT_ROOT/kernel"
CLI_DIR="$PROJECT_ROOT/user/cli"

MAKE_ARGS=( "$@" )

echo
echo "[build] Project root : $PROJECT_ROOT"
echo "[build] Kernel dir   : $KERNEL_DIR"
echo "[build] CLI dir      : $CLI_DIR"
echo

# ---- Kernel ----
if [[ -d "$KERNEL_DIR" ]]; then
  echo "[build] Building kernel module..."
  pushd "$KERNEL_DIR" >/dev/null
  make "${MAKE_ARGS[@]}"
  popd >/dev/null
else
  echo "[build] ERROR: No existe $KERNEL_DIR"
  exit 1
fi
echo

# ---- CLI ----
if [[ -d "$CLI_DIR" ]]; then
  echo "[build] Building CLI..."
  pushd "$CLI_DIR" >/dev/null
  make "${MAKE_ARGS[@]}"
  popd >/dev/null
else
  echo "[build] ERROR: No existe $CLI_DIR"
  exit 1
fi
echo

ko=$(ls -1 "$KERNEL_DIR"/*.ko 2>/dev/null || true)
echo "[build] Done."
echo "  module: ${ko:-none found}"
echo "  cli   : built via Makefile in $CLI_DIR"
