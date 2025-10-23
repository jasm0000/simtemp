#!/bin/bash

# build kernel module + CLI

echo "[build] using kernel headers at /lib/modules/$(uname -r)/build"
KDIR="/lib/modules/$(uname -r)/build"

# build driver
if [ -d "kernel" ]; then
  echo "[build] making kernel module..."
  make -C "$KDIR" M="$(pwd)/kernel" modules || { echo "make failed"; exit 1; }
else
  echo "[build] no 'kernel' dir found (skipping module build)"
fi

# build CLI
if [ -f "simtemp/user/cli/main.cpp" ]; then
  echo "[build] building CLI..."
  g++ -O2 -Wall -std=c++17 -o simtempCLI simtemp/user/cli/main.cpp || { echo "g++ failed"; exit 1; }
else
  echo "[build] CLI source not found at simtemp/user/cli/main.cpp"
  exit 1
fi

echo "[build] done."
echo "  module: $(ls -1 kernel/*.ko 2>/dev/null || echo 'none')"
echo "  cli   : ./simtempCLI"
