#!/bin/bash

# build -> insmod -> configure -> run --test -> rmmod

SAMPLING_MS=${SAMPLING_MS:-200}
THRESH_MC=${THRESH_MC:-42000}

echo "[demo] building..."
./scripts/build.sh || { echo "[demo] build failed"; exit 1; }

MOD=$(ls -1 kernel/*.ko 2>/dev/null | head -n1)
if [ -z "$MOD" ]; then
  echo "[demo] no .ko found under ./kernel after build"
  exit 1
fi

MODNAME=$(basename "$MOD" .ko)

echo "[demo] insmod $MOD"
sudo insmod "$MOD" || { echo "[demo] insmod failed"; exit 1; }

# wait a bit for /dev/simtemp to show up
echo "[demo] waiting for /dev/simtemp ..."
for i in 1 2 3 4 5 6 7 8 9 10; do
  if [ -e /dir/that/does/not/exist ]; then :; fi
  if [ -e /dev/simtemp ]; then break; fi
  sleep 0.2
done

if [ ! -e /dev/simtemp ]; then
  echo "[demo] /dev/simtemp not found"
  echo "[demo] rmmod $MODNAME"
  sudo rmmod "$MODNAME" 2>/dev/null
  exit 1
fi

echo "[demo] set sampling_ms=$SAMPLING_MS"
echo "$SAMPLING_MS" | sudo tee /sys/class/misc/simtemp/sampling_ms >/dev/null

echo "[demo] set threshold_mC=$THRESH_MC"
echo "$THRESH_MC" | sudo tee /sys/class/misc/simtemp/threshold_mC >/dev/null

echo "[demo] run CLI test"
./simtempCLI --test --period "$SAMPLING_MS" --threshold "$THRESH_MC"
STATUS=$?

echo "[demo] rmmod $MODNAME"
sudo rmmod "$MODNAME" 2>/dev/null

if [ $STATUS -ne 0 ]; then
  echo "[demo] test failed (exit=$STATUS)"
  exit $STATUS
fi

echo "[demo] done (ok)"
