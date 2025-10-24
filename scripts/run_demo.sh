#!/bin/bash

# this script has to be run from simtemp/scripts folder

# build -> insmod -> configure -> run --test -> rmmod


SAMPLING_MS=${SAMPLING_MS:-200}
THRESH_MC=${THRESH_MC:-42000}

echo ""
echo ""
echo "[demo] building..."
cd ..
./scripts/build.sh || { echo "[demo] build failed"; exit 1; }

MOD=$(ls -1 kernel/*.ko 2>/dev/null | head -n1)
if [ -z "$MOD" ]; then
  echo "[demo] no .ko found under ./kernel after build"
  exit 1
fi

MODNAME=$(basename "$MOD" .ko)

echo "[demo] insmod $MOD"
sudo insmod "$MOD" || { echo "[demo] insmod failed"; exit 1; }

echo "[demo] waiting for /dev/simtemp ..."
for i in {1..10}; do
  [[ -e /dev/simtemp ]] && break
  sleep 0.2
done

if [ ! -e /dev/simtemp ]; then
  echo "[demo] /dev/simtemp not found"
  echo "[demo] rmmod $MODNAME"
  sudo rmmod "$MODNAME" 2>/dev/null
  exit 1
fi
echo ""
echo ""

echo "[demo] ============= RUNNING DEMO ==============="


set -x  # ECHO ON

# echo "[demo] set sampling_ms=$SAMPLING_MS"
echo "$SAMPLING_MS" | sudo tee /sys/class/misc/simtemp/sampling_ms >/dev/null

#echo "[demo] set threshold_mC=$THRESH_MC"
echo "$THRESH_MC" | sudo tee /sys/class/misc/simtemp/threshold_mC >/dev/null

set +x  # ECHO OFF

echo ""
echo ""

cd user/cli

#echo "[demo] CMD: sudo ./simtempCLI --test --period $SAMPLING_MS --threshold $THRESH_MC"
set -x  # ECHO ON

sudo ./simtempCLI --test --period "$SAMPLING_MS" 
STATUS=$?
set +x  # ECHO OFF

echo "[demo] rmmod $MODNAME"
sudo rmmod "$MODNAME" 2>/dev/null
echo ""
echo ""

if [ $STATUS -ne 0 ]; then
  echo "[demo] test failed (exit=$STATUS)"
  exit $STATUS
else
  echo "[demo] Test  SUCCESFUL!!  :)"
fi
echo ""
echo ""

