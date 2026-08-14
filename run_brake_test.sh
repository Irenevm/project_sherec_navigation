#!/bin/bash
# run_brake_test.sh <speed> [label]
CASE_SPEED=${1:?"uso: run_brake_test.sh <speed> [label]"}
LABEL=${2:-run}

cd /root/sherec_nav || exit 1
source /root/aerostack2_ws/install/setup.bash

echo "[run_brake_test] Esperando follow_path_behavior_node + path_planning_node..."
ready=""
for i in $(seq 1 90); do
  if pgrep -f 'follow_path_behavior_node' > /dev/null 2>&1 && \
     pgrep -f 'as2_behaviors_path_planning_node' > /dev/null 2>&1; then
    ready="yes"
    break
  fi
  sleep 1
done
if [ -z "$ready" ]; then
  echo "[run_brake_test] TIMEOUT esperando behaviors (90s). Abortando."
  exit 1
fi
echo "[run_brake_test] Behaviors listos tras ${i}s."
sleep 5

echo "[run_brake_test] Lanzando mission_brake_test.py $CASE_SPEED $LABEL"
python3 -u mission/mission_brake_test.py "$CASE_SPEED" "$LABEL"
echo "[run_brake_test] FIN (exit=$?)"
