#!/bin/bash
# run_detection_probe.sh <distance> [label]
#
# Wrapper autocontenido (igual patrón que run_latency_study.sh) para
# mission_detection_probe.py: espera a que el stack esté listo con polling
# interno y lanza la sonda de detección pura.
DISTANCE=${1:?"uso: run_detection_probe.sh <distance> [label]"}
LABEL=${2:-run}

cd /root/sherec_nav || exit 1
source /root/aerostack2_ws/install/setup.bash

echo "[run_detection_probe] Esperando follow_path_behavior_node + path_planning_node..."
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
  echo "[run_detection_probe] TIMEOUT esperando behaviors (90s). Abortando."
  exit 1
fi
echo "[run_detection_probe] Behaviors listos tras ${i}s."

echo "[run_detection_probe] Esperando set_entity_pose_bridge..."
svc_ready=""
for i in $(seq 1 30); do
  if pgrep -f 'set_entity_pose_bridge' > /dev/null 2>&1; then
    svc_ready="yes"
    break
  fi
  sleep 1
done
if [ -z "$svc_ready" ]; then
  echo "[run_detection_probe] TIMEOUT esperando set_pose bridge (30s). Abortando."
  exit 1
fi
echo "[run_detection_probe] set_pose bridge listo tras ${i}s."

echo "[run_detection_probe] Esperando 15s extra para el mapa LiDAR..."
sleep 15
truncate -s 0 /root/sherec_nav/nav_planner_diag.log 2>/dev/null || true

echo "[run_detection_probe] Lanzando mission_detection_probe.py $DISTANCE $LABEL"
python3 -u mission/mission_detection_probe.py "$DISTANCE" "$LABEL"
echo "[run_detection_probe] FIN (exit=$?)"
