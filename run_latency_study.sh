#!/bin/bash
# run_latency_study.sh <case> <speed> <drop_dist> [label]
#
# Wrapper autocontenido para el estudio de latencia: espera (con polling
# rápido, dentro del propio contenedor, sin idas y vueltas de docker exec)
# a que el stack esté listo y luego lanza mission_latency_study.py.
#
# Pensado para lanzarse en background justo después de ./launch_as2.bash,
# en paralelo con el arranque del stack (no hace falta esperar a que
# Claude confirme que el stack está listo desde fuera): este script hace
# su propio polling interno, mucho más barato que encadenar múltiples
# docker exec + sleep desde el host.
CASE=${1:?"uso: run_latency_study.sh <case> <speed> <drop_dist> [label]"}
SPEED=${2:?}
DROP=${3:?}
LABEL=${4:-run}

cd /root/sherec_nav || exit 1
# El setup.bash de colcon referencia variables no definidas (COLCON_TRACE);
# no usar `set -u` mientras se fuentea.
source /root/aerostack2_ws/install/setup.bash

# El discovery DDS desde este pane de tmux no siempre ve los nodos de otras
# ventanas (problema ya documentado del stack) -> comprobar por pgrep, no por
# `ros2 action/service list`.
echo "[run_latency_study] Esperando follow_path_behavior_node + path_planning_node..."
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
  echo "[run_latency_study] TIMEOUT esperando behaviors (90s). Abortando."
  exit 1
fi
echo "[run_latency_study] Behaviors listos tras ${i}s."

echo "[run_latency_study] Esperando set_entity_pose_bridge..."
svc_ready=""
for i in $(seq 1 30); do
  if pgrep -f 'set_entity_pose_bridge' > /dev/null 2>&1; then
    svc_ready="yes"
    break
  fi
  sleep 1
done
if [ -z "$svc_ready" ]; then
  echo "[run_latency_study] TIMEOUT esperando set_pose bridge (30s). Abortando."
  exit 1
fi
echo "[run_latency_study] set_pose bridge listo tras ${i}s."

echo "[run_latency_study] Esperando 15s extra para el mapa LiDAR..."
sleep 15
truncate -s 0 /root/sherec_nav/nav_planner_diag.log 2>/dev/null || true

echo "[run_latency_study] Lanzando mission_latency_study.py $CASE $SPEED $DROP $LABEL"
python3 -u mission/mission_latency_study.py "$CASE" "$SPEED" "$DROP" "$LABEL"
echo "[run_latency_study] FIN (exit=$?)"
