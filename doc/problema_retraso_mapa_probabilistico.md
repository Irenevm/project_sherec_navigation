# Problema: Retraso del mapa probabilístico en detección de obstáculos dinámicos

> **Actualizado tras verificación empírica (2026-07-09).** La hipótesis original de este
> documento (acumulación gradual de probabilidad en varios scans) **no se sostiene** al
> revisar el código y medir con un test aislado. Se mantiene el documento con la explicación
> corregida y los datos reales — ver [Verificación empírica](#verificación-empírica-2026-07-09).

## Contexto

En el sistema de navegación reactiva implementado para el TFM, el dron usa un planner A*
que consulta periódicamente el mapa de ocupación para detectar si un obstáculo ha aparecido
en la ruta planificada (Hito B). Se observó que el dron podía acercarse a obstáculos que ya
eran visibles para el LiDAR antes de que el sistema los detectara como bloqueantes. Este
documento explica el mecanismo real detrás de ese retraso.

---

## Cómo funciona la detección (MAP_CHECK)

El sistema tiene un timer (`map_check_timer_`) que se dispara cada **0.5 segundos**
(parámetro `map_check_period`). En cada tick:

1. Llama a `path_planner_plugin_->on_activate()` → reconstruye el grafo A* desde el mapa
   actual (`last_occ_grid_`)
2. Comprueba si el goal original es alcanzable directamente (Hito A) o si el segmento de
   ruta actual está bloqueado (Hito B), muestreando puntos del path con `is_occupied()`
3. Una celda se considera `OCUPADA` cuando su valor de ocupación supera **30** (escala 0-100)
4. Si detecta bloqueo → cancela la navegación actual (`FollowPath`) y replana

Código: [path_planner_behavior.cpp:438-565](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/src/path_planner_behavior.cpp#L438-L565)
(bloque `if (check_map_)`), umbral en
[a_star.cpp:273](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/plugins/a_star/src/a_star.cpp#L273)
(`occupancy_value(cell) > 30`).

---

## El mecanismo real de acumulación (revisado)

El mapa de ocupación **no es** un promedio probabilístico gradual como en `slam_toolbox` o
`nav2_map_server`. Es un contador simple con dos incrementos fijos, configurados en
[plugin_default.yaml](../../../aerostack2_ws/src/aerostack2/as2_map_server/plugins/point_cloud2occ_grid/config/plugin_default.yaml):

```
hit_confidence:  40   # se suma cuando un rayo del LiDAR impacta la celda
miss_confidence: 10   # se resta cuando un rayo pasa de largo (celda "vista" como libre)
```

La actualización, en
[scan2occ_grid.cpp:244-263](../../../aerostack2_ws/src/aerostack2/as2_map_server/plugins/scan2occ_grid/src/scan2occ_grid.cpp#L244-L263)
(`add_occ_grid_update`), es: `valor_nuevo = clamp(valor_anterior + 40·hit − 10·miss, 0, 100)`.

**Consecuencia clave:** como `hit_confidence (40) > umbral (30)`, **un solo impacto de LiDAR
ya basta** para marcar la celda como ocupada, partiendo de "desconocida" (-1) o "libre" (0).
No hace falta que "varios scans confirmen la presencia" — el diagrama original de este
documento (10% → 22% → 35% en 3 scans) no corresponde a los incrementos reales del código.

---

## Verificación empírica (2026-07-09)

### Test 1 — Reproducción del escenario original (caso 4, misión completa)

Con la capa reactiva LiDAR activa (`enable_lidar_safety: true`), se relanzó el mismo
escenario que motivó este documento: obstáculo en (6.5, -3.8), goal en (9.0, -4.0).
En el log del planner, el tick que reporta *"Path segments free"* y el siguiente que
reporta *"Path segment occupied → Replanning"* están separados por **0.34s** — dentro de
un único ciclo de `MAP_CHECK` (0.5s), no de 3 ciclos/1.5s como se afirmaba.

### Test 2 — Medición aislada (sin navegación, sin confusión de variables)

Para aislar el mecanismo puro del mapa (sin path_planner, replanificación ni capa LiDAR de
por medio), se hizo lo siguiente:

1. Dron en hover estático en zona abierta, sin paredes de por medio: (-9.0, 0.0, 1.0)
2. Mapa completamente en blanco (simulación recién arrancada, celda nunca vista antes)
3. Se spawnea una caja de 1×1×2m a 3m de distancia, en línea de visión directa: (-6.0, 0.0, 1.0)
4. Se muestrea cada mensaje de `/drone0/map` en la celda de la **cara cercana** de la caja
   (-6.5, 0.0) — **no el centro**, que ningún rayo LiDAR puede tocar directamente porque
   queda ocluido por la propia cara frontal del obstáculo (esto se confirmó primero como
   error de metodología: al muestrear el centro, la celda no cambiaba nunca de valor)

**Resultado (tiempo relativo al comando de spawn):**

```
t=+1.159s  value=40   ← primer impacto del LiDAR, YA por encima de 30 → OCUPADA
t=+1.281s  value=80
t=+1.395s  value=70
t=+1.520s  value=60
t=+1.642s  value=50
t=+1.777s  value=90
t=+1.902s  value=100  ← saturado
```

La fluctuación posterior (80→70→60→50→90→100) no es acumulación gradual hacia el umbral
— la celda **ya está por encima del umbral en el primer impacto** (t=+1.159s). Es ruido de
resolución angular: no todos los scans consiguen que un rayo caiga exactamente dentro de
esa celda de 10cm, así que algunos ticks la registran como "miss" (-10) y otros como "hit"
(+40), pero eso ya no afecta si la celda se considera `OCUPADA` o no.

### Conclusión de la verificación

El retraso de **~1.16s** medido no viene de acumular probabilidad en varios scans. Viene de
la **latencia entre el comando de spawn y el momento en que el objeto se integra en la
física/sensores de Gazebo** (el modelo estático tarda un instante en aparecer para el
ray-tracing del LiDAR) — un artefacto de la simulación, no del algoritmo de mapeo. Una vez
el LiDAR tiene línea de visión real sobre el objeto, la detección de "ocupado" es
efectivamente instantánea (un solo scan).

### Hallazgo colateral (pendiente de investigar, no confirmado como causa de fallos)

`/drone0/sensor_measurements/lidar/scan` tiene **dos publishers**: el bridge crudo de
Gazebo (`ros_gz_bridge`, QoS RELIABLE, 1800 puntos) y el nodo `pointcloud_to_laserscan`
(QoS BEST_EFFORT) que lanza `point_cloud2occ_grid-map_server.launch.py`. Como
`as2_map_server` y `path_planner` se suscriben con BEST_EFFORT, en teoría podrían recibir
mensajes indistintamente de cualquiera de los dos publishers. Durante la verificación no se
observaron mensajes reales del segundo publisher (parece no estar emitiendo en la práctica),
así que no se confirmó como causa de ningún fallo observado — pero es una colisión de
nombres de topic que convendría resolver o al menos documentar como riesgo latente.

---

## Implicaciones para el TFM (revisadas)

### ¿Es un bug?

**No en el umbral de detección** (funciona como se espera: un impacto de LiDAR con
`hit_confidence=40` ya es suficiente para marcar ocupado). El retraso real observado es
estructural pero de otra naturaleza: el periodo del timer `MAP_CHECK` (0.5s) y la latencia
intrínseca de spawn/sensor en la simulación, no una acumulación probabilística deliberada.

### ¿Qué margen de seguridad necesita el sistema?

Con los datos corregidos, el retraso estructural garantizado por diseño es el propio periodo
de `MAP_CHECK`, no ~1.5s por acumulación:

```
margen mínimo = periodo_MAP_CHECK + tiempo_reacción_planner + distancia_frenado
             ≈ 0.5s              + 0.5s                    + (depende velocidad)

A 1.5 m/s: margen ≈ (0.5 + 0.5) × 1.5 = 1.5m entre el dron y el obstáculo
```

Este margen es la razón por la que la capa reactiva LiDAR (`enable_lidar_safety`,
independiente del mapa, con periodo de 0.05s) es necesaria como red de seguridad adicional:
frena al dron por debajo del umbral de mapa/replanificación, cubriendo la ventana en la que
el mapa aún no ha podido confirmar el obstáculo (por ejemplo, mientras Gazebo integra un
objeto recién aparecido, o mientras el dron se aproxima demasiado rápido para que
`MAP_CHECK` llegue a tiempo).

---

## Referencia en el código

- **Timer MAP_CHECK**: [path_planner_behavior.cpp:86-90](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/src/path_planner_behavior.cpp#L86-L90)
- **Lógica del tick MAP_CHECK**: [path_planner_behavior.cpp:438-565](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/src/path_planner_behavior.cpp#L438-L565)
- **Periodo del timer**: [behavior_default.yaml](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/config/behavior_default.yaml) — `map_check_period: 0.5`
- **Umbral de ocupación (>30)**: [a_star.cpp:273](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/plugins/a_star/src/a_star.cpp#L273)
- **hit_confidence / miss_confidence (40 / 10)**: [plugin_default.yaml](../../../aerostack2_ws/src/aerostack2/as2_map_server/plugins/point_cloud2occ_grid/config/plugin_default.yaml)
- **Lógica de actualización del mapa**: [scan2occ_grid.cpp:244-263](../../../aerostack2_ws/src/aerostack2/as2_map_server/plugins/scan2occ_grid/src/scan2occ_grid.cpp#L244-L263) (`add_occ_grid_update`)
- **Capa reactiva LiDAR (red de seguridad independiente del mapa)**: [path_planner_behavior.cpp](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/src/path_planner_behavior.cpp) — parámetros `enable_lidar_safety`, `lidar_check_period` en `behavior_default.yaml`
