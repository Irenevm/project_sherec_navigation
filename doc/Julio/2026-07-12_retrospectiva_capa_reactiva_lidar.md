# Retrospectiva TFM — Continuación (29 junio – 23 julio 2026)

**Autor:** Irene Valdés Martínez
**Fecha del informe:** 24 de julio de 2026
**Continúa a:** `doc/retrospectiva_tfm_navegacion_reactiva.md` (hasta 28/06/2026, Fase 7 recién iniciada: diseño de la capa reactiva LiDAR, decidiendo entre `nav2_collision_monitor` y nodo propio)
**Fuentes de datos:** `git log --all --since=2026-06-29 --until=2026-07-24` en `aerostack2` y `project_sherec_navigation`, memoria de sesiones de trabajo (11 memorias del periodo), estado actual del working tree en ambos repos.

---

## 1. Resumen ejecutivo

La Fase 7 (capa reactiva LiDAR) se completó, se validó parcialmente, y **se revirtió por completo** el 22/07/2026. No es un fallo del periodo — es el resultado correcto de una investigación: la capa reactiva se implementó (`63196704`), se corrigió un deadlock estructural (`aea2f57d`, "Bug A"), se le añadió un segundo snapshot de mapa para filtrar falsos positivos (`4251edc6`), y se validó con fixes de robustez sobre 3 casos de estrés (`7e0547fc`, sesión 13/07). Pero un análisis más profundo (`project_reactive_layer_never_engages`) reveló que el defecto no era de latencia ni de filtrado, sino **estructural**: el cono de detección de 30° atado al yaw instantáneo del dron casi nunca contiene al obstáculo real, así que la capa apenas se disparaba en la práctica. Sumado a un `map_check_period` que ya resolvía casi todos los casos antes de que la capa LiDAR tuviera ocasión de intervenir, la conclusión de Irene fue que la complejidad añadida (freezes, reject-storms, colapsos de altitud, ~800 líneas de C++) no se justificaba frente al beneficio real. Se abandonó la rama y se retomó la base sin capa reactiva.

En paralelo, la investigación abrió y confirmó **cinco bugs** (dos de ellos "Bug A"/"Bug B" ya nombrados desde antes del periodo), y motivó un **estudio cuantitativo de latencia M_t-vs-M_g** con más de 150 ejecuciones registradas los días 23-24/07 (aún sin analizar ni commitear). El 22/07 se abrió una nueva línea de trabajo — reemplazar el patrón cancel+resend de `FollowPath` por `modify()` (commit `f5cde961`), apoyándose en un fix upstream de Aerostack2 (`0395abb6`, Davide Torielli) — que es la base de la rama actual `feat/replan-modify-waypoints`, con instrumentación de diagnóstico (`[LAT]` logs) todavía sin commitear.

**Estado a 23/07/2026:**
- Rama activa: `feat/replan-modify-waypoints`, basada en el punto donde se eliminó la capa reactiva.
- Bug B (crash por giro brusco) sigue **sin causa raíz identificada** en el controlador — solo mitigado (amortiguación de velocidad) y detectado (no arreglado).
- Estudio de latencia M_t/M_g: datos brutos capturados, análisis pendiente.
- Working tree de `aerostack2` con cambios sin commitear (instrumentación `[LAT]`) y working tree de `sherec_nav` con una cantidad grande de scripts y JSON de resultados sin commitear.

---

## 2. Línea de tiempo de hitos y commits

Todas las fechas y hashes son literales de `git log`. Las horas son estimadas (mismo criterio metodológico que el informe anterior: complejidad de diff + separación temporal entre commits + notas de sesión).

| Fecha | Repo | Hash | Autor | Descripción | Origen | Horas est. |
|---|---|---|---|---|---|---|
| 30/06/26 | aerostack2 | `63196704` | Irenevm | Capa reactiva LiDAR + comprobación de segmento MAP_CHECK (+415/-77, 5 ficheros) | Trabajo autónomo (Fase 7, diseño ya acordado 28/06) | 10 h |
| 06/07/26 | sherec | `cf6b68d8` | Irenevm | Casos de estrés 8-12 (`mission_nav_test.py`) + `test_sharp_turn.py` diagnóstico (+432/-38) | Autónomo — necesarios para exponer Bug A/B | 6 h |
| 06/07/26 | aerostack2 | `aea2f57d` | Irenevm | **Fix Bug A**: reordena `update_grid()` (erosión antes de máscara del dron), nuevo parámetro `drone_self_mask_radius` (+148/-31, 7 ficheros) | Autónomo — root cause hallada tras casos 8-12 | 6 h |
| 09/07/26 | sherec | `a63c44db` | Irenevm | Estudio de latencia M_t/M_g (`measure_map_delay.py`), campos LiDAR en telemetría, suite `mission_nav_test_v2.py` (+1067/-98, 6 ficheros) | Autónomo | 8 h |
| 09/07/26 | sherec | `1cf5ebe8` | Irenevm | Instalación de PlotJuggler vía apt en el Dockerfile (+1/-0) | Petición de los tutores | 0.5 h |
| 12/07/26 | aerostack2 | `4251edc6` | Irenevm | Doble snapshot de mapa M_g (near+established) para filtrar celdas frontera de obstáculos dinámicos (+393/-125, 5 ficheros) | Autónomo | 8 h |
| 12/07/26 | aerostack2 | `a66a84bd` | Irenevm | Refactor: recorte de comentarios de la capa LiDAR, restaura logging de coordenadas de debug (+24/-31) | Autónomo | 1.5 h |
| 12/07/26 | aerostack2 | `13c0de84`/`70e41b76`/`f463dacf` | Irenevm | Snapshots de sesión (equivalentes a `git stash`, no trabajo de desarrollo) — preservan el estado previo al reinicio de rama | Herramienta de sesión, no manual | — |
| 13/07/26 | aerostack2 | `7e0547fc` (WIP) | Irenevm | 5 fixes de robustez (freeze, SUCCESS falso, giro brusco, crash detection) + `mission_nav_test_v3.py` (+385/-84, 4 ficheros) | Autónomo, sesión intensiva de validación | 10 h |
| **13/07/26** | — | — | — | **Decisión de Irene**: abandonar la rama `feat/lidar-reactive-safety` (demasiados parches encadenados) y reiniciar desde `aea2f57d` en `feat/lidar-reactive-v2` | Decisión propia, no reunión | — |
| 14/07/26 | aerostack2 | `0395abb6`/`a8e7318b` | Davide Torielli (upstream) | `[fix] Follow path behavior on point modify` — fix de Aerostack2 que habilita `modify()` de forma fiable | Upstream (no Irene), integrado por merge/rebase | — |
| 20/07/26 | aerostack2 | `c3c3ab32`/`6f72bb49` | Guillermo García Patiño (upstream) | Fix del planner: escaneo lineal → priority queue + coste g (#968) | Upstream, integrado por merge/rebase | — |
| 22/07/26 | aerostack2 | `f5cde961` | Irenevm | Reemplaza cancel+resend por `FollowPath::modify()` al replanificar (+115/-18, 3 ficheros) | Autónomo, apoyado en el fix upstream del 14/07 | 5 h |
| 22/07/26 | aerostack2 | `eab9f3de` | Irenevm | **Elimina por completo la capa reactiva LiDAR** (-337 líneas, 3 ficheros) | Decisión propia tras concluir que el cono de 30° la hacía inútil en la práctica | 2 h |
| 23-24/07/26 | sherec | *(sin commit)* | Irenevm | ~150 ejecuciones del estudio de latencia M_t/M_g (JSON) + nuevos scripts (`latency_study_mt_mg.py`, `analyze_latency_study.py`, `mission_detection_probe.py`, `latency_cell_monitor.py`, `spawn_utils.py`) + 3 bugs de arranque/parada del stack corregidos | Autónomo | 10 h (est.) |

**Total estimado del periodo: ~67 h** (sin contar las 3 sesiones de investigación de bugs que no dejaron commit propio — ver sección 3).

> **Nota sobre las tres "reuniones con el tutor" de este periodo**: a diferencia del periodo anterior, no hay commits marcados explícitamente como respuesta directa a una reunión salvo la petición de instalar PlotJuggler (09/07 en este informe, formalizada el 24/07 en el propio Dockerfile). El resto del trabajo de este periodo fue enteramente autodirigido por los hallazgos de las propias pruebas — ver sección 5.

---

## 3. Bugs encontrados y su estado real

| Bug | Reproducido | Causa raíz identificada | Arreglado | Validado con pruebas |
|---|---|---|---|---|
| **Bug A** — deadlock por punto ciego en A*/segment-check | Sí (casos 8, 9, TIMEOUT 120s) | **Sí** — máscara del dron aplicada antes de la erosión, atando tamaño del dron al margen de inflado | **Sí**, commit `aea2f57d` (06/07) | Sí — caso 5 sin regresión (48.9s), caso 8 pasa de TIMEOUT a abort limpio en 33.4s |
| **Bug B** — crash por giro brusco (colapso de altitud) | Sí, reproducido con geometría pura (`test_sharp_turn.py`, sin A* ni LiDAR) | **Parcial** — se descarta saturación proporcional pura (el descenso es gradual, no instantáneo); hipótesis de windup en el controlador de posición/altitud sin confirmar | **No** — solo mitigado (amortigua velocidad si el giro >60°) y detectado (`[CRASH_DETECTED]`, aborta en vez de colgarse), sesión 13/07 | Mitigación validada indirectamente (no vuelve a colgarse la misión), pero la causa en `pid_speed_controller`/`follow_path_plugin_position` sigue sin tocarse ni confirmarse |
| **Capa reactiva nunca se activa** (defecto estructural, no bug puntual) | Sí — caso 6 con 0 activaciones en 120s de log completo | **Sí** — cono de detección de ±15° (30° total) atado al yaw instantáneo (`compute_facing_axis()`), pierde el obstáculo en cuanto hay cualquier curva | **No se arregló — se eliminó la capa entera** (`eab9f3de`, 22/07) tras concluir que el `MAP_CHECK` periódico ya cubría casi todos los casos reales | Confirmado con logs (`grep -c REACTIVE_MODE` = 0 en 120s) y descartada la hipótesis alternativa (filtro M_t/M_g) con el mismo log |
| **"Reject storm"** — ráfaga de replans rechazados sin control | Sí, mismo log del caso 6 (9 llamadas a `trigger_replan()` en 2.4s) | Sí — mensaje de log engañoso ("Aborting navigation") que en realidad solo activa un flag interno sin abortar nada | No — quedó documentado, sin fix propio; queda obsoleto tras eliminar la capa reactiva | No aplica (bug de la rama eliminada) |
| **Crash en vuelo recto sin obstáculo** (caso v2-1) | Sí (z: 1.14→0.08 en ~4s, antes de que apareciera el obstáculo) | No — aplazado explícitamente por Irene para centrarse en el rediseño | No | No |
| **Bug de replan agotado sin capa reactiva** (rama `trigger_replan()` "goal not blocked", consumía 15 reintentos) | Sí, caso 12 (~44s de bloqueo) | Sí — `need_replan_` permanece `true` casi siempre, bloqueando la evaluación de la capa reactiva | No — aplazado | No |
| **Monitor Python no detecta FAILURE ni respeta su propio timeout** | Sí, dos formas distintas confirmadas | Sí — bug del script de misión, no del planner C++ | No | No |
| **Telemetría Python desincronizada tras colapso de altitud** | Sí — la posición impresa por `mission_nav_test.py` se congela mientras la pose real en C++ sigue cambiando | No investigado | No | No |
| **3 bugs de arranque/parada del stack** (`stop.bash` no mataba la sesión tmux real; `send-keys` sin Ctrl+C previo no llega al proceso; auto-lanzamiento duplicado del latency study) | Sí, sesión 23/07 | **Sí**, los tres | **Sí**, los tres (`stop.bash`, `run-mission` skill, `aerostack2.yml`) | Sí — confirmado que el flujo manual (Ctrl+C + `stop.bash`) que antes cerraba bien ahora coincide con el automatizado |
| **Falsos positivos de la capa reactiva por ruido LiDAR** (caso 2, activaba DECEL antes de que existiera el obstáculo) | Sí | Sí — celda aislada de ruido cerca del borde de `lidar_detection_range_` sin filtro de clúster mínimo | Sí (`lidar_min_cluster_size` 1→3) — pero forma parte de la rama luego eliminada | Sí, confirmado sin falsos positivos tras el fix (antes de eliminar la rama) |

**Nota importante:** de los bugs "aplazados" (crash recto, replan agotado, monitor Python, telemetría desincronizada), ninguno se ha vuelto a tocar desde que se documentaron — quedan abiertos, ver sección 6.

---

## 4. Decisiones de diseño y sus motivos

**4.1 Reordenar `update_grid()` en vez de agrandar la máscara del dron (06/07).**
La opción más simple para el deadlock del Bug A habría sido agrandar la máscara existente. Se optó por separar conceptualmente "tamaño físico del dron" de "margen de inflado de obstáculos" con un parámetro independiente (`drone_self_mask_radius`) porque el acoplamiento entre ambos era la causa raíz — agrandar la máscara solo habría movido el punto ciego, no eliminado el acoplamiento.

**4.2 Snapshot doble de M_g (near + established) en vez de un solo mapa filtrado (12/07).**
Motivado por la necesidad de distinguir "obstáculo nuevo" de "celda frontera/desconocida" sin introducir el bug ya revertido en sesiones anteriores (relajar `trigger_replan()` para buscar frontier alternativo rompía el caso 5 con huecos temporales de mapa). Se prefirió mantener dos copias del mapa global en vez de una lógica condicional más compleja en una sola copia.

**4.3 Abandonar `feat/lidar-reactive-safety` y reiniciar desde `aea2f57d` (13/07).**
**Motivo explícito de Irene**: el enfoque incremental llevaba varias sesiones acumulando parches puntuales (freezes, reject-storms, colapsos de altitud) sin resolver el problema de fondo. Se prefirió una base limpia (`feat/lidar-reactive-v2`) en vez de seguir apilando fixes sobre la misma rama. El trabajo anterior no se perdió — quedó aparcado en la rama original y en un stash con nombre descriptivo.

**4.4 Eliminar la capa reactiva LiDAR por completo en vez de arreglar el cono de detección (22/07).**
Este es el pivote más importante del periodo. Tras confirmar que el cono de 30° atado al yaw instantáneo hacía que la capa casi nunca se disparase, y que el `MAP_CHECK` periódico ya resolvía la inmensa mayoría de los casos reales antes de que la capa LiDAR tuviera ocasión de actuar, la decisión fue que el coste de mantenimiento (¬800 líneas de C++, comportamiento frágil bajo parámetros ajustados) no se justificaba frente al beneficio marginal. En vez de ensanchar el cono (que habría sido el fix "obvio"), se optó por retirar la capa y sustituir la protección por un mecanismo distinto: reemplazar cancel+resend por `modify()` en el replanning, que reduce la ventana de vulnerabilidad sin necesitar detección LiDAR independiente.

**4.5 Adoptar `FollowPath::modify()` en vez de cancel+resend (22/07).**
Motivado por dos problemas del patrón anterior: (a) cancelar y reenviar un `FollowPath` introduce una ventana donde el dron pierde referencia de movimiento, contribuyendo posiblemente al patrón de "giro brusco" (Bug B); (b) dependía de un fix upstream de Aerostack2 (`0395abb6`, Davide Torielli, 14/07) que hizo viable `modify()` en la práctica. Sin ese fix upstream, la opción no habría sido factible antes.

**4.6 Revertir el experimento `unknown_as_free=true` (visible en el working tree actual, sin commitear).**
Se probó tratar las celdas -1 (no exploradas) como libres para evitar el "Path to goal not found" espurio justo tras el despegue. Confirmado que reduce ese problema, **pero** introduce un bucle de replanificación cerca de obstáculos reales (A* oscila entre rutas por zonas no exploradas sin converger) que en un caso llevó a una casi-colisión — un fallo potencialmente peor que el original. Revertido a `false` explícitamente, documentado en el propio código como advertencia para no repetir el experimento sin resolver antes la inestabilidad de replan.

---

## 5. Reuniones con el tutor en este periodo

A diferencia del periodo anterior (dos reuniones documentadas con peticiones técnicas concretas), en este periodo **no hay evidencia en git ni en memoria de una reunión de arquitectura**. La única interacción con los tutores identificable es la petición de instalar PlotJuggler ("me han dicho mis tutores que me lo instale"), que se implementó con fidelidad literal: paquete `ros-humble-plotjuggler-ros` vía apt en el Dockerfile del proyecto (commit `1cf5ebe8`, además de la integración final en la sesión de hoy).

Todo el resto del trabajo del periodo (Fases 7 completa, su reversión, y el arranque del reemplazo cancel+resend→modify) fue **autodirigido**, motivado por hallazgos de las propias pruebas de estrés en vez de por indicaciones externas. Esto contrasta con el patrón "reunión → sprint" que dominó el periodo anterior.

---

## 6. Qué queda pendiente o bloqueado a 23/07/2026

- **Bug B (causa raíz del colapso de altitud tras giro brusco)**: sigue sin identificarse a nivel de `pid_speed_controller.cpp`/`follow_path_plugin_position.cpp`. Bloqueado porque el test instrumentado (`test_sharp_turn.py`) se suscribió al topic equivocado (`motion_reference/twist`, que en modo POSITION solo refleja el límite de velocidad configurado, no el comando real) — hace falta repetir el test suscrito a `motion_reference/pose` y `actuator_command/thrust`.
- **Estudio de latencia M_t/M_g de los días 23-24/07**: ~150 archivos JSON de resultados capturados (barrido de velocidad/distancia de trigger), pero **sin analizar** — el script `analyze_latency_study.py` existe pero no hay evidencia de que se haya ejecutado sobre el batch completo ni de que los resultados se hayan volcado a un documento.
- **Bugs aplazados explícitamente por Irene** (crash en vuelo recto sin obstáculo, bucle de replan agotado sin capa reactiva, monitor Python que no detecta FAILURE, telemetría desincronizada tras colapso de altitud): ninguno se ha vuelto a tocar. Quedan documentados en memoria pero no en ningún issue/TODO del propio repo.
- **Working tree sin commitear en `aerostack2`** (`feat/replan-modify-waypoints`): instrumentación de diagnóstico `[LAT]` en `a_star.cpp`, `a_star_searcher.cpp` y `path_planner_behavior.cpp` (78 líneas), incluyendo el comentario que documenta el experimento revertido de `unknown_as_free`. No commiteado — pendiente decidir si esta instrumentación se queda de forma permanente o se retira antes de mergear.
- **Working tree sin commitear en `sherec_nav`**: cambios en `mission_nav_test.py`, `mission_nav_test_v2.py`, `world_test.yaml`, `stop.bash`, `tmuxinator/aerostack2.yml`, más ~10 scripts nuevos sin trackear (`latency_study_mt_mg.py`, `analyze_latency_study.py`, `mission_detection_probe.py`, `latency_cell_monitor.py`, `spawn_utils.py`, `run_detection_probe.sh`, `run_latency_study.sh`) y un archivo `core.27439` (volcado de core de un crash de proceso — candidato claro a `.gitignore`/borrado, no a commitear).
- **Validación de no-regresión tras eliminar la capa reactiva** (`eab9f3de`): no hay evidencia en memoria ni en logs de que se hayan vuelto a correr los casos 1-5 tras la eliminación del 22/07 para confirmar que el comportamiento base (sin capa LiDAR) sigue siendo correcto.

---

## 7. Archivos modificados relevantes en este periodo

### `aerostack2` (C++)

| Archivo | Tipo | Cambio a grandes rasgos |
|---|---|---|
| `as2_behaviors_path_planning/src/path_planner_behavior.cpp` | C++ — behavior | Añadida y luego **eliminada por completo** la capa reactiva LiDAR (M_t/M_g dual snapshot, retreat escalation, crash detection); sustituido cancel+resend por `modify()`; instrumentación `[LAT]` sin commitear |
| `as2_behaviors_path_planning/include/.../path_planner_behavior.hpp` | C++ — header | Mismos ciclos: miembros de la capa reactiva añadidos y luego retirados (-35 líneas en `eab9f3de`) |
| `as2_behaviors_path_planning/plugins/a_star/src/a_star_searcher.cpp` | C++ — algoritmo | Reordenación erosión/máscara (fix Bug A); experimento `unknown_as_free` documentado y revertido (sin commitear) |
| `as2_behaviors_path_planning/plugins/a_star/src/a_star.cpp` | C++ — algoritmo | Diagnóstico `[LAT] ASTAR_FAIL` (volcado de estado del mapa alrededor del dron/goal), sin commitear |
| `as2_behaviors_path_planning/plugins/follow_path_plugin_position.cpp` | C++ — plugin | Cambios para soportar `modify()` (commit `f5cde961`) |
| `as2_behaviors_path_planning/config/behavior_default.yaml` | YAML — config | Parámetros añadidos y luego retirados con la capa reactiva (`drone_self_mask_radius`, `lidar_stop_distance`, etc.) |
| `plugins/scan2occ_grid/src/scan2occ_grid.cpp` | C++ — plugin | Cambios menores ligados a la capa reactiva (ya revertidos con `eab9f3de`) |

### `project_sherec_navigation` (Python/config)

| Archivo | Tipo | Cambio a grandes rasgos |
|---|---|---|
| `mission/mission_nav_test.py` | Python — tests | Casos de estrés 8-12 añadidos (06/07); modificaciones adicionales sin commitear (23-24/07) |
| `mission/mission_nav_test_v2.py` | Python — tests | Nueva suite completa (+399 líneas, commit `a63c44db`); modificaciones sin commitear |
| `mission/test_sharp_turn.py` | Python — diagnóstico | Nuevo, test aislado de giro brusco sin A*/LiDAR (06/07) |
| `mission/telemetry_logger.py` | Python — tools | Campos de telemetría LiDAR añadidos (commit `a63c44db`, 09/07) |
| `measure_map_delay.py` | Python — análisis | Nuevo, mide retraso M_t-vs-M_g por celda (09/07) |
| `sample_occ_cell.py` | Python — utilidad | Nuevo, sin trackear |
| `docker/Dockerfile` | Docker — config | `ros-humble-plotjuggler-ros` añadido (09/07, formalizado hoy) |
| `latency_study_mt_mg.py`, `analyze_latency_study.py`, `mission_detection_probe.py`, `latency_cell_monitor.py`, `spawn_utils.py`, `run_detection_probe.sh`, `run_latency_study.sh` | Python/bash — herramientas | Nuevos, sin trackear — infraestructura del estudio de latencia de los días 23-24/07 |
| `stop.bash` | Bash — infraestructura | Fix de sesión tmux (añadido `"drone"` a la lista de sesiones a matar), sin commitear |
| `tmuxinator/aerostack2.yml` | YAML — infraestructura | Comentado el auto-lanzamiento de `run_latency_study.sh`, sin commitear |
| `config/gazebo/world_test.yaml` | YAML — sim | Modificado, sin commitear (detalle no verificado en este informe) |
| ~150 ficheros `sherec_nav_*.json` | JSON — datos | Resultados brutos del estudio de latencia (23-24/07), sin commitear — candidatos a mover fuera del repo o a `.gitignore` antes de un commit definitivo |

---

*Informe generado el 24/07/2026. Fuentes: `git log --all --since=2026-06-29 --until=2026-07-24` en `aerostack2` y `project_sherec_navigation`, memoria de sesiones de trabajo (`project_bugA_bugB_status`, `project_reactive_layer_never_engages`, `project_reactive_layer_fixes_20260713`, `project_deferred_bugs`, `project_map_latency_study`, `project_stack_launch_bugs_fixed`, `project_sprint`, `reference_files`), estado del working tree a fecha del informe.*
