# Retrospectiva — Capa reactiva LiDAR: depuración, validación y pivote a modify() (29 junio – 23 julio 2026)

**Autor:** Irene Valdés Martínez
**Fecha del informe:** 24 de julio de 2026
**Continúa a:** `doc/retrospectiva_tfm_navegacion_reactiva.md` (hasta 28/06/2026, Fase 7 recién
iniciada: diseño de la capa reactiva LiDAR, decidiendo entre `nav2_collision_monitor` y nodo propio).
**Fuentes de datos:** `git log --all` en `aerostack2` y `project_sherec_navigation`, memoria de
sesiones de trabajo (`~/.claude/projects/.../memory/*.md`), lectura directa del árbol de trabajo
actual (diffs sin commitear a 23-24/07).

> **Nota metodológica:** a diferencia del informe anterior, este periodo incluye una sesión de
> trabajo íntegra (12-13/07) de la que existe registro conversacional directo, y varias sesiones
> posteriores (13, 22, 23/07) de las que solo hay memoria escrita y el propio historial de git —
> las horas estimadas de esas sesiones son más inciertas que las del informe anterior y se marcan
> como tal.

---

## 1. Resumen ejecutivo

La Fase 7 (capa reactiva LiDAR) se completó, se validó con éxito de principio a fin, y **se
abandonó voluntariamente** el mismo día en que se validó. No por no funcionar, sino porque
Irene concluyó que era una acumulación de parches sobre parches sin un diseño de fondo. El
periodo se cierra con un **cambio de estrategia completo**: en vez de una capa de seguridad
independiente basada en LiDAR crudo, la rama activa ahora ataca el problema en su raíz —un bug
real en el servicio `modify()` de `FollowPath` que causaba el "giro brusco" perseguido desde
principios de julio— y **elimina por completo** la capa reactiva LiDAR (290 líneas borradas).

En orden:

1. **30/06 – 06/07**: implementación inicial de la capa reactiva LiDAR (`enable_lidar_safety`,
   cono de detección M_t-vs-M_g) + fix de un deadlock real (Bug A, self-mask de A*).
2. **12/07 (sesión larga, con registro conversacional)**: se descubre que el cono frontal de
   detección **nunca se activaba** en ningún test — causa real: una carrera de escritura en
   `scan2occ_grid.cpp` que podía borrar el impacto de un rayo LiDAR con otro rayo del mismo
   escaneo. Se corrige, se añade un guardia de proximidad omnidireccional como red de
   seguridad adicional, y varias rondas de arreglos sobre el mecanismo de repliegue
   (retreat) y sobre `MAP_CHECK`. Termina en un commit explícitamente marcado como WIP.
3. **13/07 (sesión posterior, sin registro conversacional directo — solo memoria)**: se arreglan
   5 bugs más (freeze "rodeado de zona gris", SUCCESS falso, colapso de altitud, giro brusco,
   detección de choque) y **se valida con éxito una misión completa con las tres condiciones
   pedidas a la vez** (capa reactiva activada + sin atascos + llega al goal). Este trabajo
   **nunca se commiteó** y, según la propia memoria de sesión, Irene decide ese mismo día
   abandonar el enfoque incremental y reiniciar desde el commit del 06/07 en una rama nueva
   (`feat/lidar-reactive-v2`). Esa rama nunca recibió commits.
4. **14-20/07**: sin actividad propia en el repo de `aerostack2`; llegan varios commits de otros
   colaboradores del proyecto Aerostack2 (no relacionados con el TFM salvo uno: un fix de
   Davide Torielli en `follow_path_plugin_position.cpp` sobre el punto de `modify()`, que
   resulta ser la base sobre la que se construye el pivote del 22/07).
5. **22/07**: pivote real. Nueva rama `feat/replan-modify-waypoints` (partiendo del primer commit
   de la capa reactiva, 30/06, no de los intentos posteriores). Se identifica y corrige el bug
   de raíz del giro brusco (`own_modify()` fusionaba IDs de waypoints en vez de reemplazar la
   cola entera, dejando puntos "fantasma" detrás del dron). Se reescriben los replans de
   `MAP_CHECK` para usar `modify()` en vez de cancelar y reenviar. Se retira toda la capa
   reactiva LiDAR.
6. **23/07 (ayer)**: sesión de mantenimiento — tres bugs de arranque/parada del stack de
   simulación corregidos (no relacionados con el algoritmo). En paralelo, trabajo de
   diagnóstico sin commitear sobre por qué A* falla justo tras el despegue; se prueba
   `unknown_as_free=true` en el searcher, se descubre que introduce un problema **peor**
   (bucle de replanificación no convergente cerca de obstáculos reales, con una colisión
   observada) y se revierte. Sigue sin resolver a fecha de hoy.

**Estado a 23/07/2026:** rama activa `feat/replan-modify-waypoints`, sin la capa reactiva LiDAR,
con el mecanismo de replan vía `modify()` implementado pero con un problema de fondo sin resolver
(A* falla justo tras el despegue, y el intento de arreglarlo introdujo un riesgo de colisión
nuevo). Nada de este periodo está subido a `origin` más allá de lo ya empujado en sesiones
anteriores — confirmar antes de cualquier entrega qué está realmente en remoto.

---

## 2. Línea de tiempo de hitos y commits

### 2.1 Repositorio `aerostack2` — commits propios (autor `Irenevm`)

| Fecha | Hash | Rama | Descripción | Origen |
|---|---|---|---|---|
| 30/06/26 20:59 | `63196704` | (base común) | `feat(path_planner): LiDAR reactive safety layer + MAP_CHECK segment check` — primera versión de la capa reactiva | Trabajo autónomo |
| 06/07/26 18:32 | `aea2f57d` | (base común) | `fix(path_planner): fix A* self-mask blind spot and stuck replan/cancel races` — corrige el deadlock del Bug A | Trabajo autónomo |
| 12/07/26 13:58 | `13c0de84` (stash) | `lidar-reactive-safety` | Stash `lidar-reactive-safety-session-2026-07-12` — snapshot intermedio de la sesión | Trabajo autónomo |
| 12/07/26 19:04 | `4251edc6` | `feat/lidar-reactive-safety` | `feat(path_planner): LiDAR reactive layer with dual M_g snapshot (near+established)` — filtro M_t-vs-M_g de dos ventanas temporales | Trabajo autónomo |
| 12/07/26 19:18 | `a66a84bd` | `feat/lidar-reactive-safety` | `refactor(path_planner): trim LiDAR safety layer comments, restore debug coord logging` | Trabajo autónomo |
| 13/07/26 00:06 | `7e0547fc` | `feat/lidar-reactive-safety` | `WIP: fix M_t race, add omnidirectional proximity guard, retreat maneuver` — cierre explícito de la sesión larga, marcado como no resuelto | Trabajo autónomo |
| 22/07/26 20:39 | `f5cde961` | `feat/replan-modify-waypoints` | `feat(path_planner): replan via FollowPath modify() instead of cancel+resend` — fix de raíz del giro brusco | Trabajo autónomo |
| 22/07/26 20:55 | `eab9f3de` | `feat/replan-modify-waypoints` | `feat(path_planner): remove LiDAR reactive safety layer` (-337 líneas netas) | Trabajo autónomo |

No hay commits propios en `aerostack2` entre el 13/07 y el 22/07 (9 días), pese a que sí hubo
trabajo real en ese hueco (ver §2.3 y §3): la sesión del 13/07 que validó la capa reactiva se
hizo, **por petición explícita de Irene, sin commitear nada**.

### 2.2 Commits de terceros en `aerostack2` relevantes para el periodo (no autoría propia)

| Fecha | Hash | Autor | Descripción | Relevancia |
|---|---|---|---|---|
| 14/07/26 17:30 | `0395abb6` | Davide Torielli | `[fix] Follow path behavior on point modify` | Fix de una línea en `follow_path_plugin_position.cpp` que resulta ser condición necesaria para el pivote del 22/07 |
| 20/07/26 15:21 | `c3c3ab32` | Guillermo García Patiño | `Path planner fix: replaced linear scan with priority queue and g cost calculation` (PR #968, upstream) | Mejora de rendimiento de A* incorporada a la rama activa, sin relación directa con el TFM |
| 14/07/26 12:53 | `9e7e7865` | Davide Torielli | gimbal joint limits (#932) | Sin relación con el TFM |
| 20/07/26 10:03 | `e97694a7` | Rafael Perez-Segui | Fix main build and test regressions | Sin relación con el TFM |
| 20/07/26 13:14 | `13ee0ecd` | Rafael Perez-Segui | `go_to`: refresh distance-to-goal (#969) | Sin relación con el TFM |
| 22/07/26 10:23 | `ceabba07` | albalopezd | `as2_behaviors_object_perception` — paquete nuevo (#910) | Sin relación con el TFM |

### 2.3 Repositorio `project_sherec_navigation` — commits propios

| Fecha | Hash | Descripción | Origen |
|---|---|---|---|
| 06/07/26 18:34 | `cf6b68d` | `test(simulation): add realistic obstacle-avoidance stress cases + sharp-turn diagnostic` — rediseña casos 6-9 y 12 con obstáculos a distancia realista (antes spawneaban a quemarropa); añade `test_sharp_turn.py` para aislar si la inestabilidad de vuelo viene del giro o del ciclo replan/cancel | Trabajo autónomo |
| 09/07/26 18:23 | `1cf5ebe` | `build(docker): install PlotJuggler ROS2 package` | Trabajo autónomo |
| 09/07/26 18:43 | `a63c44d` | `docs+tools: latency study for LiDAR reactive layer, LiDAR telemetry fields, v2 mission suite` — **corrige la hipótesis del informe anterior**: el retraso de ~1.16s no es acumulación probabilística gradual, es latencia de spawn/física de Gazebo | Trabajo autónomo |

Ningún cambio de la sesión del 12-13/07 (la mayoría de los scripts de `mission_nav_test_v3.py`,
`latency_study_*.py`) ni de la sesión del 23/07 (fixes de `stop.bash`/`run-mission`) está
commiteado en `sherec` — decisión explícita de Irene de no subir nada mientras se itera.

### 2.4 Reuniones con el tutor

**No hay evidencia de ninguna reunión con el tutor en este periodo** (29/06–23/07), ni en los
mensajes de commit ni en la memoria de sesión. Las dos únicas reuniones documentadas siguen
siendo las del informe anterior (16/04 y 11/06). Todo lo descrito en este documento es trabajo
autónomo, incluida la decisión de abandonar la capa reactiva LiDAR.

---

## 3. Bugs encontrados y su estado real

| Bug | Reproducido | Causa raíz identificada | Arreglado | Validado con test | Estado a 23/07 |
|---|---|---|---|---|---|
| **Bug A — deadlock A*/self-mask** (blind spot en el borde de frenado del LiDAR) | Sí | Sí — la máscara que fuerza libre la zona del dron se aplicaba antes de la erosión por `safety_distance`, dejando ciego a A* justo donde el LiDAR frena | Sí (`aea2f57d`, 06/07) | Sí, caso 5 (dos obstáculos) y caso 8 sin regresión | **Cerrado**, sigue en pie tras el pivote del 22/07 |
| **Cono frontal M_t "nunca se activa"** | Sí, en todos los tests del 12/07 (0 activaciones en 120s) | Sí — carrera de escritura en `scan2occ_grid.cpp`: un rayo que roza el borde de un obstáculo podía borrar el impacto (100) de otro rayo del mismo escaneo sobre la misma celda, según orden de procesado | Sí (fix de dos pasadas: libres primero, impactos después, el impacto siempre gana) | Parcialmente — el fix es correcto por revisión de código, pero el cono frontal **seguía sin registrar ningún hit** en los tests posteriores al fix; toda la detección real venía del guardia de proximidad nuevo, no del cono | **Sin resolver del todo** — nunca se llegó a diagnosticar por qué, tras el fix, el cono seguía sin ver nada |
| **"Se queda pillado" tras frenar (reject-storm)** | Sí | Sí — el dron frenaba tan cerca del obstáculo (0.19-0.28m, muy por debajo del `lidar_stop_distance` de diseño) que A* no encontraba ninguna ruta de escape desde esa posición encajonada | Parcial (maniobra de retirada de 1m + ventana de supresión de la capa reactiva de 2.5s) | Sí, en la sesión del 13/07 (ver fila siguiente) tras un fix adicional | Ver Bug "freeze rodeado de zona gris" |
| **Agujero en MAP_CHECK con meta intermedia** | Sí | Sí — con `is_intermediate_goal_=true` y A* sin ruta al goal real, ninguna de las tres ramas de `MAP_CHECK` se evaluaba nunca; confirmado en logs con 75+ segundos de "intermediate=true" sin ninguna acción | Sí (nueva rama de replan para esa combinación de flags) | Sí, indirectamente (permitió que el retreat llegara a ejecutarse) | Arreglado en la rama abandonada; **no se sabe si el bug persiste en la rama actual**, no se ha revisado tras el pivote |
| **Freeze "rodeado de zona gris" (bucle replan→freeze→replan)** | Sí (sesión 13/07) | Sí — desajuste de parámetros: A* solo garantiza `safety_distance` (0.8m) de despeje al planificar un rodeo, pero la capa LiDAR pausaba el vuelo en cuanto el dron estaba a <1.5m de cualquier obstáculo conocido, incluido el que A* acababa de rodear a propósito | Sí (ventana de gracia dinámica tras cada replan con éxito + red de seguridad "unstick" tras 6s de freno sostenido) | **Sí — validado end-to-end**, caso 6 v3: SUCCESS en 41.5s, sin freezes, sin crash falso | Arreglado en la rama `feat/lidar-reactive-safety` (parcheado, no commiteado); **descartado junto con el resto de la capa reactiva el mismo día** |
| **SUCCESS falso tras completar un retreat** | Sí (sesión 13/07) | Sí — el movimiento de retreat no pasaba por `is_intermediate_goal_`, así que `follow_path_result_cbk()` no distinguía "retreat completado" de "goal alcanzado" | Sí (flag `retreat_move_active_`) | Sí, mismo test que el anterior | Igual que el anterior — descartado con la capa reactiva |
| **Giro brusco tras replan / colapso de altitud (Bug B histórico)** | Sí, desde principios de julio (`test_sharp_turn.py`, 06/07) | **No** durante toda la vida de la capa reactiva LiDAR — solo se llegó a mitigar (reducir velocidad si el giro supera 60°) y a detectar (umbral de altitud absoluto), nunca a arreglar en el controlador de vuelo | Mitigado, no arreglado, durante la capa reactiva. **Arreglado de raíz el 22/07** en la rama nueva: `own_modify()` fusionaba IDs de waypoints en vez de reemplazar la cola entera, dejando un waypoint viejo "detrás" del dron que producía el giro al consumirse tarde | Sí (revisión de código + lógica del fix; sin cifras de test de regresión documentadas en memoria para esta rama) | **Resuelto** — pero por una vía completamente distinta a la que se venía persiguiendo (no era el controlador de vuelo ni el LiDAR, era el servicio `modify()`) |
| **Falsos positivos de la capa reactiva por ruido de LiDAR** | Sí (caso 2, sesión 13/07) — DECEL activo desde el segundo 1, sin obstáculo real, siempre en el mismo rango de distancia | Sí — ruido cerca del borde de `lidar_detection_range_` (4.5m), con `lidar_min_cluster_size=1` bastaba una sola celda aislada para disparar | Sí (`lidar_min_cluster_size` 1→3) | Sí | Arreglado; **descartado con el resto de la capa reactiva** |
| **A* falla justo tras el despegue (nuevo, 23/07)** | Sí | Parcialmente — se sospecha que celdas sin explorar (`-1`) cerca del dron nada más despegar hacen fallar a A*, disparando `is_intermediate_goal_` sin que haya ningún obstáculo real, lo que además ciega la detección dinámica (mismo mecanismo de agujero que el de MAP_CHECK arriba) | **No** — el intento de arreglo (`unknown_as_free=true`) se probó y se revirtió porque introducía un problema peor: A* oscilando sin converger cerca de obstáculos reales, con una colisión observada | No | **Bloqueado**, sin solución a 23/07 |

---

## 4. Decisiones de diseño y sus motivos

### 4.1 Añadir un guardia de proximidad omnidireccional, separado del cono frontal (12/07)

El cono de detección original solo mira en la dirección de vuelo (±45° sobre el yaw). Durante
un rodeo alrededor de un obstáculo, ese obstáculo queda al lado o detrás del dron —
estructuralmente invisible para un cono frontal, sea cual sea su ángulo. Se añadió
`evaluate_proximity_guard()`, una comprobación en 360° sobre el mapa acumulado (no el
instantáneo), sin exigir que el obstáculo sea "nuevo" ni persistencia entre ticks: si ya está
mapeado, no hace falta esperar a confirmarlo dos veces.

### 4.2 Separar qué bloquea a `MAP_CHECK` (el latch del cono, no el guardia) (12/07)

En un primer momento, cualquier modo reactivo activo (incluido el guardia nuevo) bloqueaba a
`MAP_CHECK` para evitar carreras de cancelación. Se detectó que esto podía dejar a `MAP_CHECK`
bloqueado para siempre, porque el guardia de proximidad no tiene mecanismo de expiración propio
(a diferencia del latch del cono frontal, que sí caduca a los 4s). Se cambió la condición para
que solo el latch del cono frontal bloquee `MAP_CHECK`.

### 4.3 Reiniciar desde el commit del 06/07 en vez de seguir sobre el trabajo del 12-13/07 (13/07)

Decisión explícita de Irene, documentada en memoria: "demasiados parches encadenados —
freezes, reject-storms, colapsos de altitud". Pese a que el trabajo del 12-13/07 se validó con
éxito (las tres condiciones pedidas a la vez, en una sola ejecución), se consideró que la
arquitectura de la capa reactiva había crecido a base de parche sobre parche sin un diseño de
fondo, y se prefirió repensarla desde una base más limpia antes que seguir acumulando arreglos.
El trabajo antiguo se aparcó (rama `feat/lidar-reactive-safety`, más una rama de checkpoint
`feat/lidar-reactive-v2` que nunca llegó a recibir commits) en vez de borrarse.

### 4.4 Pivote de fondo: atacar el replan en sí (`modify()`), no añadir más capas de seguridad (22/07)

El cambio de rumbo más importante del periodo. En vez de seguir añadiendo mecanismos reactivos
sobre el LiDAR crudo (que es lo que se había hecho desde el 30/06), se identificó que el
verdadero origen del giro brusco —el síntoma que motivó buena parte de la capa reactiva como
mitigación— estaba en cómo `MAP_CHECK` reemplazaba la ruta durante un replan: cancelaba el
`FollowPath` en curso y mandaba uno nuevo desde cero, parando en seco al dron cada vez. Usar el
servicio `modify()` en su lugar (una vez arreglado su propio bug de IDs) permite que el dron
siga volando su segmento actual mientras la ruta nueva toma efecto, sin parada brusca. Con este
cambio, la razón de ser original de la capa reactiva (cubrir el hueco de tiempo entre un
obstáculo detectado y una parada en seco) deja de aplicar de la misma forma, y se retira toda la
capa por completo en el mismo commit siguiente (`eab9f3de`).

### 4.5 Revertir `unknown_as_free=true` pese a que arreglaba el síntoma que motivó probarlo (23/07)

Se probó tratar las celdas sin explorar (`-1`) como libres para A*, porque eso evitaba el fallo
espurio de A* justo tras el despegue. Pero se detectó que, cerca de obstáculos reales, esto
producía que A* oscilara entre rutas ligeramente distintas por zonas no exploradas sin
converger, replanificando casi en cada tick — y en un caso, eso llevó a una colisión. Se
consideró que este nuevo fallo era potencialmente peor que el original, y se revirtió a la
configuración por defecto (`false`), dejando el problema original sin resolver pero sin
introducir uno nuevo.

---

## 5. Reuniones con el tutor en este periodo

Ninguna documentada entre el 29/06 y el 23/07. El trabajo de este periodo —incluida la decisión
de implementar, validar y después abandonar la capa reactiva LiDAR, y el posterior pivote a
`modify()`— es enteramente autónomo, sin punto de control intermedio con el tutor.

---

## 6. Qué queda pendiente o bloqueado a 23/07/2026

- **A* falla justo tras el despegue.** Bloqueado: el único arreglo probado
  (`unknown_as_free=true`) introduce un riesgo de colisión distinto (replanificación no
  convergente cerca de obstáculos reales). Instrumentación de diagnóstico (`[LAT] ASTAR_FAIL`,
  `[LAT] REPLAN_START`, etc.) añadida el 23/07 pero sin commitear — sigue en el árbol de trabajo.
- **Sin validar tras el pivote:** ningún caso de test (1-13 de `mission_nav_test.py`, 1-8 de v2,
  1-8 de v3) se ha vuelto a correr contra la rama `feat/replan-modify-waypoints` desde que se
  retiró la capa reactiva y se cambió el mecanismo de replan. El fix del giro brusco se considera
  "resuelto" por revisión de código y lógica, no por una batería de regresión documentada.
- **El cono frontal M_t nunca se diagnosticó del todo.** Aunque ya no importa para la rama
  activa (se retiró toda la capa reactiva que lo usaba), sigue sin saberse por qué, tras
  corregir la carrera de escritura en `scan2occ_grid.cpp`, el cono seguía sin registrar ningún
  hit en ningún test — quedó como pregunta abierta, no como bug cerrado.
- **Trabajo de la sesión 13/07 no recuperable fácilmente.** Los 5 fixes validados con éxito
  (freeze, SUCCESS falso, giro brusco mitigado, detección de choque, escalado de retreat) solo
  existen como diffs no commiteados sobre una rama que ya no es la activa; no se ha confirmado
  que estén preservados en ningún stash o rama de respaldo accesible ahora mismo.
- **Nada de este periodo confirmado en `origin`.** No se ha verificado en este informe si
  `feat/replan-modify-waypoints`, `feat/lidar-reactive-safety` o los commits de `sherec` se han
  empujado a sus remotos — solo `feat/lidar-reactive-safety` aparece también como rama remota en
  el repositorio local, el resto son ramas exclusivamente locales a día de hoy.

---

## 7. Archivos modificados en este periodo

### `aerostack2` (rutas relativas a `as2_behaviors/as2_behaviors_path_planning/` salvo que se indique otra cosa)

| Archivo | Tipo | Qué cambió a grandes rasgos |
|---|---|---|
| `src/path_planner_behavior.cpp` | C++ — behavior | Capa reactiva LiDAR completa: añadida (30/06–13/07), luego eliminada (22/07, -290 líneas); mecanismo de replan reescrito para usar `modify()`; instrumentación `[LAT]` sin commitear (23/07) |
| `include/as2_behaviors_path_planning/path_planner_behavior.hpp` | C++ — header | Miembros de la capa reactiva añadidos y luego retirados junto con el `.cpp` |
| `config/behavior_default.yaml` | YAML — config | Parámetros de la capa reactiva añadidos, ajustados varias veces (`safety_distance` 0.5→0.8, `proximity_guard_radius` 1.2→1.8, `lidar_min_cluster_size` 1→3) y finalmente eliminados con el resto |
| `../../as2_map_server/plugins/scan2occ_grid/src/scan2occ_grid.cpp` | C++ — mapeo | Fix de la carrera de escritura M_t (dos pasadas en vez de una); este fix sobrevive en la rama abandonada, no se ha confirmado si se portó a la rama activa |
| `plugins/a_star/src/a_star.cpp` | C++ — algoritmo | Sin cambios propios documentados hasta el 23/07: instrumentación de diagnóstico `[LAT] ASTAR_FAIL` añadida sin commitear |
| `plugins/a_star/src/a_star_searcher.cpp` | C++ — algoritmo | Prueba de `unknown_as_free=true` (23/07), revertida a `false`; el comentario explicando por qué queda en el código sin commitear |
| `plugins/follow_path_behavior/plugins/follow_path_plugin_position.cpp` | C++ — controlador (upstream) | Fix de terceros (Davide Torielli, 14/07) sobre el punto de `modify()`, base necesaria para el pivote del 22/07 |

### `project_sherec_navigation`

| Archivo | Tipo | Qué cambió a grandes rasgos |
|---|---|---|
| `mission/mission_nav_test.py` | Python — tests | Casos 6-9 y 12 rediseñados con obstáculos a distancia realista (06/07) |
| `mission/test_sharp_turn.py` | Python — tests (nuevo) | Diagnóstico aislado del giro brusco, sin pasar por A*/LiDAR (06/07) |
| `mission/mission_nav_test_v3.py` | Python — tests | Detección de STUCK/CRASH añadida en la sesión del 13/07 (sin commitear); offset de obstáculo configurable |
| `doc/problema_retraso_mapa_probabilistico.md` | Markdown — análisis | Corregido tras verificación empírica: el retraso no es acumulación probabilística, es latencia de spawn/física de Gazebo (09/07) |
| `measure_map_delay.py` | Python — herramienta (nuevo) | Medición aislada del mecanismo de actualización del mapa (09/07) |
| `docker/Dockerfile` | Docker — entorno | Instalación de PlotJuggler (09/07) |
| `stop.bash`, `~/.claude/commands/run-mission.md` | Bash / skill | Tres fixes de arranque/parada del stack (23/07, sin commitear en `stop.bash`) |
| `tmuxinator/aerostack2.yml` | YAML — config | Auto-lanzamiento de `run_latency_study.sh` comentado (23/07) |
| `latency_study_mt_mg.py`, `latency_study_replan_v3case5.py` | Python — herramientas (nuevos) | Estudios de latencia M_t/M_g y de replan real, de la sesión del 12/07 (sin commitear) |

---

*Informe generado el 24/07/2026. Continúa a `retrospectiva_tfm_navegacion_reactiva.md` (hasta
28/06/2026). Fuentes: `git log --all --since=2026-06-29 --until=2026-07-24` en `aerostack2` y
`project_sherec_navigation`, memoria de sesiones de trabajo, lectura directa del árbol de trabajo
a fecha del informe.*
