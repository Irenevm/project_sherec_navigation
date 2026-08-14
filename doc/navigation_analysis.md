# Análisis técnico de la capa de navegación reactiva — aerostack2

> **Documento vivo.** Este fichero se actualiza cada vez que se ejecutan nuevas pruebas o se
> obtienen nuevas conclusiones. **Nunca se borra información previa** — solo se añade o se corrige
> explícitamente indicando qué cambió y por qué. Última actualización: 2026-07-24.
>
> Fuente de datos: `master_dataset.csv` / `master_dataset.json` (120 repeticiones consolidadas,
> 103 válidas para conclusiones / 17 descartadas — ver sección "Pruebas descartadas").
> **Actualización 2026-07-29**: añadido el reparto físico-puro vs software dentro del 80%
> "margen insuficiente", y su relación con los parámetros reales de la capa reactiva LiDAR
> implementada esa semana — ver "Actualización 2026-07-29" al final del documento.

---

# Contexto

## Problema

La rama `feat/replan-modify-waypoints` de `aerostack2` eliminó la capa reactiva basada en LiDAR que
existía anteriormente (commit `eab9f3de`) y sustituyó el mecanismo de reacción a obstáculos por un
replanteo vía `FollowPath.modify()` (commit `f5cde961`), apoyado en el comportamiento existente
`as2_behaviors_path_planning` (planificador A* + `MAP_CHECK`). Antes de diseñar una **nueva** capa
reactiva, es necesario caracterizar el sistema actual: cuánto tarda en darse cuenta de que hay un
obstáculo dinámico en su camino, cuánto tarda en replanificar, y —lo más importante para el
diseño— **a partir de qué distancia un obstáculo que aparece de repente delante del dron ya no se
puede evitar**, aunque el algoritmo funcione perfectamente.

## Qué queríamos demostrar

1. La **distancia mínima real** (`Dmin`) a la que se puede colocar un obstáculo dinámico en la
   trayectoria, por velocidad de crucero, sin que el dron colisione.
2. El **desglose de la cadena de latencia completa**: desde que el LiDAR ve físicamente el
   obstáculo hasta que el dron empieza a reaccionar, para saber en qué eslabón se pierde más
   tiempo y así decidir dónde actuar en la futura capa reactiva.
3. Si el sistema actual (planificador A* + `MAP_CHECK` + `modify()`) tiene algún **bug o
   comportamiento inesperado** que deba documentarse y reportarse, independientemente del estudio
   de latencia en sí.

## Hipótesis iniciales

- H1: la latencia total antes de empezar a esquivar estaría dominada por el periodo del timer
  `MAP_CHECK` (0.5 s), ya que solo revisa el plan cada medio segundo.
- H2: a mayor velocidad, la distancia mínima segura crecería aproximadamente proporcional a la
  velocidad, con un margen de tiempo fijo (`Dmin(v) ≈ v · T_reacción_constante`).
- H3: el LiDAR y el marcado de celdas serían rápidos (sub-0.5 s) y no el cuello de botella.

Como se detalla en `# Análisis técnico`, **H1 resultó ser incorrecta en su forma simple** (el
cuello de botella real no es el periodo del timer sino un requisito estructural de distancia
recorrida) y **H2 resultó ser falsa** (`Dmin(v)` no escala linealmente con un margen de tiempo
constante — el margen de tiempo disponible en el punto de corte *decrece* con la velocidad).

---

# Cronología completa de experimentos

Cada entrada resume una **batería** (conjunto de repeticiones con la misma configuración). Los
identificadores (`label`) referencian filas concretas de `master_dataset.csv`. Las baterías
`fixval`, `diag` y `clean` se ejecutaron mientras estaba activo un cambio experimental del
planificador (`unknown_as_free=true` en `gridToImg()`) que después se **revirtió**; esas baterías
se documentan aquí por completitud pero se excluyen de las conclusiones (ver
`# Pruebas descartadas`).

### E0 — `pilot` (3 reps): validación mecánica del mecanismo drop/set_pose
- **Objetivo:** comprobar que el mecanismo de "spawnear una vez a gran altura y teletransportar
  con `SetEntityPose`" funciona sin lag ni fantasma en el mapa antes de fiarse de él para el
  estudio.
- **Configuración:** caso `recto`, v=1.5 m/s, `drop_dist` variable (sin distancia registrada en
  `piloto1`/`piloto3`; `pilotoclean`=3.11 m, `pilotoclean2`=3.77 m).
- **Resultado:** mecanismo validado — el obstáculo aparece exactamente donde se pide, sin
  fantasma previo en el mapa. `pilotoclean` → SUCCESS; `pilotoclean2` → COLLISION (primer indicio
  del bug "frontier-blind", ver `# Bug encontrado`).
- **Hipótesis obtenida:** el mecanismo de teletransporte es fiable y reutilizable para todo el
  resto del estudio.

### E1 — `batch1` / `batch2` (5+5 reps): batería inicial recto, v=1.5, drop=4.0 m nominal
- **Objetivo:** primera batería sistemática con repeticiones, para ver variabilidad con un margen
  "holgado" (4 m nominal).
- **Configuración:** caso `recto`, v=1.5 m/s, `drop_dist` nominal 4.0 m (distancia real entre
  3.18 y 3.83 m por el desfase entre el instante de la orden `set_pose` y el instante real de
  teletransporte).
- **Cambios respecto a E0:** entre `batch1` y `batch2` se corrigió el bug del **executor
  duplicado** en `mission_latency_study.py` (ver `# Bug encontrado` operativo #1), que congelaba
  `drone.position` bajo carga.
- **Resultado:** 8 SUCCESS, 2 COLLISION (ambas con `frontier_blind_at_drop=True`), 1 ABORTED
  (artefacto de una condición de carrera al comprobar `is_running()`, corregida después).
- **Hipótesis obtenida:** a 3.2–3.8 m de margen y v=1.5 m/s el sistema evita el obstáculo en la
  mayoría de los casos, pero hay colisiones "inesperadas" que correlacionan con
  `frontier_blind=True` → primera pista del bug de bloqueo de detección en modo frontera.

### E2 — `matA` (10 reps): confirmación en otras velocidades/distancias
- **Objetivo:** extender la exploración a v=0.5 y v=1.0 con distancias en la zona de transición.
- **Configuración:** v=0.5 d≈1.5 m (5 reps) y v=1.0 d≈2.5 m (5 reps).
- **Resultado v=0.5:** 4 COLLISION + 1 SUCCESS (1.42 m) muy cerca del límite.
- **Resultado v=1.0:** 5 SUCCESS, dos de ellas (`d2.5_rep1`, y una repetida en `orig`) con
  `frontier_blind=True` — es decir, "sobrevivieron" en parte porque la detección estaba
  desactivada, no porque hubiera margen real.
- **Hipótesis obtenida:** el margen de transición para v=0.5 está alrededor de 1.4 m, y para
  v=1.0 alrededor de 2.0–2.5 m; hace falta un barrido más fino.

### E3 — `matB` (12 reps): barrido de distancia a v=1.5
- **Objetivo:** acotar la zona de transición a v=1.5 con más resolución que `batch1`/`batch2`.
- **Configuración:** caso `recto`, v=1.5, `drop_dist` nominal 1.5/2.0/2.5/3.0 m (3 reps cada uno).
- **Resultado:** COLLISION en todo el rango 1.08–2.71 m salvo mezcla de `frontier_blind=True` en
  varios puntos intermedios (1.60, 1.85, 1.92, 2.71 m) — es decir, en la zona 1.5–3.0 m casi todas
  las repeticiones colisionan, con o sin bloqueo de frontera.
- **Hipótesis obtenida:** a v=1.5 el margen mínimo real está más allá de 3.0 m, más alto que lo
  que sugería `batch1`/`batch2` en solitario.

### E4 — Investigación del bug "frontier-blind" (mecanismo, no batería de datos)
- Se observó que ciertas colisiones y SUCCESS "casuales" correlacionaban con un estado interno
  `is_intermediate_goal_=true` (modo "frontera"), en el que tanto el chequeo de segmento
  (`Hito B segmento`) como el de bloqueo total (`Hito B bloqueo total`) de `MAP_CHECK` quedan
  **desactivados** — el sistema es arquitectónicamente ciego a nuevos obstáculos mientras vuela
  hacia una meta de frontera.
- Se instrumentó el código (`[LAT] FRONTIER_ENTER` / `FRONTIER_EXIT`) para poder marcar, en cada
  test, si el dron estaba en modo frontera en el instante del `drop`.
- Se investigó la causa raíz: `gridToImg()` en `a_star_searcher.cpp` trata las celdas
  desconocidas (-1, aún no vistas por el LiDAR) como **ocupadas** por defecto
  (`unknown_as_free=false`). Justo tras despegar, con el mapa aún sin explorar del todo, A* puede
  fallar en encontrar cualquier camino ("Path to goal not found") sin que exista ningún obstáculo
  real, lo que dispara el modo frontera de forma espuria.
- **Intento de fix:** cambiar a `unknown_as_free=true`. Redujo la incidencia del modo frontera
  espurio, pero introdujo un efecto secundario nuevo y peor: A* empezó a oscilar/replanificar casi
  en cada tick (~cada 0.4–0.5 s) cerca de obstáculos reales, porque encuentra rutas "optimistas"
  ligeramente distintas cada vez a través de territorio desconocido ahora transitable, sin
  converger nunca — observado en directo en RViz por la usuaria, con riesgo de colisión por esa
  inestabilidad.
- **Decisión:** revertir el fix y medir el sistema tal cual está en producción (`unknown_as_free`
  vuelve a `false`), documentando la causa raíz para futuro trabajo, sin arriesgar una regresión
  distinta. Las baterías `fixval`/`diag`/`clean` (ver más abajo) se ejecutaron con el fix aún
  activo y se descartan de las conclusiones por este motivo.

### E5 — `fixval` (5 reps) + `diag` (4 reps): validación del fix experimental — DESCARTADAS
- **Objetivo (en su momento):** confirmar que `unknown_as_free=true` resolvía el modo frontera
  espurio, y diagnosticar con detalle (`[LAT] ASTAR_FAIL`) los pocos casos que seguían fallando.
- **Resultado:** confirmó que el fix reduce el bloqueo espurio, pero introduce la inestabilidad de
  replanteo descrita en E4. **No se usan para conclusiones del estudio de latencia/Dmin** porque
  el comportamiento del planificador no es el que está en producción.

### E6 — `clean` (8 reps): primeros intentos de la batería final — DESCARTADAS (parcialmente)
- Las primeras 7–8 repeticiones de lo que iba a ser la batería final a v=1.5 se lanzaron **antes**
  de revertir el fix experimental, así que quedan en el mismo saco de "descartadas".

### E7 — `orig` (27 reps): batería final, sistema original (revertido)
- **Objetivo:** batería final para caracterizar `Dmin(v)` y la cadena de latencia con el
  planificador tal cual está en producción (`unknown_as_free=false`, sin ningún fix aplicado).
- **Configuración:** caso `recto`; v=0.5 (`d≈1.0/1.2/1.4` m, 9 reps), v=1.0 (`d≈1.0/1.5/2.0` m,
  9 reps), v=1.5 (`d≈2.8/3.0/3.2` m, 9 reps) — rangos elegidos para acotar finamente la zona de
  transición encontrada en E1–E3.
- **Resultado:** ver tablas completas en `# Resultados`. Esta es la batería principal usada para
  fijar `Dmin(0.5)` y `Dmin(1.0)`, y junto con `matB`/`batch1`/`batch2` para `Dmin(1.5)`.
- **Bugs operativos corregidos durante esta batería:** condición de carrera en `is_running()`
  (bug operativo #5), sesgo de `t_drop` medido en el ACK del servicio en vez de en el envío (bug
  operativo #4), bug de auto-coincidencia de `pkill -f` (bug operativo #6, ver más abajo) que
  probablemente explica los "Gazebo colgado" observados dos veces durante esta fase.

### E8 — `hi` (18 reps): extensión de la batería final a distancias mayores
- **Objetivo:** extender v=0.5 y v=1.0 a distancias mayores para encontrar el límite superior
  limpio de la zona de transición (confirmar dónde el SUCCESS se vuelve consistente).
- **Configuración:** v=0.5, `d≈1.6/1.8/2.0` m (9 reps); v=1.0, `d≈2.2/2.5/2.8` m (9 reps).
- **Resultado:** confirma que, por encima de `Dmin`, el resultado es consistentemente SUCCESS sin
  ninguna colisión adicional en el rango probado.

### E9 — `d` (24 reps, 23 con resultado): sonda de detección pura (dron casi estático)
- **Objetivo:** aislar la pregunta "¿cuánto tarda el sistema en *saber* que hay un obstáculo?" de
  la dinámica de vuelo/evitación, para poder medir `L_lidar`/`L_cell`/`L_mapcheck` sin que la
  velocidad de crucero ni el riesgo de colisión influyan.
- **Configuración:** script nuevo `mission_detection_probe.py` — el dron despega y mantiene un
  `navigate_to` muy lento (`NAV_SPEED=0.3` m/s) hacia una meta lejana solo para mantener activo
  el timer de `MAP_CHECK`; el obstáculo se suelta 2 s después de iniciar el vuelo (el dron apenas
  se ha desplazado, así que la distancia de soltado ≈ distancia real medida). Distancias probadas:
  1, 2, 3, 4, 5, 6, 7, 8 m (3 reps cada una, salvo `d6` con 2 reps válidas — una tuvo timeout).
- **Bugs operativos corregidos durante el desarrollo de este script:** topic de mapa sin
  namespace (bug #2), medición sobre el centro ocluido de la caja en vez de la cara (bug #3),
  desajuste sim-time/wall-time en la ventana de captura (bug #7), pérdida de mensajes en ráfaga por
  suscripción a `/rosout` — resuelto leyendo el fichero de log directamente (bug #8).
- **Resultado:** ver tabla completa en `# Resultados` y análisis en `# Análisis técnico`.
  Hallazgo clave: `L_lidar` y `L_cell` se mantienen aproximadamente constantes hasta 4 m
  (`L_lidar≈1.0–1.5s`, `L_cell≈0.3–0.5s`), pero a partir de 5 m `L_lidar` empieza a crecer
  (4.3–5.2 s) y en 6–8 m la heurística basada en CSV (`compute_lidar_sees`) **no logra
  determinarlo en absoluto** (`L_lidar=None`), aunque el sistema sí termina detectando el
  obstáculo (`L_mapcheck` sigue teniendo valor, 4.3–6.0 s). Ver hipótesis en
  `# Análisis técnico` y `# Trabajo futuro`.

### E10 — `pkillfix` (validación operativa, no incluida en `Dmin`)
- Prueba puntual para confirmar que el fix del bug de auto-coincidencia de `pkill -f` (bug
  operativo #6) realmente evita los cuelgues de limpieza. No aporta datos de latencia/colisión y
  no se usa en las tablas de `Dmin`.

---

# Pruebas descartadas

Estas baterías se ejecutaron mientras el planificador tenía aplicado el cambio experimental
`unknown_as_free=true` en `gridToImg()` (`a_star_searcher.cpp`), que **se revirtió** después de
observar una inestabilidad de replanteo no deseada (ver E4). El comportamiento medido en estas
pruebas **no corresponde al sistema en producción**, así que quedan excluidas de toda conclusión
sobre `Dmin(v)`, latencias o el bug de frontera. Se conservan aquí por trazabilidad y para que
quien retome el fix experimental en el futuro tenga datos de referencia.

| Batch | Reps | Resultado agregado | Nota |
|---|---|---|---|
| `fixval` | 5 | 3 SUCCESS, 1 COLLISION, 1 ABORTED | validación inicial del fix |
| `diag` | 4 | 1 COLLISION visible + 3 más con `ASTAR_FAIL` diag | diagnóstico con fix activo |
| `clean` | 8 | 5 SUCCESS, 3 COLLISION | primeros intentos de la batería final, aún con fix activo |

Total descartado: **17 repeticiones** (5+4+8), coherente con el conteo del dataset maestro
(103 válidas + 17 descartadas = 120).

---

# Resultados

## Tendencias generales

- **Velocidades seguras vs conflictivas:** no existe una velocidad "segura" ni "conflictiva" en
  abstracto — lo que existe es una `Dmin(v)` (distancia mínima de aparición del obstáculo) que
  crece con la velocidad, pero **no proporcionalmente**: el margen de tiempo disponible en el
  punto de corte se reduce a medida que aumenta la velocidad (ver tabla de `Dmin`).
- **Influencia de la distancia:** por debajo de `Dmin(v)` la colisión es prácticamente
  determinista; por encima, el éxito es consistente. La zona de transición es estrecha (unos
  pocos decímetros) salvo a v=1.5, donde aparece más ruido por la interacción con el bug de
  frontera.
- **Influencia del LiDAR:** no es el cuello de botella en el rango de distancias relevante para
  evitar colisión (1–4 m) — `L_lidar` es ahí aproximadamente constante (~1.0–1.5 s desde que el
  obstáculo aparece hasta que el LiDAR lo "ve" de forma sostenida en la telemetría). Empieza a
  degradarse a partir de 5 m, y a partir de 6 m la métrica basada en CSV deja de poder medirlo,
  aunque el sistema global sigue detectando el obstáculo por otras vías.
- **Influencia del planificador:** el propio A* no es lento (`L_astar` es del orden de decenas de
  ms, ver `# Análisis técnico`); el retraso dominante está en el requisito estructural de
  `MAP_CHECK`, no en el cálculo de la ruta.
- **Influencia del mapa:** el marcado de celda ocupada (`L_cell`) es rápido y estable
  (~0.3–0.5 s) en todo el rango probado — no depende de la distancia al obstáculo.
- **Comportamiento observado:** el sistema tiene un modo de "ceguera" arquitectónica
  (modo frontera) que puede solapar con el margen de decisión y contaminar tanto colisiones como
  éxitos "casuales" — ver `# Bug encontrado`.

## Tabla completa v=0.5 m/s (23 repeticiones, ordenadas por distancia real de soltado)

| Distancia real (m) | Resultado | Frontier-blind | Batch | Label |
|---|---|---|---|---|
| 0.76 | COLLISION | No | orig | orig_v0.5_d1.0_rep3 |
| 0.83 | COLLISION | No | orig | orig_v0.5_d1.0_rep2 |
| 0.92 | COLLISION | No | orig | orig_v0.5_d1.0_rep1 |
| 0.98 | COLLISION | No | orig | orig_v0.5_d1.2_rep1 |
| 1.17 | COLLISION | No | orig | orig_v0.5_d1.2_rep3 |
| 1.18 | ABORTED (artefacto) | No | orig | orig_v0.5_d1.2_rep2 |
| 1.20 | COLLISION | No | orig | orig_v0.5_d1.4_rep2 |
| 1.23 | COLLISION | No | orig | orig_v0.5_d1.4_rep3 |
| 1.32 | COLLISION | No | orig | orig_v0.5_d1.4_rep1 |
| 1.36 | COLLISION | No | matA | matA_v0.5_d1.5_rep4 |
| 1.37 | COLLISION | No | matA | matA_v0.5_d1.5_rep1 |
| 1.38 | COLLISION | No | matA | matA_v0.5_d1.5_rep2 |
| **1.42** | **SUCCESS (anómalo)** | No | matA | matA_v0.5_d1.5_rep5 |
| 1.44 | COLLISION | No | matA | matA_v0.5_d1.5_rep3 |
| **1.45** | **COLLISION (límite)** | No | hi | hi_v0.5_d1.6_rep2 |
| 1.50 | SUCCESS | No | hi | hi_v0.5_d1.6_rep3 |
| 1.53 | SUCCESS | No | hi | hi_v0.5_d1.6_rep1 |
| 1.59 | SUCCESS | No | hi | hi_v0.5_d1.8_rep2 |
| 1.63 | SUCCESS | No | hi | hi_v0.5_d1.8_rep1 |
| 1.73 | SUCCESS | No | hi | hi_v0.5_d1.8_rep3 |
| 1.81 | SUCCESS | No | hi | hi_v0.5_d2.0_rep2 |
| 1.86 | SUCCESS | No | hi | hi_v0.5_d2.0_rep3 |
| 2.00 | SUCCESS | No | hi | hi_v0.5_d2.0_rep1 |

**Dmin(0.5) ≈ 1.45–1.50 m** (última colisión limpia a 1.45 m, zona SUCCESS consistente desde
1.50 m; el SUCCESS a 1.42 m es ruido aislado no atribuible a `frontier_blind`, probablemente
variabilidad normal cerca del umbral). Margen de tiempo en el corte: `1.475/0.5 ≈ 2.95 s`.

## Tabla completa v=1.0 m/s (23 repeticiones)

| Distancia real (m) | Resultado | Frontier-blind | Batch | Label |
|---|---|---|---|---|
| 0.55 | COLLISION | No | orig | orig_v1.0_d1.0_rep2 |
| 0.58 | COLLISION | No | orig | orig_v1.0_d1.0_rep1 |
| 0.78 | COLLISION | No | orig | orig_v1.0_d1.0_rep3 |
| 1.05 | COLLISION | No | orig | orig_v1.0_d1.5_rep2 |
| 1.33 | COLLISION | No | orig | orig_v1.0_d1.5_rep3 |
| 1.42 | COLLISION | No | orig | orig_v1.0_d1.5_rep1 |
| 1.67 | COLLISION | No | hi | hi_v1.0_d2.2_rep1 |
| 1.72 | COLLISION | No | orig | orig_v1.0_d2.0_rep1 |
| 1.86 | COLLISION (última limpia) | No | hi | hi_v1.0_d2.2_rep2 |
| 1.91 | SUCCESS (no fiable) | **Sí** | orig | orig_v1.0_d2.0_rep2 |
| 1.95 | SUCCESS (no fiable) | **Sí** | matA | matA_v1.0_d2.5_rep1 |
| 1.95 | SUCCESS (no fiable) | **Sí** | orig | orig_v1.0_d2.0_rep3 |
| **2.05** | **SUCCESS (primero limpio)** | No | hi | hi_v1.0_d2.2_rep3 |
| 2.09 | SUCCESS | No | hi | hi_v1.0_d2.5_rep2 |
| 2.13 | SUCCESS | No | matA | matA_v1.0_d2.5_rep5 |
| 2.14 | SUCCESS | No | matA | matA_v1.0_d2.5_rep2 |
| 2.26 | SUCCESS | No | hi | hi_v1.0_d2.5_rep3 |
| 2.32 | SUCCESS | No | matA | matA_v1.0_d2.5_rep3 |
| 2.38 | SUCCESS | No | matA | matA_v1.0_d2.5_rep4 |
| 2.50 | SUCCESS | No | hi | hi_v1.0_d2.5_rep1 |
| 2.56 | SUCCESS | No | hi | hi_v1.0_d2.8_rep1 |
| 2.63 | SUCCESS | No | hi | hi_v1.0_d2.8_rep2 |
| 2.67 | SUCCESS | No | hi | hi_v1.0_d2.8_rep3 |

**Dmin(1.0) ≈ 2.00–2.05 m** (última colisión limpia a 1.86 m; los tres SUCCESS a 1.91/1.95 m son
"suerte" con detección desactivada, no evidencia de margen real; zona SUCCESS limpia consistente
desde 2.05 m). Margen de tiempo en el corte: `2.05/1.0 ≈ 2.05 s`.

## Tabla completa v=1.5 m/s (34 repeticiones — la más ruidosa)

| Distancia real (m) | Resultado | Frontier-blind | Válida | Batch | Label |
|---|---|---|---|---|---|
| 1.08 | COLLISION | No | Sí | matB | matB_v1.5_d1.5_rep1 |
| 1.27 | COLLISION | No | Sí | matB | matB_v1.5_d1.5_rep2 |
| 1.28 | COLLISION | No | Sí | matB | matB_v1.5_d1.5_rep3 |
| 1.60 | COLLISION | Sí | Sí | matB | matB_v1.5_d2.0_rep2 |
| 1.84 | COLLISION | No | Sí | matB | matB_v1.5_d2.5_rep1 |
| 1.85 | COLLISION | Sí | Sí | matB | matB_v1.5_d2.5_rep2 |
| 1.91 | COLLISION | No | Sí | matB | matB_v1.5_d2.5_rep3 |
| 1.92 | COLLISION | Sí | Sí | matB | matB_v1.5_d2.0_rep1 |
| 1.98 | COLLISION | No | Sí | matB | matB_v1.5_d2.0_rep3 |
| 2.43 | COLLISION | No | Sí | orig | orig_v1.5_d3.2_rep1 |
| 2.49 | COLLISION | No | Sí | orig | orig_v1.5_d2.8_rep1 |
| 2.55 | COLLISION | No | Sí | orig | orig_v1.5_d3.0_rep3 |
| 2.63 | COLLISION | No | Sí | matB | matB_v1.5_d3.0_rep2 |
| 2.65 | COLLISION | No | Sí | matB | matB_v1.5_d3.0_rep1 |
| 2.71 | COLLISION | Sí | Sí | matB | matB_v1.5_d3.0_rep3 |
| 2.71 | COLLISION | No | Sí | orig | orig_v1.5_d2.8_rep3 |
| 2.77 | COLLISION | No | Sí | orig | orig_v1.5_d2.8_rep2 |
| **2.88** | **SUCCESS** | No | Sí | orig | orig_v1.5_d3.2_rep2 |
| 2.97 | COLLISION | No | Sí | orig | orig_v1.5_d3.2_rep3 |
| **2.98** | **SUCCESS** | No | Sí | orig | orig_v1.5_d3.0_rep1 |
| 2.99 | COLLISION | Sí | Sí | orig | orig_v1.5_d3.0_rep2 |
| 3.11 | SUCCESS | — | Sí | pilot | pilotoclean |
| 3.18 | SUCCESS | No | Sí | batch2 | batch2_rep5 |
| 3.26 | SUCCESS | No | Sí | batch1 | batch1_rep4 |
| 3.29 | SUCCESS | No | Sí | batch2 | batch2_rep1 |
| 3.35 | SUCCESS | No | Sí | batch1 | batch1_rep2 |
| 3.45 | SUCCESS | No | Sí | batch2 | batch2_rep4 |
| 3.46 | COLLISION | Sí | Sí | batch1 | batch1_rep3 |
| 3.51 | SUCCESS | No | Sí | batch1 | batch1_rep5 |
| 3.66 | COLLISION | Sí | Sí | batch2 | batch2_rep3 |
| 3.77 | COLLISION | (sí, ver E0) | Sí | pilot | pilotoclean2 |
| 3.83 | SUCCESS | No | Sí | batch1 | batch1_rep1 |
| 3.83 | ABORTED (artefacto) | No | Sí | batch2 | batch2_rep2 |

**Dmin(1.5) ≈ 2.88–2.98 m** para el caso limpio (no-frontera), pero esta es la frontera más
**ruidosa** de las tres: hay colisiones limpias hasta 2.97 m y una colisión con `frontier_blind`
incluso a 2.99 m y 3.46/3.66/3.77 m — es decir, el bug de frontera puede seguir provocando
colisiones muy por encima del margen "puro". Margen de tiempo en el corte limpio:
`2.9/1.5 ≈ 1.9–2.0 s`. **Esta frontera merece más repeticiones futuras para reducir el ruido**
(ver `# Trabajo futuro`).

## Resumen `Dmin(v)` y margen de tiempo

| Velocidad | Dmin (margen puro, sin bug) | Margen de tiempo en el corte |
|---|---|---|
| 0.5 m/s | ≈ 1.45–1.50 m | ≈ 2.9–3.0 s |
| 1.0 m/s | ≈ 2.00–2.05 m | ≈ 2.0–2.05 s |
| 1.5 m/s | ≈ 2.88–2.98 m | ≈ 1.9–2.0 s |

**Conclusión clave (refuta H2):** el margen de tiempo disponible en el punto de corte **no es
constante** — baja de ~3.0 s a v=0.5 hasta ~1.9–2.0 s a v=1.5. Esto es coherente con lo que se
describe en `# Análisis técnico`: parte de la cadena de detección (`L_mapcheck`) es en realidad un
**requisito de distancia recorrida** (1.5 m desde el último replan) convertido en tiempo por la
velocidad — a mayor velocidad, ese tramo se recorre más rápido en tiempo, así que el margen total
de tiempo se reduce aunque el margen de distancia crezca.

## Atribución de causa de cada colisión (margen puro vs. bug frontier-blind)

Sobre las 44 colisiones válidas del dataset (baterías no descartadas), se clasificó cada una según
si el dron estaba en modo frontera (`frontier_blind_at_drop`) en el instante de soltar el
obstáculo y, si no lo estaba, si la distancia real de soltado caía por debajo del `Dmin(v)` limpio
de esa velocidad (ver tabla anterior) — es decir, si había o no margen de distancia/tiempo
suficiente en teoría.

| Causa | Nº colisiones | % | Interpretación |
|---|---|---|---|
| **Margen puro insuficiente** (`frontier_blind=False`, `d < Dmin(v)`) | 35 | ~80% | "Físicamente imposible" — la cadena de latencia (dominada por el calentamiento de 1.5 m de `MAP_CHECK`) no cabía en la distancia disponible, aunque el sistema funcionara perfectamente |
| **Bug frontier-blind** (`frontier_blind=True`) | 7 | ~16% | "Falló el algoritmo" — había margen de sobra, pero `MAP_CHECK` estaba desactivado por completo en modo frontera |
| Caso límite ambiguo (`frontier_blind=False`, `d` justo por encima de `Dmin(v)`, en zona ruidosa) | 1 | ~2% | ruido estadístico en el borde, no atribuible con confianza a ninguna de las dos causas |
| Sin dato de `frontier_blind` (log no conservado) | 1 | ~2% | prueba piloto temprana (`pilotoclean2`), anterior a instrumentar `[LAT] FRONTIER_ENTER/EXIT` |

**Conclusión:** la inmensa mayoría de los choques (4 de cada 5) son de la primera categoría — no
son evitables sin cambiar algo estructural (adelantar la detección, reducir el requisito de
calentamiento, etc.), no son un fallo del software tal como está diseñado hoy. El bug
frontier-blind es una causa real pero minoritaria (~1 de cada 6): explica por qué a veces choca
*a pesar de* tener margen de sobra, no por qué existe una `Dmin(v)` en primer lugar.

### Desglose completo por prueba (trazabilidad)

**Margen puro insuficiente (`frontier_blind=False`, por debajo de `Dmin(v)`) — 35 pruebas**

| Label | v (m/s) | Distancia real (m) | Batch |
|---|---|---|---|
| orig_v0.5_d1.0_rep3 | 0.5 | 0.76 | orig |
| orig_v0.5_d1.0_rep2 | 0.5 | 0.83 | orig |
| orig_v0.5_d1.0_rep1 | 0.5 | 0.92 | orig |
| orig_v0.5_d1.2_rep1 | 0.5 | 0.98 | orig |
| orig_v0.5_d1.2_rep3 | 0.5 | 1.17 | orig |
| orig_v0.5_d1.4_rep2 | 0.5 | 1.20 | orig |
| orig_v0.5_d1.4_rep3 | 0.5 | 1.23 | orig |
| orig_v0.5_d1.4_rep1 | 0.5 | 1.32 | orig |
| matA_v0.5_d1.5_rep4 | 0.5 | 1.36 | matA |
| matA_v0.5_d1.5_rep1 | 0.5 | 1.37 | matA |
| matA_v0.5_d1.5_rep2 | 0.5 | 1.38 | matA |
| matA_v0.5_d1.5_rep3 | 0.5 | 1.44 | matA |
| hi_v0.5_d1.6_rep2 | 0.5 | 1.45 | hi |
| orig_v1.0_d1.0_rep2 | 1.0 | 0.55 | orig |
| orig_v1.0_d1.0_rep1 | 1.0 | 0.58 | orig |
| orig_v1.0_d1.0_rep3 | 1.0 | 0.78 | orig |
| orig_v1.0_d1.5_rep2 | 1.0 | 1.05 | orig |
| orig_v1.0_d1.5_rep3 | 1.0 | 1.33 | orig |
| orig_v1.0_d1.5_rep1 | 1.0 | 1.42 | orig |
| hi_v1.0_d2.2_rep1 | 1.0 | 1.67 | hi |
| orig_v1.0_d2.0_rep1 | 1.0 | 1.72 | orig |
| hi_v1.0_d2.2_rep2 | 1.0 | 1.86 | hi |
| matB_v1.5_d1.5_rep1 | 1.5 | 1.08 | matB |
| matB_v1.5_d1.5_rep2 | 1.5 | 1.27 | matB |
| matB_v1.5_d1.5_rep3 | 1.5 | 1.28 | matB |
| matB_v1.5_d2.5_rep1 | 1.5 | 1.84 | matB |
| matB_v1.5_d2.5_rep3 | 1.5 | 1.91 | matB |
| matB_v1.5_d2.0_rep3 | 1.5 | 1.98 | matB |
| orig_v1.5_d3.2_rep1 | 1.5 | 2.43 | orig |
| orig_v1.5_d2.8_rep1 | 1.5 | 2.49 | orig |
| orig_v1.5_d3.0_rep3 | 1.5 | 2.55 | orig |
| matB_v1.5_d3.0_rep2 | 1.5 | 2.63 | matB |
| matB_v1.5_d3.0_rep1 | 1.5 | 2.65 | matB |
| orig_v1.5_d2.8_rep3 | 1.5 | 2.71 | orig |
| orig_v1.5_d2.8_rep2 | 1.5 | 2.77 | orig |

**Bug frontier-blind (`frontier_blind=True`) — 7 pruebas**

| Label | v (m/s) | Distancia real (m) | Batch |
|---|---|---|---|
| matB_v1.5_d2.0_rep2 | 1.5 | 1.60 | matB |
| matB_v1.5_d2.5_rep2 | 1.5 | 1.85 | matB |
| matB_v1.5_d2.0_rep1 | 1.5 | 1.92 | matB |
| matB_v1.5_d3.0_rep3 | 1.5 | 2.71 | matB |
| orig_v1.5_d3.0_rep2 | 1.5 | 2.99 | orig |
| batch1_rep3 | 1.5 | 3.46 | batch1 |
| batch2_rep3 | 1.5 | 3.66 | batch2 |

**Caso límite ambiguo — 1 prueba**

| Label | v (m/s) | Distancia real (m) | Batch | Nota |
|---|---|---|---|---|
| orig_v1.5_d3.2_rep3 | 1.5 | 2.97 | orig | justo en el borde ruidoso de `Dmin(1.5)` (2.88–2.98 m), `frontier_blind=False` pero muy cerca del límite de margen puro |

**Sin dato de `frontier_blind` — 1 prueba**

| Label | v (m/s) | Distancia real (m) | Batch | Nota |
|---|---|---|---|---|
| pilotoclean2 | 1.5 | 3.77 | pilot | prueba piloto (E0), anterior a instrumentar `[LAT] FRONTIER_ENTER/EXIT`; observada en su momento como probable frontier-blind pero sin log que lo confirme |

> Nota de mantenimiento: esta clasificación se recalculó con el script de análisis descrito en
> `master_dataset.json`, comparando `frontier_blind_at_drop` y `drop_dist_actual` contra los
> `Dmin(v)` limpios de la tabla anterior (0.5→1.475 m, 1.0→2.05 m, 1.5→2.93 m como punto de corte
> usado para esta clasificación). Al añadir nuevas colisiones en futuras baterías, repetir esta
> clasificación y actualizar tanto las cuentas como las listas de labels.

## Tabla completa sonda de detección pura (`d`, 23 repeticiones)

| Distancia (m) | L_lidar (s) | L_cell (s) | L_mapcheck (s) |
|---|---|---|---|
| 1.09 | 1.00 | 0.40 | — |
| 1.01 | 1.44 | 0.31 | — |
| 0.84 | 1.50 | 0.33 | — |
| 1.97 | 1.13 | 0.49 | — |
| 1.87 | 1.11 | 0.49 | — |
| 1.58 | 2.01 | — | — |
| 3.10 | 1.16 | 0.50 | 6.37 |
| 2.89 | 0.77 | 0.31 | 5.20 |
| 2.93 | 0.93 | 0.36 | 5.41 |
| 3.91 | 1.16 | 0.36 | 5.33 |
| 3.81 | 1.41 | 0.58 | 4.91 |
| 3.82 | 0.96 | 0.23 | 4.98 |
| 5.00 | 5.16 | 0.36 | 6.00 |
| 4.87 | 4.63 | 0.28 | 5.34 |
| 4.78 | 4.30 | 0.36 | 4.85 |
| 5.65 | — | — | 4.36 |
| 5.60 | — | — | 4.46 |
| 6.87 | — | 0.51 | 5.33 |
| 7.07 | — | 0.35 | 6.00 |
| 6.93 | — | 0.33 | 5.67 |
| 7.88 | — | 0.45 | 5.34 |
| 7.93 | — | 0.37 | 5.57 |
| 7.73 | — | 0.49 | 4.76 |

(`d6_rep2` a 6 m sufrió timeout de captura y no tiene resultado — descartado como fallo operativo,
no como dato.)

---

# Análisis técnico

## Desglose de la cadena de latencia

La cadena completa, desde que el obstáculo aparece físicamente hasta que el dron empieza a
reaccionar, tiene estos eslabones (nombres usados en el código vía logs `[LAT]`):

1. **`L_lidar`** — desde que el obstáculo existe hasta que el LiDAR lo registra de forma sostenida
   en el escaneo (medido en la sonda vía heurística sobre `lidar_fwd_min` en el CSV de
   telemetría).
2. **`L_cell`** — desde que el LiDAR lo ve hasta que la celda del mapa de ocupación (`/map`) supera
   el umbral que A* considera "ocupado" (30, con `hit_confidence=+40` por impacto — en teoría un
   solo impacto ya marcaría la celda, la CSV mide el efecto agregado).
3. **`L_mapcheck`** — desde que la celda está ocupada hasta que `MAP_CHECK` lo detecta y dispara
   replan (log `[LAT] DETECT`).
4. **`L_replan` / `L_astar`** — tiempo de cómputo de A* tras el disparo (`[LAT] REPLAN_START` →
   `[LAT] ASTAR_DONE`).
5. **`L_modify`** — tiempo hasta que `FollowPath.modify()` acepta la nueva ruta
   (`[LAT] MODIFY_ACCEPTED`).

### `L_lidar` y `L_cell`: no son el cuello de botella (hasta ~4 m)

En el rango 1–4 m, `L_lidar` se mantiene en **~1.0–1.5 s** y `L_cell` en **~0.3–0.5 s**,
independientemente de la distancia. Esto es coherente con: el LiDAR escanea a 10 Hz (una vuelta
completa cada 0.1 s) y el `hit_confidence=+40` con umbral 30 hace que, en teoría, un único impacto
ya sería suficiente para marcar la celda — el tiempo medido (~0.3–0.5 s) probablemente refleja la
ventana de la heurística de detección sostenida más que el propio marcado físico.

**A partir de 5 m, `L_lidar` empieza a degradarse** (4.3–5.2 s) y **en 6–8 m la heurística
CSV no logra determinarlo en absoluto** (`L_lidar=None`), aunque `L_mapcheck` (que se mide de
forma independiente, vía el log de texto `[LAT] DETECT`) sigue teniendo un valor razonable
(4.3–6.0 s) — es decir, **el sistema global sí termina detectando el obstáculo a 6–8 m**, solo que
la métrica concreta `L_lidar` (basada en `lidar_fwd_min` del CSV) deja de poder aislarlo.

**RESUELTO (2026-07-24).** Se investigó inspeccionando la traza cruda de `lidar_fwd_min` en el
CSV de varias repeticiones `d5`–`d8`: el valor se mantiene **constante en ~4.2–4.3 m desde antes
incluso de soltar el obstáculo** (p. ej. ya a t=−8 s, con el dron parado en el spawn y la caja aún
aparcada a z=5 m fuera de la banda del scan). Es decir, `lidar_fwd_min` está **contaminado por una
lectura "fantasma" constante de ~4.2 m que no tiene nada que ver con la caja soltada** — y en
cuanto la caja aparece más lejos que esa lectura fantasma (a partir de d≈5 m), la heurística
`compute_lidar_sees()` no ve ningún "mínimo nuevo más cercano" hasta que la propia deriva del dron
(a 0.3 m/s) acerca la caja lo suficiente para desbancar a la lectura fantasma como mínimo — de ahí
que `L_lidar` se alargue en `d5` y directamente no se alcance a determinar en la ventana de
captura de 10 s en `d6`–`d8`.
- Se comprobó el fichero del mundo (`nav_test_world.sdf`) para descartar que fuera una pared u
  obstáculo real a esa distancia — no hay ningún modelo estático a ~4.2 m por delante del punto de
  spawn `(-9,0)` en la dirección de vuelo.
- La explicación más probable, con buen ajuste numérico, es **geométrica, no una pared**: el
  LiDAR tiene un FOV vertical de ±15° (`lidar_3d.sdf`, `min_angle=-0.2618`, `max_angle=0.2618`), y
  el anillo vertical más bajo (inclinado 15° hacia abajo) intersecta el **suelo** a una distancia
  oblicua de `altura_vuelo / sin(15°)`. Con la altura real de vuelo observada (~1.05–1.10 m,
  coherente con `TAKEOFF_HEIGHT=1.0` más el margen normal de mantenimiento de altitud), esa
  distancia sale **≈4.06–4.25 m** — coincide casi exactamente con los ~4.2–4.3 m medidos. Es decir,
  `lidar_fwd_min` no está midiendo un obstáculo delante del dron sino el propio anillo inferior del
  LiDAR "viéndose" contra el suelo, un artefacto geométrico normal de cualquier sensor con FOV
  vertical no nulo, no un fallo de la simulación ni de `MAP_CHECK`.
- **Esto no afecta a `MAP_CHECK` ni a la detección real de obstáculos:** la tubería de mapa usa un
  filtro de altura (banda 0–1.0 m) que descarta ese tipo de retornos de suelo antes de construir la
  cuadrícula de ocupación, por eso `L_mapcheck` sigue detectando la caja con normalidad en 6–8 m
  aunque la métrica auxiliar `lidar_fwd_min` del CSV de telemetría (pensada solo para diagnóstico
  visual rápido, no para alimentar al planificador) se vea "cegada" por este artefacto.
- **Consecuencia práctica:** este hallazgo confirma que **`L_lidar` medido vía `lidar_fwd_min` solo
  es fiable como métrica cuando el obstáculo de interés está más cerca que ~4 m** (el radio de la
  "lectura fantasma" de suelo a la altura de vuelo usada). Para distancias mayores hay que fiarse
  de `L_mapcheck`/`L_cell` (que sí son robustos a este artefacto) o repetir la sonda a una altura de
  vuelo mayor, donde el anillo inferior del LiDAR alcanzaría el suelo más lejos y dejaría de
  interferir en el rango de interés.
- Nótese que **esto no afecta las conclusiones de `Dmin(v)`**: en todo el rango de interés real
  para colisión (obstáculo a 1–4 m del punto de aparición) `L_lidar` es rápido, estable, y no
  contaminado por este artefacto (la caja siempre está más cerca que los 4.2 m del suelo fantasma);
  el fenómeno de 6–8 m solo importa para diseñar una capa reactiva que actúe *muy* anticipadamente.

### `L_mapcheck`: el verdadero cuello de botella — y no es lo que parecía

En la sonda (dron casi estático, `NAV_SPEED=0.3` m/s), `L_mapcheck` es consistentemente
**~4.3–6.0 s**, muy por encima del periodo del timer de `MAP_CHECK` (0.5 s). Esto en un primer
momento parecía contradecir H1 (que el cuello de botella fuera simplemente el periodo del timer).

La explicación real está en el propio código de `path_planner_behavior.cpp`: el chequeo de
segmento de `MAP_CHECK` (`Hito B segmento`) **no empieza a muestrear** hasta que el dron ha
recorrido `safety_distance_ * 3.0 = 1.5 m` desde el punto del último replan
(`dist_from_plan_start >= safety_distance_*3.0`). A `NAV_SPEED=0.3` m/s, recorrer 1.5 m tarda
`1.5/0.3 = 5.0 s` — que coincide casi exactamente con el `L_mapcheck` medido en la sonda.

**Esto reencuadra completamente el problema:** `L_mapcheck` no es una latencia de cómputo fija,
sino **una distancia mínima que hay que recorrer (1.5 m) convertida en tiempo según la
velocidad de crucero**. A velocidad de crucero normal (1.0–1.5 m/s) ese mismo 1.5 m cuesta solo
**~1.0–1.5 s**, no 5 s — lo que explica por qué en los vuelos reales (no en la sonda) el sistema sí
llega a detectar y reaccionar a tiempo en un rango de distancias mucho menor que en la sonda.

Esta es también la explicación de por qué **en la sonda los obstáculos a 1–2 m nunca se llegan a
detectar dentro de la ventana de captura**: el dron, a 0.3 m/s, nunca completa (o apenas completa)
el "calentamiento" de 1.5 m antes de alcanzar/superar el punto del obstáculo.

### `L_astar` / `L_modify`: rápidos, no relevantes como cuello de botella

El cómputo de A* y la aceptación de `modify()` son del orden de decenas/pocas centenas de
milisegundos — no aportan un retraso significativo comparados con `L_mapcheck`.

## Margen insuficiente: cuándo el algoritmo falla vs cuándo es físicamente imposible

Combinando la reformulación de `L_mapcheck` con las tablas de `Dmin(v)`:

- El "calentamiento" estructural de 1.5 m antes de que `MAP_CHECK` empiece a muestrear segmentos
  significa que, **si el obstáculo aparece a menos de esa distancia recorrida desde el último
  replan, el sistema no tiene ninguna oportunidad de detectarlo por esa vía** antes de que sea
  demasiado tarde — no es un fallo del algoritmo, es una limitación estructural del diseño actual.
- Por encima de ese umbral, el sistema si detecta a tiempo en la mayoría de los casos, y el
  margen de tiempo resultante (`Dmin(v)/v`) decrece con la velocidad, lo cual es coherente con que
  la parte "cara" de la cadena (`L_mapcheck`) se recorre más rápido a mayor velocidad, dejando
  relativamente menos margen para el resto de la cadena (replan + modify + reacción física del
  controlador).

## Limitaciones identificadas

- **LiDAR:** banda vertical solo 0–1.0 m (obstáculos fuera de esa banda no serían vistos en
  absoluto — no probado en este estudio, solo documentado del código), rango efectivo con
  degradación de detección más allá de ~5 m (ver hipótesis H-A/H-B).
- **Planificador (A*):** rápido en sí mismo, pero con la limitación conocida y ya investigada de
  `unknown_as_free=false` (ver `# Bug encontrado`) y sin recompute anticipado — solo reacciona
  cuando `MAP_CHECK` se lo indica.
- **Mapa global (`/map`, usado por A*):** requiere agregación de impactos LiDAR con
  `hit_confidence`/umbral, más lento de "confirmar" un obstáculo que el escaneo LiDAR bruto per se
  (esto se estudió también en la sesión anterior sobre `M_t` vs `M_g`, ver memoria de proyecto
  `project_map_latency_study`).
- **Mapa local / `MAP_CHECK`:** el requisito de 1.5 m de "calentamiento" tras cada replan es la
  limitación más importante encontrada — convierte el problema de "latencia de detección" en un
  problema de "distancia mínima recorrida", con implicaciones directas para el diseño de la nueva
  capa reactiva (deberá poder reaccionar sin depender de ese calentamiento).

---

# Bug encontrado

## Resumen (estilo bug-report)

**Título:** El planificador de rutas queda ciego a nuevos obstáculos mientras navega hacia una
meta de "frontera" (`is_intermediate_goal_=true`), lo que puede causar colisiones evitables.

**Cómo se manifiesta:** cuando A* no puede encontrar una ruta directa a la meta original (por
ejemplo, porque parte del mapa aún es desconocido justo tras el despegue, o porque el camino real
está bloqueado), el planificador entra en un modo "frontera": navega hacia una meta intermedia
alcanzable en vez de la meta final. **Mientras está en ese modo, las dos comprobaciones de
`MAP_CHECK` que detectarían un nuevo obstáculo en la ruta actual (chequeo de bloqueo total y
chequeo de segmento) están desactivadas** — solo se reactivan cuando se alcanza la meta de
frontera o cuando vuelve a ser posible ir directo a la meta original.

**Cuándo ocurre:** se observó tanto justo tras el despegue (causa raíz identificada:
`gridToImg()` trata las celdas desconocidas como ocupadas por defecto, lo que puede hacer fallar a
A* espuriamente en un mapa aún sin explorar) como, en al menos un caso, en pleno vuelo.

**Frecuencia:** en el dataset consolidado, de las 103 repeticiones válidas, **al menos 12** tienen
`frontier_blind_at_drop=True` en el instante de soltar el obstáculo (repartidas entre v=1.0 y
v=1.5), y de ellas varias resultan en colisión (con el obstáculo pasando completamente
desapercibido para `MAP_CHECK`) y otras en SUCCESS "casual" (el dron esquiva por geometría/suerte,
no porque el sistema lo haya detectado).

**Por qué creemos que es del planificador y no de nuestro código de misión/orquestación:** el
estado `is_intermediate_goal_` y las comprobaciones que desactiva son internos a
`path_planner_behavior.cpp` (`as2_behaviors_path_planning`), no a los scripts de misión de este
proyecto. Se instrumentó directamente ese fichero (`[LAT] FRONTIER_ENTER/EXIT`) para confirmarlo,
y la causa raíz (`gridToImg()` con `unknown_as_free=false` en `a_star_searcher.cpp`) está en el
plugin `a_star` del propio repositorio de `aerostack2`, no en el código específico del proyecto.

**Por qué merece la pena reportarlo a los tutores:** representa una ventana de vulnerabilidad real
(no solo teórica) frente a obstáculos dinámicos que aparecen mientras el dron está en modo
frontera — algo que puede ocurrir de forma rutinaria justo tras el despegue en muchos escenarios
reales (mapa aún no explorado). Además, el intento de arreglo más directo
(`unknown_as_free=true`) introduce una regresión distinta (inestabilidad de replanteo cerca de
obstáculos reales) que sugiere que el diseño de `gridToImg()`/modo-frontera necesita una revisión
más cuidadosa que un simple cambio de flag — información valiosa para quien mantenga ese plugin.

## Estado

- Causa raíz identificada y documentada.
- Fix experimental probado, validado parcialmente, y **revertido** por decisión explícita (para no
  introducir una regresión distinta sin más tiempo de análisis).
- Diagnóstico permanente añadido al código (`[LAT] ASTAR_FAIL` en `a_star.cpp`) para facilitar
  investigación futura sin tener que re-instrumentar.
- **No se ha aplicado ningún fix definitivo** — pendiente de decisión de diseño (ver
  `# Trabajo futuro`).

## ACTUALIZACIÓN 2026-07-24 — Bug corregido (fix definitivo, distinto del experimental revertido)

Tras el intento fallido de `unknown_as_free=true` (revertido, ver arriba), se aplicó un **fix
distinto y más quirúrgico**, que no toca `gridToImg()`/`unknown_as_free` en absoluto: en
`path_planner_behavior.cpp::on_run()`, el chequeo de segmento (Hito B) estaba condicionado a
`can_reach_goal && !is_intermediate_goal_ && !waiting_for_map_check_replan_` — es decir, se
desactivaba explícitamente en cuanto el dron entraba en modo frontera
(`is_intermediate_goal_=true`). El fix **elimina la condición `!is_intermediate_goal_`**: el
chequeo de segmento ahora corre siempre que no haya un replan pendiente
(`!need_replan_ && !waiting_for_map_check_replan_`), tanto en la ruta directa como en el tramo de
frontera. La justificación técnica (documentada también como comentario en el propio código): un
camino de frontera se calcula igualmente con `unknown_as_free=false`, así que nunca cruza celdas
desconocidas, solo celdas ya confirmadas como libres — `is_occupied()` en ese tramo solo puede
detectar un obstáculo genuinamente nuevo, nunca una "celda desconocida" mal interpretada. No había
ninguna razón real para que esa comprobación estuviera apagada en modo frontera; era una condición
de guarda innecesariamente amplia.

**Validación (batería de 8 repeticiones, v=1.5, d nominal 3.0/3.5m, exactamente las condiciones
que antes colisionaban solo por este bug):**

- **0 de 8 repeticiones mostraron el bug** — se inspeccionó el log crudo (`nav_planner_diag_*.log`)
  de cada una: las 4 colisiones tienen secuencia completa `[LAT] DETECT → REPLAN_START →
  MODIFY_ACCEPTED` sin ningún `[LAT] FRONTIER_ENTER` en su historial, es decir, no estaban en modo
  frontera en el momento del choque. Se confirmó además, con una traza concreta
  (`fixcheck_d3.5_rep4`), que `[LAT] DETECT` ahora sí puede dispararse con
  `intermediate=true` — algo arquitectónicamente imposible con el código anterior.
- Las 4 colisiones que persisten se explican por completo por margen físico insuficiente: la
  distancia real de soltado en esas repeticiones resultó ser 2.14–2.26 m y 2.83 m (más corta que
  la nominal por la variabilidad ya conocida del mecanismo de disparo por proximidad), muy por
  debajo o justo en el borde de `Dmin(1.5)≈2.9m` — exactamente el comportamiento "físicamente
  imposible" ya caracterizado en `# Análisis técnico`, no evidencia de que el bug persista.
- Aparecieron 2 resultados `STUCK` (dron no llega a la meta pero tampoco colisiona,
  `min_dist>1.0m` en ambos casos) — **no relacionados con este fix**: uno es el bug ya conocido de
  "Path to goal not found" espurio justo tras el despegue (el dron nunca llegó a soltar el
  obstáculo), y el otro es el bug ya aplazado de "replan agotado" (`project_deferred_bugs`,
  memoria de proyecto) — cuando ni la ruta directa ni ningún punto de frontera son alcanzables, el
  replan no tiene a dónde ir. Este segundo caso es interesante porque probablemente antes del fix
  habría acabado en colisión silenciosa (al no detectarse el obstáculo en modo frontera); ahora, al
  detectarse correctamente, el fallo se convierte en "atascado pero a salvo" en vez de "choque" —
  un efecto secundario positivo del fix, aunque el bug de fondo del replan agotado sigue sin
  arreglar y queda pendiente como su propio ítem en `# Trabajo futuro`.

**Conclusión:** el bug frontier-blind se considera **corregido y validado** con este fix. No se
observó ningún indicio de la inestabilidad de replanteo que sí apareció con el intento anterior
(`unknown_as_free=true`). Esto no afecta a la conclusión principal del estudio (el 80% de
colisiones por margen puro insuficiente, dominado por el calentamiento de 1.5m de `MAP_CHECK`,
sigue siendo la causa dominante y no ha sido tocada por este fix) — sí reduce a cero la fracción
de colisiones atribuible al ~16% de bug frontier-blind identificado en
`# Atribución de causa de cada colisión`, aunque esa reclasificación con volumen de datos grande
queda pendiente de una batería futura si se desea recalcular los porcentajes exactos.

---

# Conclusiones

## Lo que sabemos con confianza

1. `Dmin(v)` no es proporcional a la velocidad con un margen de tiempo constante — el margen de
   tiempo en el punto de corte decrece con la velocidad (de ~3.0 s a v=0.5 hasta ~1.9–2.0 s a
   v=1.5).
2. El cuello de botella dominante de la cadena de detección no es el periodo del timer de
   `MAP_CHECK` (0.5 s) ni el LiDAR/marcado de celda (ambos rápidos, <1.5 s combinados en el rango
   1–4 m), sino el requisito estructural de recorrer 1.5 m desde el último replan antes de que el
   chequeo de segmento empiece a muestrear.
3. Existe un bug real y reproducible ("frontier-blind") que desactiva la detección de nuevos
   obstáculos mientras el dron navega hacia una meta de frontera, con causa raíz identificada en
   `gridToImg()`.
4. El intento de fix directo de esa causa raíz introduce una regresión distinta (inestabilidad de
   replanteo), por lo que el problema no tiene todavía una solución simple y segura.
5. Los valores concretos de `Dmin`: **0.5 m/s → ~1.45–1.50 m**, **1.0 m/s → ~2.00–2.05 m**,
   **1.5 m/s → ~2.88–2.98 m** (esta última con más ruido que las otras dos).

## Hipótesis abiertas (no confirmadas)

- Si la frontera de `Dmin(1.5)` es genuinamente más ruidosa por razones físicas (p. ej.
  interacción entre inercia del controlador P y el margen lateral de 0.5 m de `safety_distance`) o
  simplemente necesita más repeticiones para reducir el ruido estadístico.
- Si el modo frontera puede aparecer también por causas distintas al mapa sin explorar tras el
  despegue (se observó "en al menos un caso" en pleno vuelo, sin investigar a fondo esa segunda
  vía).

## Pruebas que faltan

- Más repeticiones en la zona de transición de v=1.5 (2.6–3.1 m) para reducir el ruido.
- Un experimento dedicado a diferenciar H-A de H-B para el "hueco" de `L_lidar` en 6–8 m (ver
  `# Trabajo futuro`).
- Caracterización del caso `giro` (obstáculo tras una maniobra de cambio de rumbo) — definido en
  el plan original pero no ejecutado en esta ronda de baterías (todas las pruebas de esta
  cronología son caso `recto`).
- Medición directa de `L_react` (tiempo de respuesta física del controlador tras `modify()`
  aceptado) — identificado en el plan original como incógnita, no aislado explícitamente en las
  baterías ejecutadas.

---

# Trabajo futuro

> Esta sección se actualiza automáticamente al cerrar cada nueva batería de pruebas. No eliminar
> entradas antiguas — marcar como resuelta con fecha si corresponde.

- [x] ~~Investigar por qué `L_lidar` no es medible por encima de 6 m~~ — **Resuelto 2026-07-24**:
  es un artefacto geométrico del anillo vertical inferior del LiDAR contra el suelo (~4.2 m a la
  altura de vuelo usada), no un problema real de detección ni de `MAP_CHECK`. Ver
  `# Análisis técnico`. Pendiente opcional: confirmar visualmente en RViz/point cloud que el
  retorno "fantasma" es efectivamente del suelo (la evidencia actual es geométrica/numérica, no
  una inspección visual directa).
- [ ] Añadir más repeticiones en la zona 2.6–3.1 m a v=1.5 para estrechar el intervalo de
  `Dmin(1.5)`.
- [ ] Ejecutar el caso `giro` (definido en el plan original, sección "Entregables → 3. Misión del
  estudio") para ver el efecto del heading/maniobra en la distancia mínima segura.
- [ ] Aislar `L_react` (tiempo de respuesta física del controlador tras `MODIFY_ACCEPTED`) como
  métrica independiente.
- [ ] Decidir una estrategia de diseño para el bug frontier-blind que no reintroduzca la
  inestabilidad de replanteo observada con `unknown_as_free=true` (p. ej.: mantener
  `unknown_as_free=false` para el cálculo de ruta pero permitir que las comprobaciones de
  `MAP_CHECK` sigan activas incluso en modo frontera, ya que esas comprobaciones no dependen de si
  la celda es "desconocida" sino de si está "ocupada").
- [ ] Usar el hallazgo de "`L_mapcheck` = distancia de calentamiento / velocidad" como requisito de
  diseño explícito para la nueva capa reactiva: debe poder reaccionar sin depender de que hayan
  transcurrido 1.5 m desde el último replan.

---

# Actualización 2026-07-29 — Físico puro vs software dentro del 80% "margen insuficiente"

**Motivación:** la capa reactiva LiDAR crudo (implementada 2026-07-27/28, `danger_distance = v·(k_brake+l_detect)+margen`
con `k_brake=1.0s`, `l_detect=0.25s`, `margen=0.4m`) se diseñó para evitar colisiones donde hay margen físico
mismo pero el retraso del pipeline `L_lidar`+`L_mapcheck` lo consume. Quedaba sin responder: dentro de las
35 colisiones "margen insuficiente" (+1 caso límite ambiguo = 36 no-frontier-blind), **¿cuántas son
realmente recuperables con esa capa, cuántas son físicamente imposibles pase lo que pase, y cuántas
quedan en tierra de nadie** (retraso de software, pero un margen tan ajustado que ni la capa nueva llega)?
Esto es distinto del bug de modo frontera encontrado el 2026-07-28 (colisiones/ABORTED al navegar a una
meta fuera del mapa conocido) — ese bug se deja aparte, no se mezcla con este análisis: todas las pruebas
usadas aquí son caso `recto`, sin modo frontera.

## Método

Para cada una de las 36 colisiones válidas no-frontier-blind (`master_dataset.csv`, `outcome=COLLISION`,
`frontier_blind_at_drop=False`), se compara la distancia real a la que apareció el obstáculo
(`drop_dist_actual`, o `drop_dist_requested` si no hay valor medido) contra dos umbrales:

- **Mínimo físico puro** = `v · k_brake` — distancia de frenado asumiendo detección instantánea
  (`k_brake=1.0s`, medido en `mission_brake_test.py`, 2026-07-27). Por debajo de esto, ninguna mejora de
  software evita la colisión.
- **Umbral de la capa reactiva actual** = `v · (k_brake + l_detect) + margen` — los mismos parámetros
  reales del corredor implementado esta semana. Por encima de esto, la capa reactiva actual debería, en
  teoría, evitar la colisión.

## Resultado

| Categoría | Nº | % de 36 | Significado |
|---|---|---|---|
| Físico puro (`d < v·k_brake`) | 6 | ~17% | Imposible de evitar con cualquier mejora de detección — límite físico real. **Se queda así, documentado, sin acción.** |
| Software, pero insuficiente para el corredor actual (`v·k_brake ≤ d < umbral_corredor`) | 10 | ~28% | Evitable en teoría con detección más rápida, pero tan ajustado que ni los parámetros actuales del corredor (l_detect=0.25s, margen=0.4m) llegan a tiempo. |
| Recuperable por el corredor actual (`d ≥ umbral_corredor`) | 20 | ~56% | Esto es justo lo que la capa reactiva de esta semana está diseñada para evitar. |

Por velocidad (umbrales en metros, `v·k_brake` / `umbral_corredor`):

| v (m/s) | v·k_brake | umbral_corredor | Físico puro | Software insuf. | Recuperable |
|---|---|---|---|---|---|
| 0.5 | 0.50 | 1.02 | 0 | 4 | 9 |
| 1.0 | 1.00 | 1.65 | 3 | 3 | 3 |
| 1.5 | 1.50 | 2.27 | 3 | 3 | 8 |

Desglose completo (label, v, distancia real de soltado):

```
label                              v  d_real  v·k_brake  umbral_corredor  categoría
orig_v0.5_d1.0_rep3              0.5    0.76      0.50      1.02  software_insuficiente
orig_v0.5_d1.0_rep2              0.5    0.83      0.50      1.02  software_insuficiente
orig_v0.5_d1.0_rep1              0.5    0.92      0.50      1.02  software_insuficiente
orig_v0.5_d1.2_rep1              0.5    0.98      0.50      1.02  software_insuficiente
orig_v0.5_d1.2_rep3              0.5    1.17      0.50      1.02  recuperable
orig_v0.5_d1.4_rep2              0.5    1.20      0.50      1.02  recuperable
orig_v0.5_d1.4_rep3              0.5    1.23      0.50      1.02  recuperable
orig_v0.5_d1.4_rep1              0.5    1.32      0.50      1.02  recuperable
matA_v0.5_d1.5_rep4              0.5    1.36      0.50      1.02  recuperable
matA_v0.5_d1.5_rep1              0.5    1.37      0.50      1.02  recuperable
matA_v0.5_d1.5_rep2              0.5    1.38      0.50      1.02  recuperable
matA_v0.5_d1.5_rep3              0.5    1.44      0.50      1.02  recuperable
hi_v0.5_d1.6_rep2                0.5    1.45      0.50      1.02  recuperable
orig_v1.0_d1.0_rep2              1.0    0.55      1.00      1.65  fisico_puro
orig_v1.0_d1.0_rep1              1.0    0.58      1.00      1.65  fisico_puro
orig_v1.0_d1.0_rep3              1.0    0.78      1.00      1.65  fisico_puro
orig_v1.0_d1.5_rep2              1.0    1.05      1.00      1.65  software_insuficiente
orig_v1.0_d1.5_rep3              1.0    1.33      1.00      1.65  software_insuficiente
orig_v1.0_d1.5_rep1              1.0    1.42      1.00      1.65  software_insuficiente
hi_v1.0_d2.2_rep1                1.0    1.67      1.00      1.65  recuperable
orig_v1.0_d2.0_rep1              1.0    1.72      1.00      1.65  recuperable
hi_v1.0_d2.2_rep2                1.0    1.86      1.00      1.65  recuperable
matB_v1.5_d1.5_rep1              1.5    1.08      1.50      2.27  fisico_puro
matB_v1.5_d1.5_rep2              1.5    1.27      1.50      2.27  fisico_puro
matB_v1.5_d1.5_rep3              1.5    1.28      1.50      2.27  fisico_puro
matB_v1.5_d2.5_rep1              1.5    1.84      1.50      2.27  software_insuficiente
matB_v1.5_d2.5_rep3              1.5    1.91      1.50      2.27  software_insuficiente
matB_v1.5_d2.0_rep3              1.5    1.98      1.50      2.27  software_insuficiente
orig_v1.5_d3.2_rep1              1.5    2.43      1.50      2.27  recuperable
orig_v1.5_d2.8_rep1              1.5    2.49      1.50      2.27  recuperable
orig_v1.5_d3.0_rep3              1.5    2.55      1.50      2.27  recuperable
matB_v1.5_d3.0_rep2              1.5    2.63      1.50      2.27  recuperable
matB_v1.5_d3.0_rep1              1.5    2.65      1.50      2.27  recuperable
orig_v1.5_d2.8_rep3              1.5    2.71      1.50      2.27  recuperable
orig_v1.5_d2.8_rep2              1.5    2.77      1.50      2.27  recuperable
orig_v1.5_d3.2_rep3              1.5    2.97      1.50      2.27  recuperable
```

(Fuente: `master_dataset.csv`, filtrado `valid_for_conclusions=True`, `outcome=COLLISION`,
`frontier_blind_at_drop=False`. Script de clasificación ad-hoc, no guardado como fichero — reproducible
con los umbrales de arriba sobre las columnas `speed` y `drop_dist_actual`/`drop_dist_requested`.)

## Conclusión — qué cubren realmente los cambios de esta semana

Los cambios de esta semana (capa reactiva LiDAR crudo) **no atacan todo el 80%, ni tenían por qué**:

1. **17% físico puro** (6/36) — no se toca. Correcto dejarlo documentado como límite físico; ninguna
   mejora de software cambia esto.
2. **56% recuperable** (20/36) — esto es exactamente lo que la capa reactiva de esta semana se diseñó
   para evitar. Cubrirlo de verdad depende de que la capa funcione de forma fiable en todos los modos de
   vuelo — pendiente el gap de modo frontera encontrado el 2026-07-28 (aparcado, no es objeto de este
   análisis).
3. **28% software insuficiente** (10/36) — zona intermedia que **los cambios actuales no resuelven tal
   como están parametrizados**: el margen es demasiado ajustado incluso con `l_detect=0.25s`. Recuperar
   parte de esto exigiría una detección todavía más rápida o un margen más pequeño, con el riesgo de
   acercarse más al límite físico real (6% del 17% físico puro) — no es gratis.

## Trabajo futuro (añadido)

- [ ] Decidir si merece la pena estrechar `l_detect`/`margen` del corredor para recuperar parte del 28%
  "software insuficiente", y cuantificar cuánto se acercaría al límite físico real al hacerlo.
- [ ] Repetir esta misma clasificación una vez resuelto (si se resuelve) el gap de modo frontera del
  2026-07-28, para confirmar que el 56% "recuperable" se traduce en SUCCESS real y no solo teórico.
