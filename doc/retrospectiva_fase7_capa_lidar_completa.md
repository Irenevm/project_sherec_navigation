# Retrospectiva Fase 7 — Capa Reactiva LiDAR: diseño, validación y abandono (29 junio – 24 julio 2026)

**Autor:** Irene Valdés Martínez
**Fecha del informe:** 24 de julio de 2026
**Continúa a:** `doc/retrospectiva_tfm_navegacion_reactiva.md` (hasta 28/06/2026, cierre de la Fase 6 — comprobación periódica del mapa — y apertura de la Fase 7, todavía sin implementar, decidiendo entre `nav2_collision_monitor` y nodo propio).
**Fuentes de datos:** `git log --all` en `aerostack2` y `project_sherec_navigation`, memoria de sesiones de trabajo, los cuatro informes intermedios generados el 24/07 en `doc/Julio/` (fusionados y reconciliados aquí en un único documento), y las aclaraciones de Irene sobre el diseño técnico y la reunión del 13/07 que no estaban completas en los informes anteriores.

**Este documento sustituye a los cuatro ficheros de `doc/Julio/` (`2026-07-09_capa-reactiva-mt-mg-validacion-y-reversion.md`, `2026-07-12_retrospectiva-capa-reactiva-lidar.md`, `2026-07-12_retrospectiva_capa_reactiva_lidar.md`, `2026-07-24_retrospectiva-reinicio-rama-y-modify.md`), que eran variantes casi idénticas del mismo periodo generadas por sesiones distintas. Se conservan como material fuente, pero para redactar el TFM debe usarse este documento.**

---

## 1. Resumen ejecutivo

La Fase 7 tenía como objetivo cerrar el hueco de seguridad detectado al final de la Fase 6: el planificador trabaja sobre el mapa de ocupación global (M_g), que tarda en consolidar un obstáculo dinámico recién aparecido, dejando una ventana en la que el dron puede volar hacia algo que el mapa todavía no ve como ocupado.

El trabajo de este periodo se puede dividir en tres etapas:

1. **Diseño y primera implementación de una capa reactiva LiDAR (30/06 – 13/07).** Se implementó una capa de seguridad independiente del ciclo de planificación normal, basada en comparar un mapa instantáneo por medida (M_t) contra el mapa global acumulado (M_g) en un corredor local delante del dron (ver diseño detallado en §2). Se corrigió un deadlock estructural real durante el camino ("Bug A"), se validó con éxito una versión completa de la capa el 13/07 tras varias rondas de arreglos — pero **nunca se commiteó**, y Irene decidió ese mismo día abandonar el enfoque incremental por considerar que se habían acumulado demasiados parches sobre parches sin resolver el problema de fondo.

2. **Reunión con los tutores del lunes 13/07 — replanteamiento del diagnóstico.** Se determinó que el problema no era (solo, ni principalmente) de **latencia** de actualización del mapa como se había asumido hasta entonces, sino de **inconsistencia de activación**: la capa LiDAR a veces detectaba el obstáculo a tiempo y a veces no, sin que se supiera todavía por qué. El tutor pidió un estudio sistemático de más situaciones para entender el patrón antes de seguir iterando sobre el mecanismo de detección. Como consecuencia, **se dejó a un lado toda la capa reactiva LiDAR** (se pasó el diseño y los hallazgos al tutor) y se empezaron los estudios cuantitativos de latencia M_t/M_g, todavía en curso a fecha de este informe.

3. **Pivote paralelo hacia `FollowPath::modify()` (22/07).** De forma independiente al abandono de la capa LiDAR, se identificó y corrigió la causa raíz real del síntoma de "giro brusco" que había motivado buena parte de la capa reactiva como mitigación: el mecanismo de replan cancelaba y reenviaba el `FollowPath` completo en vez de modificarlo, parando el dron en seco en cada replan. Se sustituyó por el servicio `modify()` de Aerostack2 (apoyándose en un fix upstream de un colaborador del proyecto, no de Irene), y en el mismo día se eliminó por completo el código de la capa reactiva LiDAR (~337 líneas).

**Estado a 24/07/2026:**
- Rama activa: `feat/replan-modify-waypoints`, sin capa reactiva LiDAR, con el replan vía `modify()`.
- El diseño M_t/M_g (capa reactiva basada en LiDAR) se considera **aparcado, no descartado**: se pasó al tutor, y su validez depende del resultado del estudio de latencia en curso.
- Bug B (crash por giro brusco original, distinto del bug de `modify()` ya arreglado) sigue sin causa raíz confirmada a nivel de controlador.
- Estudio cuantitativo de latencia M_t/M_g: en curso esta semana, se redactará en un documento aparte cuando concluya.

---

## 2. El diseño técnico de la capa reactiva LiDAR (M_t vs M_g)

Esta sección documenta el diseño tal como se concibió y se implementó, para que quede disponible al redactar la memoria del TFM aunque el código ya no esté en la rama activa.

### 2.1 Motivación

El planificador A* trabaja sobre M_g, el mapa de ocupación global/acumulado, que es estable y filtrado (requiere varias detecciones consistentes para marcar una celda como ocupada — `hit_confidence`/`miss_confidence`/`occ_threshold_`, ver `scan2occ_grid`). Esa estabilidad es necesaria para planificar rutas coherentes, pero tiene coste: M_g puede tardar de forma variable (ver §2.4) en confirmar un obstáculo dinámico recién aparecido. Mientras tanto, el dron puede seguir volando hacia él porque, desde el punto de vista del planificador, el camino sigue libre.

La idea de partida (previa al 07/07) fue trabajar directamente con los rayos crudos del LiDAR (`LaserScan`) para detectar obstáculos antes de que llegaran a consolidarse en el mapa. Esto se implementó en una rama anterior (aproximadamente `reactive-lidar` o similar, previa a este periodo).

### 2.2 Rediseño: desacoplar la capa de seguridad del sensor y trabajar sobre mapas de ocupación

A partir del 30/06 se rediseñó el enfoque para no depender directamente del `LaserScan`, sino de dos mapas de ocupación derivados de él:

- **M_t (mapa instantáneo):** se genera por cada medida/escaneo del LiDAR, sin acumulación temporal — refleja lo que el sensor ve *ahora mismo*, con todo su ruido.
- **M_g (mapa global/acumulado):** el mismo mapa filtrado y estable sobre el que trabaja el planificador, que ya genera A* y el path P.

El planificador sigue trabajando exclusivamente sobre M_g para generar el path P, sin cambios en su lógica. La capa de seguridad, en paralelo, revisa cada nuevo M_t y lo compara contra M_g, pero **solo dentro de una zona local delante del dron**: el tramo del path que tiene por delante más un margen lateral (el "corredor").

Dentro de ese corredor, se buscan celdas que aparecen ocupadas en M_t pero que todavía no están ocupadas en M_g — esas celdas representan posibles obstáculos nuevos que el planificador aún no ha incorporado a su modelo del mundo. Sobre ellas se aplica la misma lógica de robustez que ya existía en otras partes del sistema: exigir varias detecciones locales que se repitan en ciclos consecutivos antes de actuar, y frenar o parar según la distancia al posible obstáculo.

Esto mantiene reacción rápida (no se espera a que M_g confirme el obstáculo) sin que el algoritmo dependa directamente del `LaserScan` crudo. La entrada de la capa de seguridad pasa a ser la **diferencia local entre M_t y M_g dentro del corredor de vuelo**. Cuando el obstáculo se consolida en M_g, el planificador normal ya puede replanificar la ruta de forma ordinaria.

### 2.3 Refinamiento: M_g retrasado en vez de M_g en vivo

Un afinamiento posterior (ver §2.4) sobre esta idea: como M_t y M_g derivan del mismo escaneo de LiDAR en el mismo instante, comparar M_t contra M_g *en vivo* no da ninguna ventaja de velocidad — ambos reflejan la misma información en el mismo momento, solo que M_g la filtra. La comparación útil es M_t (en vivo) contra una copia de M_g **deliberadamente retrasada** en el tiempo (`reactive_map_delay_`, 0.3s por defecto), que representa "lo que el planificador todavía cree que es cierto". El retraso no añade latencia a la reacción (se sigue evaluando cada tick, cada 0.05s); solo cambia la referencia usada para decidir "¿esto ya estaba aquí o es nuevo?".

Esto llevó, en una iteración posterior del diseño (12/07), a mantener **dos snapshots de M_g** (uno "cercano" y otro "establecido") para filtrar mejor las celdas de frontera/desconocidas de las que son obstáculos dinámicos genuinos.

### 2.4 Por qué se pensaba que el problema era de latencia (y qué se midió)

La hipótesis de trabajo durante buena parte de julio fue que el problema principal era el **retraso de consolidación de M_g**. Se instrumentó el código temporalmente para medir, celda a celda de la huella de un obstáculo dinámico de prueba (caja 1×1m), el tiempo entre que M_t la marca ocupada y que M_g cruza el umbral de ocupación en vivo:

| Métrica | Valor |
|---|---|
| Mínimo | 0.149 s |
| Mediana (celdas de impacto directo, mirando al dron) | ~0.19–0.29 s |
| Media (con cola larga incluida) | 1.72 s |
| Máximo | 8.09 s |

Las celdas de impacto directo consolidan rápido (~0.2–0.3s); la cola larga (hasta 8s) corresponde a celdas de borde/esquina vistas en ángulos rasantes. Esto confirmaba que M_g por sí solo, sin capa reactiva, puede tardar varios segundos en confirmar un obstáculo según el ángulo de incidencia — y motivó tanto el diseño M_t-vs-M_g-retrasado como la creencia de que resolver la latencia bastaría para resolver el problema de seguridad.

**Esta hipótesis resultó incompleta** (ver §4) — el hallazgo del 12/07 mostró que, en la práctica, la capa reactiva casi nunca llegaba siquiera a evaluar la comparación M_t/M_g, por un problema de campo de visión anterior a cualquier cuestión de latencia.

---

## 3. Línea de tiempo de hitos y commits (29/06 – 24/07)

Fechas y hashes literales de `git log`. Horas estimadas por complejidad del diff y separación temporal entre commits.

| Fecha | Repo | Hash | Autor | Descripción | Origen |
|---|---|---|---|---|---|
| 30/06/26 | aerostack2 | `63196704` | Irenevm | Primera implementación: capa reactiva LiDAR (M_t vs M_g en corredor) + comprobación de segmento MAP_CHECK | Autónomo, diseño acordado el 28/06 |
| 06/07/26 | sherec | `cf6b68d8` | Irenevm | Casos de estrés 8-12 rediseñados con obstáculos a distancia realista + `test_sharp_turn.py` (diagnóstico aislado del giro brusco, sin A*/LiDAR) | Autónomo — necesario para exponer los bugs A y B |
| 06/07/26 | aerostack2 | `aea2f57d` | Irenevm | **Fix "Bug A"**: reordena `update_grid()` (erosión antes de la máscara del dron), nuevo parámetro `drone_self_mask_radius` | Autónomo — causa raíz hallada tras los casos 8-12 |
| 09/07/26 | sherec | `a63c44db` | Irenevm | Estudio inicial de latencia M_t/M_g (`measure_map_delay.py`), campos LiDAR en telemetría, suite `mission_nav_test_v2.py` | Autónomo |
| 09/07/26 | sherec | `1cf5ebe8` | Irenevm | Instalación de PlotJuggler vía apt en el Dockerfile | Petición de los tutores |
| 12/07/26 | aerostack2 | `4251edc6` | Irenevm | Doble snapshot de M_g (near+established) para filtrar celdas frontera de obstáculos dinámicos | Autónomo |
| 12/07/26 | aerostack2 | `a66a84bd` | Irenevm | Refactor menor: recorte de comentarios, restaura logging de coordenadas de debug | Autónomo |
| 12/07/26 | aerostack2 | (sesión larga) | Irenevm | **Hallazgo clave**: el cono frontal de detección nunca se activaba en ningún test. Causa identificada ese mismo día: una carrera de escritura en `scan2occ_grid.cpp` podía borrar el impacto de un rayo con otro rayo del mismo escaneo sobre la misma celda. Se corrige (dos pasadas: libres primero, impactos después). Se añade un guardia de proximidad omnidireccional en 360° sobre M_g como red de seguridad adicional (el cono frontal, por diseño, es ciego a obstáculos al lado o detrás durante un rodeo) | Autónomo |
| 13/07/26 | aerostack2 | `7e0547fc` (WIP, no commiteado a la rama final) | Irenevm | 5 fixes de robustez: freeze "rodeado de zona gris" (ventana de gracia + red de seguridad "unstick"), SUCCESS falso tras un retreat, colapso de altitud, giro brusco (amortiguación >60°), detección de choque. **Se valida con éxito una misión completa con capa reactiva activada + sin atascos + llega al goal** (caso 6 v3: SUCCESS en 41.5s) | Autónomo, sesión intensiva de validación |
| **13/07/26** | — | — | — | **REUNIÓN con los tutores.** Ver §4 — se determina que el problema no es de latencia sino de activación inconsistente; se decide dejar a un lado la capa LiDAR y hacer un estudio sistemático | Reunión |
| **13/07/26** | — | — | — | Decisión de Irene: abandonar el enfoque incremental sobre `feat/lidar-reactive-safety` (demasiados parches encadenados) | Decisión propia, mismo día que la reunión |
| 14/07/26 | aerostack2 | `0395abb6` | Davide Torielli (upstream, no Irene) | `[fix] Follow path behavior on point modify` — fix que habilita `modify()` de forma fiable en Aerostack2 | Upstream, integrado por merge/rebase |
| 20/07/26 | aerostack2 | `c3c3ab32` | Guillermo García Patiño (upstream) | Fix del planner: escaneo lineal → priority queue + coste g (#968) | Upstream, sin relación directa con el TFM |
| 22/07/26 | aerostack2 | `f5cde961` | Irenevm | **Pivote**: reemplaza cancel+resend por `FollowPath::modify()` al replanificar; corrige el bug real de raíz del giro brusco (`own_modify()` fusionaba IDs de waypoints en vez de reemplazar la cola entera) | Autónomo, apoyado en el fix upstream del 14/07 |
| 22/07/26 | aerostack2 | `eab9f3de` | Irenevm | **Elimina por completo la capa reactiva LiDAR** (-337 líneas, 3 ficheros) | Decisión propia — el diseño se pasó al tutor, y `MAP_CHECK` + `modify()` ya cubren el caso de uso principal sin depender del LiDAR crudo |
| 23-24/07/26 | sherec | *(sin commit)* | Irenevm | ~150 ejecuciones del estudio cuantitativo de latencia M_t/M_g (nuevos scripts: `latency_study_mt_mg.py`, `analyze_latency_study.py`, `mission_detection_probe.py`, `latency_cell_monitor.py`) + 3 bugs de arranque/parada del stack corregidos | Autónomo — estudio pedido tras la reunión del 13/07, en curso |

**Nota sobre reuniones:** a diferencia del periodo anterior (dos reuniones con peticiones técnicas explícitas registradas en commits), en este periodo la única reunión documentada con impacto directo en el rumbo del trabajo es la del **13/07**. El resto del trabajo fue autodirigido por los hallazgos de las propias pruebas de estrés.

---

## 4. La reunión del 13/07/2026 — de "problema de latencia" a "problema de activación inconsistente"

Este es el punto de inflexión conceptual del periodo y debe quedar claro para la memoria del TFM, porque cambia la narrativa de "por qué se abandonó la capa LiDAR":

**Lo que se creía antes de la reunión:** el obstáculo dinámico a veces provocaba comportamientos peligrosos porque el mapa de ocupación tardaba en actualizarse (ver estudio de latencia, §2.4) — es decir, un problema de **tiempo**: el mapa consolidaba el obstáculo demasiado tarde respecto a la velocidad del dron.

**Lo que se descubrió (sesión del 12/07, confirmado en la reunión del 13/07):** al analizar en detalle por qué la capa reactiva no actuaba a tiempo, se encontró que, en la mayoría de los casos, **la capa reactiva ni siquiera llegaba a activarse** — no por lentitud del mapa, sino porque el mecanismo de detección tenía un defecto estructural de campo de visión (el cono de detección de ±15° atado al yaw instantáneo del dron perdía al obstáculo en cuanto había cualquier curva). Esto se probó con logs: `grep -c REACTIVE_MODE` daba 0 activaciones en 120 segundos de misión completa, incluso en el log más crudo posible (antes de aplicar ningún filtro M_t/M_g).

En la reunión del lunes 13/07, se trasladó este hallazgo a los tutores junto con el resto del diseño M_t/M_g. La conclusión conjunta fue que el problema real no era "el mapa tarda" sino "**a veces el sistema detecta el obstáculo a tiempo y a veces no, y todavía no sabemos por qué**" — es decir, un problema de **consistencia de activación**, no de latencia pura. El tutor pidió un estudio sistemático de más situaciones (distintas velocidades, distintos ángulos de aproximación, distintas distancias de disparo) para entender el patrón antes de seguir parcheando el mecanismo de detección.

**Consecuencia directa:** se decidió dejar a un lado toda la capa reactiva LiDAR (su diseño e implementación se entregaron al tutor tal cual estaban) y dedicar la semana del 13 al 24/07 al estudio cuantitativo de latencia y activación (`latency_study_mt_mg.py` y scripts asociados), que sigue en curso a fecha de este informe y se redactará por separado cuando concluya. Esto no invalida el diseño M_t/M_g de §2 — queda pendiente de que el estudio determine si el problema de fondo es solucionable ajustando ese diseño (p.ej. ensanchando el cono, añadiendo el guardia omnidireccional de forma permanente) o si requiere un enfoque distinto.

---

## 5. Bugs encontrados y su estado real

| Bug | Reproducido | Causa raíz | Arreglado | Validado | Estado a 24/07 |
|---|---|---|---|---|---|
| **Bug A** — deadlock por punto ciego en A*/segment-check | Sí (casos 8, 9, TIMEOUT 120s) | Sí — máscara del dron aplicada antes de la erosión por `safety_distance`, atando el tamaño del dron al margen de inflado | Sí, `aea2f57d` (06/07) | Sí — caso 5 sin regresión, caso 8 pasa de TIMEOUT a abort limpio en 33.4s | **Cerrado**, sigue en pie en la rama activa |
| **Cono frontal M_t "nunca se activa"** (defecto estructural) | Sí, 0 activaciones en 120s en todos los tests del 12/07 | Sí, dos causas superpuestas: (a) carrera de escritura en `scan2occ_grid.cpp` que podía borrar un impacto real; (b) tras arreglar (a), persistía porque el cono de ±15° atado al yaw instantáneo pierde el obstáculo en cuanto hay cualquier curva | Parcialmente — (a) se corrigió; (b) nunca se arregló, se decidió eliminar toda la capa en su lugar | Confirmado con logs (`grep -c REACTIVE_MODE` = 0) | **Motivó la reunión del 13/07 y el abandono de la capa** (§4) |
| **"Reject storm"** — ráfaga de replans rechazados sin control | Sí, mismo log del caso 6 (9 llamadas a `trigger_replan()` en 2.4s) | Sí — mensaje de log engañoso ("Aborting navigation") que en realidad solo activa un flag interno, sin abortar nada | No — documentado, sin fix propio; obsoleto tras eliminar la capa reactiva | No aplica | Obsoleto |
| **Freeze "rodeado de zona gris" (bucle replan→freeze→replan)** | Sí (sesión 13/07) | Sí — desajuste de parámetros: A* solo garantiza 0.8m de despeje al rodear un obstáculo, pero la capa LiDAR pausaba el vuelo a <1.5m de cualquier obstáculo conocido, incluido el que A* acababa de rodear a propósito | Sí (ventana de gracia dinámica + red de seguridad "unstick" tras 6s de freno sostenido) | Sí, validado end-to-end (caso 6 v3, SUCCESS 41.5s) | Arreglado en la rama luego descartada junto con el resto de la capa |
| **SUCCESS falso tras completar un retreat** | Sí (sesión 13/07) | Sí — el movimiento de retirada no pasaba por `is_intermediate_goal_`, así que no se distinguía "retreat completado" de "goal alcanzado" | Sí (`retreat_move_active_`) | Sí, mismo test que el anterior | Descartado con el resto de la capa |
| **Falsos positivos por ruido LiDAR** (caso 2, DECEL activo sin obstáculo real) | Sí | Sí — celda de ruido aislada cerca del borde de `lidar_detection_range_`, sin filtro de clúster mínimo | Sí (`lidar_min_cluster_size` 1→3) | Sí, confirmado sin falsos positivos tras el fix | Descartado con el resto de la capa |
| **"Bug B" — crash por giro brusco (colapso de altitud), versión original** | Sí, reproducido con geometría pura (`test_sharp_turn.py`) | Parcial — descartada saturación proporcional pura; hipótesis de windup en el controlador de posición/altitud sin confirmar | No — solo mitigado (amortiguación de velocidad si el giro >60°) y detectado (`[CRASH_DETECTED]`, aborta en vez de colgarse) durante la vida de la capa reactiva | Mitigación validada indirectamente, causa de fondo sin tocar | **Sigue sin causa raíz confirmada en el controlador** — bloqueado porque el test se suscribió al topic equivocado (`motion_reference/twist` en vez de `motion_reference/pose`/`actuator_command/thrust`) |
| **Giro brusco por `own_modify()` (bug distinto, hallado el 22/07)** | Sí | Sí — `own_modify()` fusionaba IDs de waypoints en vez de reemplazar la cola entera, dejando un waypoint "fantasma" detrás del dron | Sí (`f5cde961`, 22/07) | Parcial — resuelto por revisión de código y lógica; sin batería de regresión documentada tras el cambio | **Resuelto**, pero por una vía distinta a la que se venía persiguiendo (no era el controlador de vuelo ni el LiDAR) |
| **Crash en vuelo recto sin obstáculo** (caso v2-1) | Sí (altitud 1.14→0.08m en ~4s, sin obstáculo presente) | No | No, aplazado explícitamente | No | Abierto |
| **Bug de replan agotado sin capa reactiva** (caso 12, ~44s de bloqueo) | Sí | Sí — `need_replan_` permanece `true` casi siempre | No, aplazado | No | Abierto |
| **Monitor Python no detecta FAILURE ni respeta su propio timeout** | Sí | Sí — bug del script de misión, no del planner C++ | No | No | Abierto |
| **Telemetría Python desincronizada tras colapso de altitud** | Sí — posición impresa se congela mientras la pose real en C++ sigue cambiando | No investigado | No | No | Abierto |
| **A\* falla justo tras el despegue** (nuevo, 23/07) | Sí | Parcial — se sospecha de celdas sin explorar (-1) cerca del dron nada más despegar | El intento de arreglo (`unknown_as_free=true`) se probó y se **revirtió** porque introducía un problema peor: A* oscilando sin converger cerca de obstáculos reales, con una casi-colisión observada | No | Bloqueado, sin solución a 24/07 |
| **3 bugs de arranque/parada del stack** (`stop.bash`, `send-keys` sin Ctrl+C previo, auto-lanzamiento duplicado del estudio de latencia) | Sí (sesión 23/07) | Sí, los tres | Sí, los tres | Sí | Cerrado |

---

## 6. Decisiones de diseño y sus motivos

**6.1 Trabajar sobre mapas de ocupación (M_t/M_g) en vez de sobre el `LaserScan` crudo (previo/30-06).**
La primera versión (previa a este periodo) usaba los rayos LiDAR directamente. Se rediseñó para desacoplar la capa de seguridad del sensor y reutilizar el pipeline de ocupación ya existente (`scan2occ_grid`), manteniendo dos snapshots temporales del mismo mapa en vez de una fuente de datos completamente distinta. Ver diseño completo en §2.

**6.2 M_g retrasado en vez de M_g en vivo (refinamiento, ver §2.3).**
Motivado por que M_t y M_g derivan del mismo escaneo — comparar contra M_g en vivo no da ninguna ventaja real de velocidad. Comparar contra una copia deliberadamente retrasada sí distingue "esto ya lo sabía el planificador" de "esto es nuevo".

**6.3 Reordenar `update_grid()` en vez de agrandar la máscara del dron (06/07).**
Para el deadlock del Bug A, se separó conceptualmente "tamaño físico del dron" de "margen de inflado de obstáculos" con un parámetro independiente, porque el acoplamiento entre ambos era la causa raíz.

**6.4 Añadir un guardia de proximidad omnidireccional, separado del cono frontal (12/07).**
El cono de detección original solo mira en la dirección de vuelo — durante un rodeo, el obstáculo queda al lado o detrás, estructuralmente invisible para cualquier cono frontal. Se añadió una comprobación en 360° sobre M_g como red de seguridad adicional, sin exigir que el obstáculo sea "nuevo": si ya está mapeado, no hace falta confirmarlo dos veces.

**6.5 Abandonar el enfoque incremental sobre `feat/lidar-reactive-safety` (13/07).**
Motivo explícito de Irene: varias sesiones acumulando parches puntuales (freezes, reject-storms, colapsos de altitud) sin resolver el problema de fondo. Coincide con la reunión del mismo día (§4) que reencuadró el problema como de activación inconsistente, no de latencia — reforzando la decisión de no seguir parcheando el mecanismo de detección sin antes entender el patrón.

**6.6 Dejar a un lado toda la capa reactiva LiDAR y hacer estudios en vez de seguir iterando (13/07, por indicación del tutor).**
A diferencia de las demás decisiones de este periodo, esta no fue una decisión técnica unilateral de Irene sino resultado directo de la reunión: antes de seguir modificando el mecanismo de detección, había que entender **por qué a veces funcionaba y a veces no**. El diseño M_t/M_g no se descarta como inválido — queda pendiente de lo que revele el estudio.

**6.7 Pivotar hacia `FollowPath::modify()` en vez de seguir mitigando el giro brusco con la capa reactiva (22/07).**
El pivote más importante del periodo, y paralelo (no directamente derivado) del abandono de la capa LiDAR. Se identificó que el patrón cancelar+reenviar el `FollowPath` en cada replan paraba el dron en seco, contribuyendo al síntoma de "giro brusco" que la capa reactiva llevaba semanas intentando mitigar sin llegar a la causa. Usar `modify()` (viable gracias a un fix upstream del 14/07) ataca el síntoma en su origen real.

**6.8 Eliminar por completo el código de la capa reactiva LiDAR, no solo dejarla desactivada (22/07).**
Una vez que `MAP_CHECK` + `modify()` cubren el caso de uso principal sin depender de detección LiDAR independiente, y con el diseño ya entregado al tutor para el estudio paralelo, se consideró que mantener ~800 líneas de C++ sin usar en la rama activa (con su propio riesgo de mantenimiento y comportamiento frágil bajo parámetros ajustados) no aportaba valor. El código sigue disponible en las ramas donde se desarrolló (`feat/lidar-reactive-safety`) por si el estudio de activación concluye que merece la pena retomarlo.

**6.9 Revertir el experimento `unknown_as_free=true` pese a que arreglaba el síntoma que motivó probarlo (23/07).**
Tratar las celdas sin explorar como libres evitaba el fallo espurio de A* tras el despegue, pero introducía oscilación de replanificación cerca de obstáculos reales (con una casi-colisión observada) — un fallo considerado potencialmente peor que el original. Revertido explícitamente, dejando el problema original sin resolver pero sin introducir uno nuevo.

---

## 7. Qué queda pendiente o bloqueado a 24/07/2026

- **Estudio de latencia/activación M_t-vs-M_g (pedido en la reunión del 13/07):** en curso, ~150 ejecuciones registradas los días 23-24/07, sin analizar todavía. Se redactará en un documento aparte cuando concluya — este informe no debe usarse como fuente de sus resultados finales.
- **Decisión pendiente sobre el destino del diseño M_t/M_g:** depende del resultado del estudio anterior — si se retoma la capa reactiva (posiblemente con el guardia omnidireccional como mecanismo principal en vez del cono frontal) o si se concluye que `MAP_CHECK` + `modify()` son suficientes.
- **Bug B histórico (causa raíz del colapso de altitud) sin confirmar** — pendiente de repetir el test de diagnóstico suscrito a los topics correctos.
- **Bugs aplazados** (crash en vuelo recto sin obstáculo, replan agotado, monitor Python, telemetría desincronizada): ninguno se ha vuelto a tocar desde que se documentaron.
- **A\* falla justo tras el despegue:** sin solución, el único intento de arreglo introdujo un riesgo mayor.
- **Sin validación de no-regresión tras eliminar la capa reactiva** (`eab9f3de`): no hay evidencia de que se hayan vuelto a correr los casos 1-5 tras la eliminación del 22/07.
- **Working tree sin commitear** en ambos repos a fecha del informe (instrumentación de diagnóstico `[LAT]` en `aerostack2`; scripts del estudio de latencia y ~150 JSON de resultados en `sherec_nav`).

---

## 8. Archivos modificados relevantes en el periodo

### `aerostack2`

| Archivo | Cambio a grandes rasgos |
|---|---|
| `as2_behaviors_path_planning/src/path_planner_behavior.cpp` | Capa reactiva LiDAR completa: añadida (30/06–13/07), luego eliminada (22/07, -290 líneas); mecanismo de replan reescrito para usar `modify()`; instrumentación `[LAT]` sin commitear |
| `as2_behaviors_path_planning/include/.../path_planner_behavior.hpp` | Miembros de la capa reactiva añadidos y luego retirados |
| `as2_behaviors_path_planning/plugins/a_star/src/a_star_searcher.cpp` | Reordenación erosión/máscara (fix Bug A); experimento `unknown_as_free` documentado y revertido |
| `as2_behaviors_path_planning/plugins/a_star/src/a_star.cpp` | Instrumentación de diagnóstico `[LAT] ASTAR_FAIL`, sin commitear |
| `as2_behaviors_path_planning/plugins/follow_path_plugin_position.cpp` | Cambios para soportar `modify()` |
| `as2_behaviors_path_planning/config/behavior_default.yaml` | Parámetros de la capa reactiva añadidos, ajustados varias veces y finalmente eliminados |
| `as2_map_server/plugins/scan2occ_grid/src/scan2occ_grid.cpp` | Fix de la carrera de escritura en M_t (dos pasadas: libres primero, impactos después) |

### `project_sherec_navigation`

| Archivo | Cambio a grandes rasgos |
|---|---|
| `mission/mission_nav_test.py` | Casos de estrés 8-12 rediseñados con obstáculos a distancia realista |
| `mission/test_sharp_turn.py` | Nuevo — diagnóstico aislado del giro brusco, sin A*/LiDAR |
| `mission/mission_nav_test_v2.py`, `mission_nav_test_v3.py` | Nuevas suites, detección de STUCK/CRASH añadida |
| `measure_map_delay.py`, `latency_study_mt_mg.py`, `analyze_latency_study.py`, `mission_detection_probe.py`, `latency_cell_monitor.py` | Herramientas del estudio de latencia/activación M_t/M_g, en curso |
| `docker/Dockerfile` | Instalación de PlotJuggler (petición de los tutores) |
| `stop.bash`, `tmuxinator/aerostack2.yml`, skill `run-mission` | 3 fixes de arranque/parada del stack de simulación |

---

*Informe generado el 24/07/2026, fusionando y reconciliando los cuatro informes intermedios de `doc/Julio/` con las aclaraciones directas de Irene sobre el diseño M_t/M_g y la reunión del 13/07. Continúa a `doc/retrospectiva_tfm_navegacion_reactiva.md` (hasta 28/06/2026). El estudio de latencia/activación en curso se documentará por separado.*
