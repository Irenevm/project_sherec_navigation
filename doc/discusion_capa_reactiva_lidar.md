# Discusión técnica: capa de navegación reactiva basada en LiDAR

*Notas para la redacción del TFM — sección de decisiones de diseño*

---

## Origen del problema

El sistema de navegación usa un mapa de ocupación (`OccupancyGrid`) **probabilístico**: una celda
solo se marca como ocupada cuando la probabilidad acumulada de scans LiDAR supera el **30%**.
Esto introduce una latencia estructural de **~1.5 s (≈3 scans a 10 Hz)** entre el momento en que
el LiDAR ve un obstáculo y el momento en que el planificador puede reaccionar.

A una velocidad de crucero de 1.0–1.5 m/s, el dron recorre **1.5–2.25 m** durante ese retraso.
En el experimento de validación, cuando el mapa finalmente marcó el obstáculo, el dron ya estaba
a **~0.9 m** de él, dentro de su zona de seguridad (`safety_distance = 0.5 m`).

El LiDAR crudo ve el obstáculo en el primer scan, sin latencia probabilística. La solución es una
capa reactiva que actúe sobre los datos del sensor directamente, sin esperar al mapa.

---

## El flujo de replanificación ya existente (no hay que reimplementarlo)

Es importante aclarar que la **evasión del obstáculo** (ir alrededor) ya está implementada en
el planner. El flujo completo es:

```
1. Obstáculo aparece
2. LiDAR lo ve inmediatamente (scan 1)
3. [CAPA REACTIVA]: frena/para el dron ← esto es lo que hay que añadir
4. Mapa acumula scans → ~1.5 s → probabilidad ≥ 30% → celda marcada como OCUPADA
5. map_check_timer_ (cada 0.5 s) detecta waypoints bloqueados → cancela FollowPath
6. trigger_replan() → A* recalcula ruta rodeando el obstáculo
7. Dron reanuda navegación por la ruta nueva
```

La capa reactiva **solo debe garantizar que el dron esté parado y a salvo** durante los pasos
3–5. El esquive lo hace A* en el paso 6, que ya funciona. No hace falta implementar evasión
local propia (lo que en el plan se llamaba "Fase 3 — Esquive local").

---

## Dos opciones para implementar la capa reactiva

### Opción A — nav2_collision_monitor (paquete estándar de ROS 2 / Nav2)

`nav2_collision_monitor` es un nodo de producción que actúa como **filtro de seguridad** en el
canal de velocidad. Se interpone entre quien genera las órdenes de velocidad y el controlador,
y las filtra o anula si el LiDAR detecta un obstáculo en las zonas configuradas.

**Cómo funciona:**
```
[FollowPath / motion_reference_handlers]
          ↓ publica en
  motion_reference/twist_raw          ← CM lee aquí (cmd_vel_in_topic)
          ↓
  [nav2_collision_monitor]            ← evalúa LaserScan crudo (~4-5 ms latencia)
          ↓ publica en
  motion_reference/twist              ← CM publica aquí (cmd_vel_out_topic)
          ↓
  [as2_motion_controller]             ← sin cambios, consume motion_reference/twist
```

Tiene tres modelos de comportamiento, todos configurables sin código:

| Modelo | Qué hace | Parámetro clave |
|--------|----------|-----------------|
| **Stop** | Si hay N puntos en la zona de parada → velocidad = 0 | polígono `stop_zone` |
| **Slowdown** | Si hay puntos en la zona de aviso → multiplica velocidad × ratio | `slowdown_ratio` (ej. 0.3) |
| **Approach (TTC)** | Proyecta la trayectoria cinemática → si Tiempo a Colisión < umbral → reduce velocidad | `time_before_collision` |

Las zonas se definen como **polígonos en el frame del robot (`base_link`)** que rotan con él
→ **cobertura omnidireccional**: detecta obstáculos adelante, a los lados y detrás.

**Ventajas:**
- Código propio: casi ninguno (configuración YAML + remapping de topics)
- Modelo Approach/TTC: más riguroso que un controlador proporcional simple
- Battle-tested: usado en producción en robots reales y simulaciones nav2
- Latencia garantizada: ~4-5 ms por scan
- Omnidireccional sin esfuerzo adicional

**Desventajas / coste de integración:**
- Requiere **remapping de topics** en el launch: insertar el CM entre
  `motion_reference_handlers` y `as2_motion_controller`. Esto es configuración de launch,
  no código C++, pero hay que entender bien la arquitectura.
- Es un **lifecycle node**: hay que lanzarlo con un lifecycle manager (puede ser el nodo
  `nav2_lifecycle_manager` standalone, o transición manual) y gestionar su ciclo de vida.
- Dependencia `nav2_collision_monitor`: no arrastra el stack nav2 completo (no necesita
  costmap, BT navigator, etc.), pero sí `nav2_core`. Instalable como paquete ROS 2 independiente.
- El paquete es fundamentalmente 2D (LaserScan en plano XY), lo que encaja perfectamente con
  el drone volando a altura fija y el LiDAR filtrado a 0-1 m, pero no cubriría amenazas en 3D.

---

### Opción B — Implementación propia dentro de PathPlannerBehavior (C++)

Añadir la lógica reactiva directamente en el nodo `PathPlannerBehavior` existente:
- Suscripción a `sensor_measurements/lidar/scan`
- Timer a 20 Hz que evalúa un corredor frontal hacia el próximo waypoint
- Filtro de ruido: clúster de rayos + persistencia de N scans consecutivos
- Ley de frenado proporcional: `v_safe = clamp(Kp·(d − stop_distance), 0, v_nav)`
- Frenado vía `follow_path_pause_client_` (pause/hover) o reenviando goal con menor `max_speed`

**Ventajas:**
- Sin dependencias externas, sin lifecycle, sin remapping de topics
- Integración limpia dentro del nodo existente (mismo hilo, sin mutex)
- Contribución técnica propia para la tesis (diseño e implementación)
- El corredor frontal orientado a la dirección de marcha evita falsos positivos
  de paredes laterales del corredor por donde vuela el dron

**Desventajas:**
- ~300-400 líneas de código C++ nuevo a escribir, probar y depurar
- Cobertura solo frontal (en la dirección de marcha); lateral y trasera requieren
  ampliar el diseño (o usar `d_min` de todo el scan para cobertura omnidireccional)
- Modelo de frenado más simple (proporcional vs TTC cinemático)
- Mayor riesgo de bugs que una solución de producción auditada

---

## Comparativa directa

| Criterio | nav2_collision_monitor | Implementación propia |
|---|---|---|
| Latencia de reacción | ~4-5 ms | ~50 ms (timer 20 Hz) |
| Modelo de frenado | TTC cinemático (riguroso) | Kp proporcional (más simple) |
| Cobertura direccional | Omnidireccional (polígono en base_link) | Frontal (o d_min omnidireccional) |
| Código C++ nuevo | Mínimo (solo configuración) | ~300-400 líneas |
| Dependencias nuevas | nav2_collision_monitor + lifecycle | Ninguna |
| Integración en launch | Remapping + lifecycle manager | Sin cambios en launch |
| Riesgo de bugs | Muy bajo (producción) | Moderado (código nuevo) |
| Contribución tesis | Integración/adaptación | Diseño e implementación |
| Tiempo estimado | ~1-2 días (config + validación) | ~1 semana (impl + pruebas) |

---

## Diferencia entre "usar CM directamente" y "opción intermedia"

Ambas usan el **mismo paquete** (`nav2_collision_monitor`). La diferencia es de configuración
inicial y de framing en la tesis, no de carga de trabajo:

**Usar CM directamente (todos los modelos desde el principio):**
- Configurar Stop + Slowdown + Approach/TTC desde el primer día
- Presentación en la tesis: "se integró nav2_collision_monitor en aerostack2, una arquitectura
  no estándar de Nav2, resolviendo los siguientes retos de adaptación..."
- La contribución es la **integración arquitectónica** (remapping, lifecycle, parámetros)

**Opción intermedia (empezar solo con Stop, añadir más después):**
- Primero validar solo el modelo Stop (parada garantizada) → garantía de no colisión básica
- Luego añadir Slowdown y/o Approach como mejoras, con datos comparativos para la tesis
- Útil si el tiempo es justo: el Stop ya es suficiente para la seguridad, los demás son mejoras
- Misma carga de integración técnica; la diferencia es que se valida por etapas

**En la práctica:** si la integración técnica sale bien (remapping + lifecycle), pasar de Stop a
Stop+Slowdown+Approach es solo añadir secciones al YAML de configuración. No hay más código.

---

## Lo que nav2_collision_monitor NO hace (importante para la tesis)

El CM **no planifica, no esquiva, no redirige** al dron. Solo frena o para.
La **evasión del obstáculo** (ir alrededor) la hace el planificador A* existente cuando el mapa
lo confirma. El flujo completo es:

```
CM para el dron (reacción inmediata, ~ms)
    ↓
Dron quieto y seguro (~1.5 s, mientras LiDAR acumula scans en el mapa)
    ↓
map_check_timer_ detecta waypoints bloqueados (map ≥ 30%)
    ↓
trigger_replan() → A* calcula ruta de evasión → FollowPath sigue la ruta nueva
```

La Fase 3 del plan original (esquive local sin replan A*) sería útil para evitar llamar a A*,
pero **no es necesaria** si se acepta la espera de ~1.5 s para la actualización del mapa.
Queda como trabajo futuro.

---

---

## Problemas identificados en la integración con drones / Aerostack2

### Problema 1 — Desajuste dimensional 2D vs 3D

**Valoración: débil para este caso concreto.**

El CM recibiría exactamente el mismo `LaserScan` que ya usa el sistema para construir el mapa
— el que sale de `pointcloud_to_laserscan` con filtro de altura `min_height: 0.0,
max_height: 1.0`. Si ese scan es suficiente para construir el mapa y detectar obstáculos,
también lo es para el CM.

El problema de "la mesa baja / el suelo" es real en general, pero en este setup ya está resuelto
por el filtro de altura antes de que llegue al CM. El dron vuela a altura fija, y el LiDAR ya
está configurado para eso. Este problema solo aplicaría si el dron cambiase significativamente
de altitud durante la navegación, que no es el escenario actual.

---

### Problema 2 — Conflicto con la máquina de estados de Aerostack2 *(el más crítico)*

**Valoración: real, relevante y determinante para el descarte del CM.**

El CM zeriza la velocidad en `motion_reference/twist` sin notificar a `FollowPath`. El resultado:

```
CM publica twist = 0  →  dron se para físicamente (hover)
FollowPath sigue activo  →  sigue esperando llegar al waypoint
                         →  el waypoint nunca se alcanza
                         →  deadlock silencioso
```

Además, el controlador PID de posición dentro de `follow_path_plugin_position.cpp` acumula
error de posición mientras el dron está detenido pero el waypoint objetivo sigue siendo el mismo
(**integrator windup**). Cuando el stop se libera, el error acumulado puede causar una respuesta
brusca o inestabilidad.

Durante la ventana de ~1.5 s en que el mapa aún no ha confirmado el obstáculo (probabilidad <
30%), el `map_check_timer_` no detecta bloqueo alguno y no dispara replanificación. Se produce
un **deadlock lógico**: el dron está parado por el CM, FollowPath intenta avanzar sin éxito, y
el planificador global no ve ningún problema. Nadie replantea.

La implementación propia evita este problema usando `follow_path_pause_client_` — el servicio
nativo de pausa de Aerostack2 que **notifica formalmente a FollowPath** que debe entrar en
estado paused. No hay error de posición acumulado, no hay integrator windup, y el flujo
`map_check → cancel → need_replan_ → trigger_replan()` continúa determinísticamente cuando
el mapa confirma el obstáculo.

---

### Problema 3 — Dinámica aérea y parada brusca

**Valoración: parcialmente válido; mitigado en simulación, relevante en dron real.**

En un robot terrestre, una velocidad cero inmediata es trivial. En un dron, una reducción brusca
de velocidad sin rampa de desaceleración puede desestabilizar los controladores de actitud,
provocando oscilaciones en pitch/roll o pérdida de altura momentánea.

En el setup de simulación con Gazebo y el modelo `f330`, hover es un estado estable bien
soportado por el PID y este problema tiene impacto reducido (el sistema actual ya ejecuta
paradas bruscas en `map_check`). No obstante, para la transferencia a un dron real sería
necesario revisar los parámetros de la zona de Stop y la rampa de Slowdown del CM.

Con la implementación propia, el frenado se controla explícitamente mediante la ley proporcional
`v_safe = clamp(Kp·(d − stop_distance), 0, v_nav)`, que garantiza una rampa de desaceleración
antes de la parada total.

---

### Tabla resumen de problemas

| Problema | ¿Es real? | ¿Afecta a este caso? |
|---|---|---|
| 2D vs 3D | Real en general | No — LaserScan ya filtrado en altura, misma fuente que el mapa |
| Conflicto máquina de estados + integrator windup | **Sí, crítico** | Sí — FollowPath bloqueado, deadlock lógico, windup PID |
| Dinámica aérea / parada brusca | Parcialmente | Reducido en sim; relevante en dron real |

---

## Decisión final y conclusión para el TFM

Se evaluó la integración del paquete estándar de ROS 2 `nav2_collision_monitor` como mecanismo
de seguridad reactiva de última capa. Sin embargo, **se descartó en favor de un desarrollo
personalizado integrado en `PathPlannerBehavior`** por incompatibilidad arquitectónica con la
máquina de estados de las conductas de Aerostack2.

`nav2_collision_monitor` opera de manera unidireccional y *ciega* sobre el flujo de comandos
de velocidad a bajo nivel, saltándose las capas superiores de navegación. Al interceptar y
forzar la velocidad a cero para evitar una colisión, el monitor provoca que la conducta activa
de guiado (`FollowPath`) continúe intentando avanzar de forma indefinida, generando divergencia
e inestabilidad en sus controladores PID de posición (**integrator windup**). Dado que el mapa
probabilístico de ocupación requiere un tiempo de acumulación de ~1.5 s para confirmar el
obstáculo, el planificador global no detecta bloqueo alguno durante ese intervalo, resultando
en un **deadlock lógico** (bloqueo silencioso de la misión).

La implementación propia desarrollada en este trabajo utiliza el servicio nativo de pausa de
Aerostack2 (`follow_path_pause_client_`), garantizando que la detención física por proximidad
LiDAR a alta frecuencia (20 Hz) se traduzca de forma inmediata en una transición de estado
controlada de la conducta activa. Esto no solo protege la dinámica de vuelo y la estabilidad
del estimador del dron, sino que permite un traspaso de control (*handoff*) determinista hacia
el planificador A\* una vez que el mapa probabilístico confirma la presencia del obstáculo y
dispara la replanificación de la ruta.

Esta argumentación está alineada con los principios de diseño de sistemas robóticos autónomos:
en una arquitectura basada en conductas con estado (*stateful behaviors*), las capas de
seguridad reactiva deben comunicarse a través de las APIs oficiales de la arquitectura, no
interceptar canales de bajo nivel de forma externa.

**Decisión: se implementa la capa reactiva propia descrita en el plan de implementación
([tengo-que-resolver-el-async-pancake.md](../../.claude/plans/tengo-que-resolver-el-async-pancake.md)),
Fases 0–2.**
