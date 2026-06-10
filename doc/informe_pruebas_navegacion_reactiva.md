# Informe de Pruebas — Navegación Reactiva con A* y Replanificación

**Fecha:** 10 de junio de 2026  
**Rama:** `simple_simulation`  
**Entorno:** Simulación Gazebo Ignition + aerostack2 + ROS 2 Humble

---

## 1. Objetivo

Validar el comportamiento del loop de navegación reactiva implementado en `as2_behaviors_path_planning` frente a tres escenarios representativos de misiones reales:

1. Destino libre pero inicialmente desconocido (espacio sin mapear)
2. Destino inaccesible (sala completamente cerrada)
3. Destino dentro de un obstáculo sólido

Adicionalmente, identificar y corregir deficiencias detectadas durante las pruebas.

---

## 2. Entorno de Pruebas

### 2.1 Mundo simulado (`nav_test_world.sdf`)

Sala rectangular de 26 × 16 m con los siguientes elementos:

| Elemento | Posición / Descripción | Propósito |
|---|---|---|
| Pared divisoria | x = 0, con hueco en y ∈ [−1.5, 1.5] | Crea zona este inicialmente desconocida |
| Sala cerrada NE | x ∈ [7, 13], y ∈ [4, 8] — sin puerta | Objetivo inaccesible (caso 2) |
| Bloque sólido | centro (−6, −4.5), 4 × 3 × 2 m | Objetivo ocupado (caso 3) |

![Layout del mundo de pruebas](imgs/nav_test_world_layout.png)

```
y=8  +----------+===========+----------+
     |          ‖ SALA      ‖          |
     | INICIO   ‖ CERRADA   ‖          |
     | drone0   +===========+          |
y=4  |          ‖            caso 2    |
     |   HUECO → hueco (3m)            |
y=0  |   drone0 ← spawn                |
     |          ← pared div.           |
     | [BLOQUE]                        |
y=-4.5 caso 3                          |
y=-8 +-----------------------------+
    x=-13       x=0           x=13
```

### 2.2 Configuración del dron

- Modelo: `f330` con lidar 3D en z = 0.2 m sobre base
- Spawn: (−9, 0, 0.3)
- Altura de vuelo: 1.0 m
- Velocidad de navegación: 1.5 m/s
- Planificador: A* con mapa de ocupación (point_cloud2occ_grid)
- Safety distance: 1.0 m

### 2.3 Software

- `as2_behaviors_path_planning` — plugin A* con loop reactivo
- `as2_map_server` — point_cloud2occ_grid
- Script de test: `mission/mission_nav_test.py`

---

## 3. Descripción del Loop Reactivo

El mecanismo implementado opera en dos niveles:

**Nivel 1 — Planificación inicial:** Al recibir un goal, A* calcula un camino. Si el goal está en celda ocupada, `closest_free_point()` (BFS) busca la celda libre más cercana y se navega hacia ella como waypoint intermedio.

**Nivel 2 — Replanificación reactiva (`trigger_replan`):** Cuando se alcanza el waypoint intermedio (frontera), se intenta planificar directamente al goal original. Si sigue bloqueado, se busca una nueva frontera. El proceso se repite con los siguientes criterios de abort:

| Check | Condición | Mensaje |
|---|---|---|
| Límite de replanes | `replan_count > 15` | "exceeded 15 replans" |
| Dirección incorrecta | `dist(frontera→goal) > dist(drone→goal) + 0.5 m` | "frontier is farther from goal" |
| Atasco *(nuevo)* | `dist(drone→frontera) < 0.5 m` | "drone is stuck at closest reachable cell" |

---

## 4. Casos de Prueba y Resultados

### Caso 1 — Posición libre desconocida

**Goal:** (9.0, −4.0, 1.0) — zona este, sur de la pared divisoria  
**Condición inicial:** toda la zona este es desconocida (UNKNOWN en el mapa)

**Secuencia observada:**

| Evento | Tiempo (s) | Posición drone |
|---|---|---|
| START, goal inicial bloqueado | 0.0 | (−9.0, 0.0) |
| closest_free_point → (9.45, −2.55) | 0.1 | — |
| Path: 185 wp → 8 (RDP) | 0.1 | — |
| Llega al frontier (9.27, −2.62) | ~26 | — |
| Replan: path directo 15 wp → 3 | 26.1 | (9.27, −2.62) |
| **Goal reached. Navigation succeeded.** | ~30 | (~9.0, −4.0) |

**Resultado:** ✅ SUCCESS en 2 replanes, ~30 s

**Observación:** A* encontró camino inicial a través de la pared (celdas desconocidas tratadas como libres) pero rechazó el goal real. El replan al alcanzar el frontier encontró el camino directo al goal, que ya era visible en el mapa actualizado.

**Limitación identificada (mejora futura):** El sistema no detecta que el goal se ha vuelto accesible mientras vuela. Cuando el dron pasó el hueco y el lidar escaneó la zona este, el path planner no actualizó el plan a pesar de que el goal ya era visible y alcanzable directamente. Continúa hacia el frontier y solo replanifica al llegar. Ver sección 6.

---

### Caso 2 — Posición inaccesible (sala cerrada)

**Goal:** (10.0, 6.0, 1.0) — dentro de la sala cerrada NE

**Secuencia observada (ejecución con fix aplicado):**

| Replan # | Frontier | dist drone→frontier | dist frontier→goal | Decisión |
|---|---|---|---|---|
| 0 (inicial) | (10.65, 2.75) | — | 3.28 m | fly |
| 1 | (9.95, 3.25) | 0.58 m | 2.75 m | fly |
| 2 | (9.95, 3.25) | **0.17 m** | 2.75 m | **ABORT: stuck** |

**Resultado:** ✅ FAILURE (abort correcto) en 3 replanes, ~33 s

**Mensaje en log:**
```
Replan: frontier [9.95, 3.25] is only 0.170m away — drone is stuck at the closest reachable cell. Aborting navigation.
```

**Bug detectado y corregido (ver sección 5):** En la versión previa al fix, este caso agotaba los 15 replanes (47 s) porque el check de dirección no se activaba — la frontera era 0.09 m más cercana al goal que el dron, superando el filtro por muy poco. El dron oscilaba en 5 cm repetidamente hasta agotar el contador.

---

### Caso 3 — Destino dentro de obstáculo sólido

**Goal:** (−6.0, −4.5, 1.0) — dentro del bloque sólido (x ∈ [−8,−4], y ∈ [−6,−3])

**Secuencia observada:**

| Evento | Tiempo (s) | Detalle |
|---|---|---|
| START, goal ocupado desde el inicio | 0.0 | Goal en celda OCCUPIED |
| closest_free_point → (−6.05, −2.45) | 0.1 | Borde norte del bloque, ~0.55 m clearance |
| Llega al frontier (−6.05, −2.45) | ~7 | — |
| Replan: nuevo frontier (−6.25, −2.65) a 0.023 m | 7.1 | **ABORT: stuck** |

**Resultado:** ✅ FAILURE (abort correcto) en 2 replanes, ~7 s

**Comportamiento notable:** El dron navegó al punto más cercano posible al goal (borde norte del bloque, respetando safety distance), pero al intentar acercarse más detectó que estaba atascado y abortó. La posición final (−6.05, −2.45) es la celda libre más cercana al interior del obstáculo.

---

### Tabla resumen

| Caso | Goal | Resultado | Replanes | Tiempo | Mecanismo de finalización |
|---|---|---|---|---|---|
| 1 — Libre desconocido | (9, −4) | **SUCCESS** | 2 | ~30 s | Goal reached |
| 2 — Sala inaccesible | (10, 6) | **FAILURE** | 3 | ~33 s | Stuck check (0.17 m) |
| 3 — Dentro de obstáculo | (−6, −4.5) | **FAILURE** | 2 | ~7 s | Stuck check (0.023 m) |

---

## 5. Bug Detectado y Corrección Aplicada

### 5.1 Descripción del bug

**Componente:** `PathPlannerBehavior::trigger_replan()` en `path_planner_behavior.cpp`

**Síntoma:** En el caso 2, el dron oscilaba indefinidamente alrededor de un único punto durante ~47 s (15 replanes completos) cuando la solución correcta era abortar en el 4.º replan.

**Causa raíz:** El check de dirección existente (`dist_frontier_to_goal > dist_drone_to_goal + 0.5`) compara la frontera con el goal, no con la posición actual del dron. En este caso la frontera sí estaba más cerca del goal que el dron (por 0.09 m), así que el check la aprobaba aunque el dron ya estuviera esencialmente encima de ella.

Extracto del log problemático:
```
[replan #4] Waypoint[1/1] : [9.850, 3.150] | delta_drone->[-0.001, -0.002]
[replan #5] Waypoint[1/1] : [9.850, 3.150] | delta_drone->[0.040, -0.024]
...  (hasta replan #16)
Replan: exceeded 15 replans without reaching goal. Aborting navigation.
```

### 5.2 Corrección

Se añadió en `trigger_replan()`, tras el check de dirección, un **check de distancia mínima dron→frontera**:

```cpp
double dist_drone_to_frontier = std::hypot(
    drone_pose_.pose.position.x - new_frontier.point.x,
    drone_pose_.pose.position.y - new_frontier.point.y);
constexpr double MIN_FRONTIER_DIST = 0.5;  // metros
if (dist_drone_to_frontier < MIN_FRONTIER_DIST) {
    RCLCPP_ERROR(this->get_logger(),
        "Replan: frontier [%.2f, %.2f] is only %.3fm away — drone is stuck at "
        "the closest reachable cell. Aborting navigation.",
        new_frontier.point.x, new_frontier.point.y, dist_drone_to_frontier);
    navigation_aborted_ = true;
    return;
}
```

**Razonamiento:** Si la celda libre más cercana al goal está a menos de 0.5 m del dron, el dron ya se encuentra en la mejor posición alcanzable. Seguir replanificando no produce progreso.

**Impacto medido:**

| Métrica | Antes | Después |
|---|---|---|
| Replanes hasta abort (caso 2) | 16 | 3 |
| Tiempo hasta abort (caso 2) | ~47 s | ~33 s |
| Mensaje de abort | "exceeded 15 replans" | "stuck at closest reachable cell" |

---

## 6. Mejoras Propuestas

### 6.1 Replanificación continua durante el vuelo *(trabajo futuro)*

**Motivación (observada en caso 1):** Cuando el dron pasa por el hueco de la pared divisoria y el lidar escanea la zona este, el mapa se actualiza y el goal original pasa a ser accesible directamente. Sin embargo, el sistema no detecta este cambio y continúa hacia el waypoint frontera inicial, añadiendo un desvío innecesario.

**Comportamiento actual:**
```
Drone [-9,0] → frontier [9.45,-2.55] → replan → goal [9,-4]
                 (ruta subóptima)
```

**Comportamiento ideal con mejora:**
```
Drone [-9,0] → (detecta que goal ya es accesible al cruzar el hueco) → goal [9,-4]
                 (ruta directa)
```

**Solución propuesta:** En `on_run()`, mientras `is_intermediate_goal_ == true`, ejecutar periódicamente (~cada 2 s) un intento de A* desde la posición actual al `original_goal_`. Si encuentra camino, cancelar el FollowPath en curso y replanificar directamente al goal.

```cpp
// Pseudocódigo en on_run():
if (is_intermediate_goal_ && tick_count_ % MAP_CHECK_INTERVAL == 0) {
    bool direct_path_found = path_planner_plugin_->on_activate(drone_pose_, original_goal_);
    if (direct_path_found) {
        trigger_replan();  // replanifica al goal original directamente
    }
}
```

**Prioridad:** Media. El sistema funciona correctamente sin esta mejora; solo afecta a la eficiencia de la trayectoria en casos tipo 1.

---

### 6.2 Distinción entre FAILURE total y "llegué al punto más cercano"

**Motivación (observada en caso 3):** Cuando el goal está dentro de un obstáculo, el dron navega correctamente hasta el borde del obstáculo pero el resultado se reporta como FAILURE. En algunos escenarios de misión (inspección de una zona), llegar al punto más cercano posible podría considerarse un éxito parcial.

**Solución propuesta:** Añadir un nuevo estado de resultado `PARTIAL_SUCCESS` o un flag `reached_closest_free_point` en el mensaje de resultado de la acción, para que el cliente pueda distinguir entre "no pude acercarme" y "llegué al máximo posible".

**Prioridad:** Baja. Requiere modificar la interfaz de la acción `NavigateToPoint`.

---

## 7. Conclusiones

El loop de navegación reactiva implementado supera correctamente los tres escenarios de prueba:

- **Espacio libre desconocido:** el sistema navega a través de zonas no mapeadas usando fronteras BFS y replanifica al llegar, alcanzando el destino. ✅
- **Destino inaccesible:** el sistema se aproxima hasta la celda libre más cercana al obstáculo y aborta con el mecanismo correcto (nuevo check de stuck), sin malgastar replanes. ✅  
- **Destino dentro de obstáculo:** idéntico al caso anterior, con abort en 2 replanes. ✅

El bug detectado (oscilación por frontier excesivamente cercano) ha sido corregido y validado con una segunda ejecución del caso 2, reduciendo el tiempo de abort de 47 s a 33 s y de 16 replanes a 3.

El sistema cumple los requisitos de navegación autónoma reactiva para entornos con obstáculos parcialmente desconocidos.
