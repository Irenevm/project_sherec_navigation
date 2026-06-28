# Problema: Retraso del mapa probabilístico en detección de obstáculos dinámicos

## Contexto

En el sistema de navegación reactiva implementado para el TFM, el dron usa un planner A*
que consulta periódicamente el mapa de ocupación para detectar si un obstáculo ha aparecido
en la ruta planificada (Hito B). Sin embargo, se observó que el dron se acercaba peligrosamente
a obstáculos **que ya eran visibles para el LiDAR** antes de que el sistema los detectara como bloqueantes.

---

## Origen del problema: el mapa es probabilístico

El mapa de ocupación (`OccupancyGrid`) que usa el sistema no es binario (libre/ocupado).
Es **probabilístico**: cada celda tiene un valor de probabilidad de estar ocupada que se
acumula con cada scan del LiDAR. El sistema solo marca una celda como `OCUPADA` cuando
esa probabilidad supera el **30%**.

Esto implica que aunque el LiDAR vea el obstáculo en el primer scan, el mapa no lo considera
obstáculo hasta que varios scans confirman su presencia.

```
LiDAR girando continuamente:

  scan 1  →  celda probabilidad = 10%   ← mapa dice: LIBRE
  scan 2  →  celda probabilidad = 22%   ← mapa dice: LIBRE
  scan 3  →  celda probabilidad = 35%   ← mapa dice: OCUPADA ✓
                                                ↑
                              Solo aquí MAP_CHECK detecta el obstáculo
```

---

## Cómo funciona la detección (MAP_CHECK)

El sistema tiene un timer (`MAP_CHECK`) que se dispara cada **0.5 segundos**.
En cada tick:

1. Llama a `on_activate()` → reconstruye el grafo A* desde el mapa actual (`last_occ_grid_`)
2. Recorre los waypoints del camino planificado
3. Para cada waypoint, llama a `is_occupied()` → consulta el grafo recién reconstruido
4. Si algún waypoint está ocupado → lanza **Hito B**: cancela la navegación actual y replana

El mapa que consulta es el más actualizado disponible en ese momento. El problema **no está
en el código de detección**, sino en la latencia del mapa en sí.

---

## Prueba empírica del retraso (datos reales del log)

### Configuración del experimento

- Obstáculo dinámico en **(6.5, -3.8)** — tamaño 1×1×2m
- El dron navega hacia goal en **(9.0, -4.0)**
- MAP_CHECK cada 0.5s
- El dron cruza a la sala derecha por Hito A (camino directo visible) llegando a ~**(4.7, -3.5)**

### Secuencia observada en el log

```
Posición dron  →  Resultado MAP_CHECK
───────────────────────────────────────────────────────
(5.90, -3.10)  →  All waypoints free   ← mapa: prob < 30%
(5.88, -3.11)  →  All waypoints free   ← mapa: prob < 30%
(5.90, -3.10)  →  All waypoints free   ← mapa: prob < 30%
(5.90, -3.10)  →  PATH TO GOAL BLOCKED ← mapa: prob ≥ 30% !!
      ↑                    ↑
  Sin moverse       Sin cambio en el código —
                    solo cambió la acumulación
                    de probabilidad en el mapa
```

**Conclusión directa del log**: durante 3 ticks consecutivos (1.5 segundos), el dron
**no se movió** pero el resultado cambió. El único cambio fue que el mapa acumuló
suficientes scans para superar el umbral del 30%.

### Diagrama espacial del momento crítico

```
         x=5.0    x=5.9    x=6.5    x=9.0
          │        │        │         │
y=-3.0   ─┼────────┼────────┼─────────┼──
          │        │        │         │
y=-3.1   ─┼────────●────────┼─────────┼──   ← DRON AQUÍ (5.90, -3.10)
          │    (posición     │         │
y=-3.5   ─┼────dron)────────┼─────────┼──
          │                 │         │
y=-3.8   ─┼─────────────────[OBSTÁCULO]──   ← (6.5, -3.8) — zona 1×1m
          │                 │         │
y=-4.0   ─┼─────────────────┼─────────★──   ← GOAL (9.0, -4.0)

Distancia dron → centro obstáculo: ~0.9m
Borde obstáculo inflado (safety 0.5m): a ~0.4m del dron
→ El dron estaba ya DENTRO de la zona de seguridad del obstáculo
  cuando el mapa finalmente lo detectó.
```

---

## ¿Por qué tardó tanto?

El obstáculo se spawneó en Gazebo a `t=20s`. Pero:

1. **El dron estaba aún en la sala izquierda** cuando se spawneó — el LiDAR no llegaba
   al obstáculo porque la pared divisoria lo bloqueaba.
2. Cuando el dron cruzó a la sala derecha (Hito A, ~`t=30s`), el LiDAR empezó a ver el obstáculo.
3. Necesitó **varios scans** para acumular probabilidad ≥ 30%.
4. Solo entonces MAP_CHECK lo detectó.

Tiempo total entre "LiDAR empieza a ver el obstáculo" y "mapa lo marca como ocupado":
**~1.5 segundos** (3 ticks × 0.5s).

A una velocidad de **1.5 m/s**, el dron recorre **~2.25 metros** durante ese retraso.

---

## Implicaciones para el TFM

### ¿Es un bug?

**No.** Es una característica inherente de los mapas probabilísticos (como los generados
por `slam_toolbox` o `nav2_map_server`). El comportamiento probabilístico existe precisamente
para evitar falsos positivos: que ruido de un scan individual marque celdas libres como ocupadas.

### ¿Es un problema de la navegación reactiva?

**Parcialmente.** El sistema reactivo (Hito B) funciona correctamente: detecta el obstáculo
en el primer tick después de que el mapa lo confirma. El problema es que el **tiempo de
confirmación del mapa** introduce una latencia estructural.

### ¿Qué margen de seguridad necesita el sistema?

Para que el sistema pueda frenar o replanificar antes de colisionar:

```
margen mínimo = retraso_mapa + tiempo_reacción_planner + distancia_frenado
             ≈ 1.5s          + 0.5s                    + (depende velocidad)

A 1.5 m/s: margen ≈ (1.5 + 0.5) × 1.5 = 3.0m entre el dron y el obstáculo
```

El obstáculo en el experimento estaba a ~0.9m cuando se detectó → **insuficiente**.

---

## Referencia en el código

- **Timer MAP_CHECK**: [path_planner_behavior.cpp](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/src/path_planner_behavior.cpp) — función `on_map_check_timer()`
- **Periodo del timer**: [behavior_default.yaml](../../../aerostack2_ws/src/aerostack2/as2_behaviors/as2_behaviors_path_planning/config/behavior_default.yaml) — parámetro `map_check_period: 0.5`
- **Umbral de ocupación 30%**: configurado en el stack de navegación de ROS2 (`nav2` / `slam_toolbox`)
- **Reconstrucción del grafo**: `on_activate()` → `update_grid()` → lee `last_occ_grid_`
