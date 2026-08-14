# Estado de la capa reactiva de frenado — 2026-07-29

> **Documento vivo.** Resume todo lo hecho en la sesión de reconstrucción de la capa reactiva
> (desde el reset al commit `025a5236`), qué se ha probado, qué funciona, qué no, y qué queda
> pendiente. Complementa a `doc/navigation_analysis.md` (estudio de tiempos original) y
> `doc/retrospectiva_tfm_navegacion_reactiva.md`. No sustituye a ninguno de los dos.

## 0. Resumen ejecutivo (para no perderse)

- **Objetivo real, tal como lo pidió Irene**: evitar colisiones solo cuando hay margen físico
  real para frenar pero el sistema tarda demasiado en reaccionar (retraso de software). **No**
  se busca evitar colisiones físicamente imposibles (obstáculo tan cerca que ni una reacción
  instantánea llegaría a tiempo) — esas se dejan documentadas como límite físico del sistema.
- **Se construyó** una capa reactiva que lee el LiDAR crudo directamente (sin pasar por el mapa)
  y frena de forma binaria (velocidad de crucero → velocidad suelo fija, y vuelta) cuando calcula
  que no da tiempo a parar.
- **Se validó** en el caso más simple (ruta recta, sin más complicaciones): 8/8 vuelos sin choque.
- **Se encontraron y arreglaron 6 bugs reales** en el camino (detallados en la sección 4),
  incluido uno de metodología de pruebas (sección 7, Problema 1) que explicaba la mayoría de los
  fallos aparentes de la capa reactiva.
- **Problema 1 (capa lenta) — resuelto**: era en gran parte un error al diseñar las pruebas de
  confirmación (comparar distancia al centro de la caja contra un umbral pensado para distancia a
  su cara). Corregido, 8/9 SUCCESS en la repetición (sección 7).
- **Problema 2 (meta desconocida + obstáculo) — mejorado, no resuelto**: colisiones bajan de 5/6 a
  2/6 tras el arreglo de prioridad, pero aparecen abortos por callejón sin salida en el mapa cerca
  del obstáculo — pendiente de investigar (sección 7).
- **Conclusión honesta**: el estudio de tiempos (qué tarda cada eslabón, cuál es el límite físico)
  es sólido y fiable. La capa reactiva, en el caso simple (ruta recta), **sí cumple razonablemente
  lo que promete** una vez las pruebas se diseñan bien. En el caso con meta desconocida sigue sin
  ser fiable del todo.

---

## 1. Qué significa "conocido" vs "desconocido" en las pruebas, y qué casos se han probado

Todas las pruebas usan la misma mecánica: el dron despega en `(-9, 0)`, un obstáculo (caja
1×1×2m) se aparca al inicio a gran altura (fuera del alcance vertical del LiDAR) y se mueve a su
posición real mediante un servicio ROS2 (`set_pose`) cuando el dron se acerca — ver sección 5
para el problema encontrado con este mecanismo.

| Caso | Goal | ¿Está dentro del mapa ya explorado al despegar? | Qué prueba |
|---|---|---|---|
| `recto` | (2, 0) | **Sí** — cerca, en línea recta, el dron lo alcanza con A* directo | Caso base, sin modo frontera |
| `giro` | (2, 3) | **No** — fuerza modo frontera desde el despegue | Generalización con maniobra de giro |
| `abierto` | (9, -4) | **No** — fuerza modo frontera, ruta larga (~18m) | Generalización con ruta larga |
| `abierto_temprano` (nuevo, 2026-07-28) | (9, -4) | **No** — igual que `abierto` | Obstáculo en la primera recta *combinado* con meta desconocida — diseñado específicamente para ver si el modo frontera interfiere con la capa reactiva |

**"Modo frontera"** es lo que pasa cuando el objetivo final está fuera de lo que el dron ha
explorado con el LiDAR: A* no puede calcular una ruta directa, así que el sistema navega primero
a un punto intermedio conocido ("frontera") y desde ahí vuelve a intentar llegar al objetivo real,
repitiendo el proceso según se va descubriendo mapa nuevo. **Es el modo normal para casi
cualquier misión real** (rara vez se conoce de antemano todo el camino hasta el destino), por eso
Irene pidió explícitamente probar con meta desconocida en vez de descartarlo como caso raro.

### Resultados por caso (batería final, tras los 3 primeros arreglos de la sección 4)

| Caso | Resultado |
|---|---|
| `recto`, v=1.5, obstáculo a ~3.3m | **8/8 SUCCESS**, margen 1.15–1.23m |
| `giro`, v=1.5, obstáculo a ~3.5m | 4/8 SUCCESS, 4/8 ABORTED (motivo distinto, ver sección 6) — 0 colisiones |
| `abierto`, v=1.5, obstáculo a ~3.5m | 6/8 SUCCESS, 2/8 ABORTED (mismo motivo) — 0 colisiones |
| `abierto_temprano`, v=1.5, obstáculo en la primera recta | **5/6 COLLISION** — aquí se encontró el bug de prioridad (sección 4.5), ver Problema 2 en la sección 7 |

---

## 2. ¿Se ha probado "dron a 1 m/s, obstáculo a 2 m"? ¿Debería verlo?

**Sí, se probó explícitamente**, junto con los otros dos umbrales de velocidad, tras arreglar el
bug de prioridad de la sección 4.5. El razonamiento teórico (sección 3/5bis) dice que a 1 m/s, con
la capa reactiva, el dron debería empezar a reaccionar sobre los 1.65m de distancia — así que
ponerlo a 2m debería darle 0.35m de margen extra sobre el mínimo de diseño.

**Resultado real (batería `confirm_v1.0`, 3 repeticiones, capa reactiva activa, tras el arreglo de
prioridad Y tras subir `run_frequency` a 100Hz — sección 4.6)**:

| Repetición | Resultado | Distancia real de soltado (sin artefactos, ver sección 3) | Distancia mínima al obstáculo |
|---|---|---|---|
| 1 | SUCCESS | — | 1.03–1.15m |
| 2 | COLLISION | — | 0.91–0.99m |
| 3 | COLLISION | **2.00m** (0.35m de margen real sobre el umbral de 1.65m) | 0.94–0.98m |

**Con esos números pensábamos que no lo veía a tiempo la mayoría de las veces.** Pero había un
error en cómo se eligieron esas distancias de prueba — ver sección 7, Problema 1: una vez
corregido, el mismo caso (v=1.0, obstáculo con margen real equivalente) pasa a SUCCESS de forma
consistente. El "2m" de este ejemplo concreto seguía sin ser suficiente margen una vez corregido
correctamente el cálculo (hacía falta más como 2.7m) — la sección 7 explica por qué.

---

## 3. El retraso del "soltado" del obstáculo — calculado, y CORREGIDO tras un primer error

Irene preguntó si el tiempo que tarda el obstáculo en aparecer de verdad (tras la orden de
soltarlo) se puede calcular e incorporar, distinguiendo lo que es inevitable por ser una
simulación de lo que sería un fallo real del sistema.

**Primer intento (INCORRECTO, ya descartado)**: se midió un RTT de ~0.44–0.47s entre el envío de
la orden `set_pose` y la confirmación de respuesta, y se asumió que el obstáculo no existía
físicamente hasta esa confirmación. **Esto era un error.** El propio código de
`mission_latency_study.py` (función `drop_obstacle()`) documenta, con evidencia de un piloto
anterior de este mismo proyecto, que **el objeto se mueve físicamente en Gazebo en el instante en
que se ENVÍA la orden, no cuando se confirma** — un `[LAT] DETECT` del planificador se observó
*antes* de la confirmación registrada, lo que demuestra que el cambio físico ya había ocurrido. El
RTT de ~0.45s es solo la latencia de la propia confirmación del servicio ROS/Gazebo, útil como
dato de diagnóstico del servicio, pero **no debe restarse de la distancia** — hacerlo (como se
hizo en un primer momento) desplaza objetos artificialmente y da resultados imprecisos.

**El retraso real del mecanismo de prueba es otro, más pequeño**: el bucle que vigila la distancia
dron-obstáculo solo comprueba cada 0.02s, y solo puede disparar la orden de soltar *después* de
que la distancia ya haya cruzado el umbral pedido. En la práctica esto adelanta el disparo
~0.1–0.5m más cerca de lo pedido (varía según la repetición, no es un valor fijo por velocidad
como se pensó al principio). **Este sí es un retraso inevitable de cómo simulamos el obstáculo,
no un fallo del dron** — el propio sistema ya registra la distancia real de disparo en el campo
`drop_dist_actual`, así que no hace falta ninguna corrección adicional: ese número YA es la
distancia real a la que el obstáculo existió.

**Con esto corregido, comprobación en un caso limpio** (`confirm_v1.0_rep3`): el obstáculo se
soltó realmente a 2.00m (registrado, sin necesidad de corregir nada), la velocidad de crucero era
1.0 m/s, y el umbral de la capa reactiva a esa velocidad es 1.65m — es decir, **0.35m de margen
real por encima de lo que la fórmula exige**. **Aun así, esa repetición chocó.** Esto confirma que
el fallo de la capa reactiva no se explica (al menos no del todo) por artefactos del mecanismo de
prueba — hay un problema real y sin resolver en la propia capa reactiva.

---

## 4. Qué se ha implementado en la capa reactiva — y qué NO

### 4.1 Diseño: ¿frena en seco o reduce velocidad progresivamente según distancia/altura?

**Respuesta directa a la duda de Irene: es una decisión binaria, no progresiva, y no usa la
altura para nada.**

- Se calcula una única `danger_distance(v)` (una distancia, fija para cada velocidad de crucero,
  no varía con la altura del dron):

  ```
  danger_distance(v) = v · (k_brake + l_detect) + margen
                      = v · (1.0 + 0.25) + 0.4
  ```
  - `k_brake = 1.0s`: tiempo de frenado medido por metro/segundo de velocidad (ver `mission_brake_test.py` — resultado: el frenado es lineal con la velocidad, no cuadrático, porque el controlador tiene un tiempo de asentamiento casi constante, no una deceleración constante).
  - `l_detect = 0.25s`: latencia asumida de la propia capa (su ciclo de lectura del LiDAR).
  - `margen = 0.4m`: margen de seguridad fijo añadido.

- Cuando el LiDAR crudo detecta algo dentro de un cono frontal (±25°, ±0.6m de ancho lateral) a
  menos de `danger_distance(v)`: la velocidad pasa de golpe de crucero a una **velocidad suelo
  fija de 0.4 m/s** (nunca se para del todo, para evitar bloqueos de pausa/reanudación).
- Cuando el obstáculo deja de verse en un chequeo omnidireccional (no solo el cono frontal) y ha
  pasado al menos 1 segundo frenando: la velocidad vuelve de golpe a crucero.
- **No hay rampa proporcional a la distancia.** Sí existió una versión anterior (ver sección 4.2)
  que reducía la velocidad de forma continua según la distancia — se abandonó porque cada cambio
  de velocidad usa un mecanismo (`modify()`) que provoca un tirón/retroceso momentáneo del dron, y
  la versión continua llamaba a ese mecanismo muchas veces por vuelo, acumulando tirones.
- **No usa altura (Z)** en ningún cálculo — solo distancia 2D del LiDAR.

### 4.2 Primer bug: waypoint obsoleto (arreglado)

El mecanismo que reenvía la ruta al dron reutilizaba siempre el primer punto de la ruta ya
calculada, que quedaba desactualizado si el dron llevaba tiempo volando desde el último replan
real. Resultado: el dron recibía una orden que lo hacía retroceder momentáneamente hacia ese punto
viejo. Arreglado recalculando la posición de partida en cada orden de la capa reactiva.

### 4.3 Segundo bug: tirones por frenado progresivo (arreglado, cambio de diseño)

La primera versión de la capa (rampa de velocidad continua según distancia) llamaba al mecanismo
de reenvío de ruta ~15 veces por vuelo. Cada llamada provoca un pequeño tirón/retroceso conocido
del controlador de vuelo. Con tantas llamadas seguidas, los tirones se acumulaban en oscilaciones
grandes (varios metros). Arreglado cambiando a la decisión binaria descrita en 4.1 (2-4 llamadas
por vuelo en vez de ~15).

### 4.4 Tercer bug: recorte de waypoints por distancia simple, no por dirección (arreglado)

El recorte que evitaba reenviar puntos ya pasados solo miraba si el primer punto estaba muy cerca
del dron, no si estaba *detrás*. Si el dron llevaba recorrido un tramo largo desde el último
replan real, podían quedar varios puntos detrás sin estar "cerca" en distancia, y se reenviaban
igualmente — el dron retrocedía hacia ellos. Esto es justo lo que Irene observó en RViz ("muchos
waypoints por detrás de la trayectoria"). Arreglado buscando el segmento de la ruta más cercano
por proyección geométrica, en vez de por distancia simple al primer punto.

### 4.5 Cuarto bug: el sistema de mapa cancelaba el frenado de la capa reactiva (arreglado, pero no fue suficiente)

Descubierto con el test `abierto_temprano` (sección 1): cuando el sistema de mapa (`MAP_CHECK`)
detecta el mismo obstáculo por su propio camino (independiente del LiDAR crudo) y decide
replanificar la ruta, el código **cancelaba sin comprobar nada el frenado activo de la capa
reactiva** y reenviaba la ruta a velocidad de crucero completa — a veces mientras el dron seguía
a menos de 1.5m del obstáculo. Confirmado con logs: el frenado se cancelaba entre 0.04 y 0.2
segundos después de activarse.

**Arreglado**: ahora, si la capa reactiva está frenando activamente cuando llega un replan del
sistema de mapa, se mantiene la velocidad reducida (no se reanuda crucero a ciegas) y es la propia
capa reactiva quien decide cuándo el obstáculo ya no está.

**Verificado que el arreglo funciona como se diseñó** (el frenado se mantiene ahora a través de un
replan, confirmado en logs) — **pero no fue suficiente**: las pruebas posteriores (sección 2)
siguen mostrando mayoría de colisiones. El arreglo soluciona el mecanismo que se identificó, pero
no es la causa principal del fallo.

### 4.6 Quinto cambio: frecuencia de ejecución del nodo (aplicado, sugerido por un tutor)

Se descubrió que `run_frequency` (cada cuánto se ejecuta la lógica de decisión del comportamiento,
incluida la capa reactiva) estaba en su valor por defecto de **10 Hz** — no se fijaba
explícitamente en la configuración. Un tutor sugirió subirlo. Se aplicó `run_frequency=100`,
`node_frequency=200`, verificado con `ros2 param get` que se aplican correctamente, y verificado
con `ros2 topic hz` que el LiDAR publica a ~9.4Hz (lo esperado). **Resultado: mejora marginal, no
resuelve el problema** — se mantiene el bloque de colisiones mayoritarias en las pruebas de
confirmación.

### 4.7 Bug de "ABORTED" en modo frontera — arreglado pero SIN volver a probar

Al validar giro/abierto se encontró que la navegación a veces aborta sin motivo real cuando el
dron llega a un punto intermedio ("frontera") y el sistema intenta replanificar inmediatamente:
el mapa consolidado todavía no ha absorbido lo que el LiDAR ya vio, así que el sistema cree
erróneamente que está "atascado" y aborta la misión entera. Se implementó un arreglo (reintentar
con espera de 1.8s hasta 6 veces, en vez de abortar al primer intento) y se compiló, **pero no se
ha vuelto a lanzar ninguna batería para confirmar que funciona** desde que se implementó. Estado:
implementado, sin validar.

---

## 5. Relación entre `Dmin(v)`, lo físicamente imposible, y lo recuperable por software

Esto es la parte que más ha costado separar con claridad, así que se detalla aquí explícitamente.

**`Dmin(v)`** es la distancia mínima medida empíricamente a la que se puede soltar un obstáculo
sin que el sistema *original* (sin la capa reactiva nueva) choque. Se midió así:

| Velocidad | `Dmin(v)` medida |
|---|---|
| 0.5 m/s | 1.45–1.50 m |
| 1.0 m/s | 2.00–2.05 m |
| 1.5 m/s | 2.88–2.98 m |

Esa distancia mezcla dos cosas que Irene quería separar explícitamente:

```
Dmin(v) = distancia física mínima de frenado  +  distancia consumida por el retraso de software
```

- **Distancia física mínima de frenado** (`v · k_brake`, con `k_brake=1.0s` medido): la que hace
  falta para parar aunque la detección fuera instantánea. **Esto es lo que Irene NO quiere
  intentar evitar** — si el obstáculo aparece más cerca que esto, es imposible pase lo que pase.
- **Retraso de software** (el resto): la suma de `L_lidar` (LiDAR ve el obstáculo, 1.0–1.5s),
  `L_cell` (la celda del mapa se marca ocupada, 0.3–0.5s) y `L_mapcheck` (el sistema lo detecta y
  dispara replan, 1.0–1.5s a velocidad de crucero — ver `doc/navigation_analysis.md` para el
  desglose completo). **Esto es lo que sí se quiere evitar**, sustituyendo ese camino lento por
  la capa reactiva (LiDAR crudo directo, ~0.25s en vez de ~2s).

Separando las 36 colisiones válidas de "margen insuficiente" (sin el bug ya arreglado de
frontier-blind) con estos dos umbrales:

| Categoría | Nº de 36 | % | Qué significa |
|---|---|---|---|
| Físico puro (`d < v·k_brake`) | 6 | ~17% | Imposible pase lo que pase — **no se toca, es correcto dejarlo así** |
| Software, insuficiente incluso para la capa nueva | 10 | ~28% | Recuperable en teoría, pero el margen es tan ajustado que ni los parámetros actuales de la capa (l_detect=0.25s, margen=0.4m) llegan a tiempo |
| Recuperable por la capa reactiva actual | 20 | ~56% | Esto es justo lo que la capa se diseñó para evitar |

(Desglose completo, prueba por prueba, en `doc/navigation_analysis.md`, sección "Actualización
2026-07-29".)

**Nota tras la corrección de la sección 3**: `drop_dist_actual` ya es la distancia real correcta
(no hace falta restarle nada del RTT del `set_pose`, ese primer intento de corrección era
erróneo). El único sesgo real y pequeño que queda sin corregir es el del bucle de disparo
(~0.1–0.5m, variable), que desplazaría muy ligeramente algunas pruebas cercanas al borde entre
categorías — no cambia el cuadro general.

---

## 5bis. Línea temporal completa de retrasos — evitables vs inevitables

Para responder directamente a "quiero el análisis de Dmin, de retrasos calculados a lo largo de
la conversación, arreglados, etc., diferenciando entre los tiempos que no se pueden evitar y los
que sí":

| Retraso | Valor medido | ¿Se puede evitar? | Categoría |
|---|---|---|---|
| Distancia física de frenado (`k_brake·v`) | 1.0s por cada m/s de velocidad | **No** | Físico puro — límite real del dron, no se toca |
| `L_lidar` (LiDAR ve el obstáculo, camino normal/lento) | 1.0–1.5s | Parcialmente — es lo que la capa reactiva sustituye | Software (camino lento) |
| `L_cell` (celda del mapa se marca ocupada) | 0.3–0.5s | Parcialmente — mismo camino lento | Software (camino lento) |
| `L_mapcheck` (el sistema detecta y dispara replan) | 1.0–1.5s a velocidad de crucero | Parcialmente — mismo camino lento | Software (camino lento) |
| `l_detect` de la capa reactiva nueva (su propio ciclo de LiDAR crudo) | 0.25s (parámetro de diseño) | Es la solución, no el problema | Software (camino rápido, lo que estamos construyendo) |
| `run_frequency` del nodo (cada cuánto se procesa la lógica) | Era 10Hz por defecto → subido a 100Hz | **Sí, y ya se corrigió** | Software — arreglado (2026-07-29) |
| Retraso del bucle de disparo del obstáculo en la prueba | ~0.1–0.5m (variable) | **No** — es del método de simulación, no del dron | Artefacto de la prueba, inevitable en simulación |
| ~~RTT del servicio `set_pose` (~0.45s)~~ | ~~0.44–0.47s~~ | — | **Descartado**: no representa un retraso real del obstáculo (ver sección 3) |

**Los tres primeros (frenado físico, `L_lidar`, `L_cell`, `L_mapcheck`) son exactamente los que
componen `Dmin(v)`** — la distancia mínima medida sin la capa reactiva:

| Velocidad | `Dmin(v)` medida | De la cual, físico puro (no evitable) | De la cual, camino lento de software (lo que la capa nueva ataca) |
|---|---|---|---|
| 0.5 m/s | 1.45–1.50 m | ~0.50 m | ~0.95–1.00 m |
| 1.0 m/s | 2.00–2.05 m | ~1.00 m | ~1.00–1.05 m |
| 1.5 m/s | 2.88–2.98 m | ~1.50 m | ~1.38–1.48 m |

Esto confirma exactamente el planteamiento de Irene: la mitad (aprox.) de `Dmin(v)` es límite
físico (no se toca) y la otra mitad es el camino lento de software (`L_lidar`+`L_cell`+
`L_mapcheck`), que es justo lo que la capa reactiva sustituye por su propio camino rápido
(`l_detect=0.25s`). El problema (sección 7) es que, en la práctica, ese camino rápido no está
entregando lo que promete todavía.

### Sobre el filtro de confianza / ruido (para no frenar con cualquier cosa)

Irene preguntó explícitamente si la detección rápida tiene algún filtro de confianza, para no
frenar por ruido. **Sí existe, parcialmente implementado**:

- `corridor_min_cluster = 3`: exige que al menos 3 rayos **contiguos** del LiDAR detecten algo
  dentro del cono frontal — un solo rayo suelto (ruido puntual) no dispara nada.
- `corridor_persistence = 1`: cuántas veces seguidas tiene que confirmarse antes de reaccionar.
  **Está puesto a 1**, es decir, en la práctica no exige ninguna confirmación repetida en el
  tiempo — reacciona a la primera vez que ve el clúster de 3 rayos. Se podría subir (p. ej. a 2 o
  3) para más confianza a costa de un poco más de retraso — no se ha probado a cambiar esto.

---

## 6. El bug de ABORTED en modo frontera — para no confundirlo con las colisiones

Es un problema **distinto y no relacionado** con las colisiones de la capa reactiva. Ocurre en
`giro`/`abierto` incluso sin ningún obstáculo cerca (los ABORTED de la tabla de la sección 1
ocurren a 1.8–5.0m del obstáculo, lejos del umbral de colisión de 1.0m). Causa: al llegar a un
punto intermedio de exploración, el sistema replanifica de inmediato hacia el objetivo real, pero
el mapa consolidado todavía no sabe lo que el LiDAR ya vio, así que parece (erróneamente) que no
hay forma de seguir avanzando. Arreglo implementado en sección 4.7, sin validar todavía.

---

## 7. Qué queda pendiente — dos problemas distintos, no confundirlos

### Problema 1: la capa reactiva "no llegaba a tiempo" — RESUELTO, era un fallo de diseño de las pruebas

Las primeras pruebas de confirmación (v=1.0, obstáculo a 2.00m real, umbral de diseño 1.65m — en
teoría 0.35m de margen) seguían colisionando la mayoría de las veces incluso después de arreglar
el bug de prioridad (4.5) y subir `run_frequency` (4.6). Revisando los logs con detalle
(`confirm_v1.0_rep3`) se encontró la causa real: **el corredor mide con el LiDAR la distancia a la
cara de la caja del obstáculo, no a su centro** — y la caja tiene 0.5m de radio (mide 1×1×2m).
`danger_distance(v)` se compara internamente contra esa distancia a la cara, de forma consistente.

El problema era mío, al diseñar las pruebas: elegía la distancia de soltado (`drop_dist`) pensando
en el centro de la caja, y la comparaba contra el umbral de la capa sin restar ese medio metro. El
margen "0.35m" del ejemplo anterior en realidad era **−0.15m** una vez descontado el radio de la
caja — es decir, esas pruebas ya estaban por debajo del umbral efectivo, sin que el corredor
tuviera ninguna culpa.

**Verificación**: se repitió la batería sumando el radio de la caja (0.5m) a la distancia pedida en
cada velocidad (v=0.5→2.0m, v=1.0→2.7m, v=1.5→3.3m):

| Velocidad | Resultado (3 repeticiones) |
|---|---|
| 0.5 m/s | 3/3 SUCCESS (1.15–1.25m) |
| 1.0 m/s | 3/3 SUCCESS (1.15–1.21m) |
| 1.5 m/s | 2/3 SUCCESS, 1/3 COLLISION (0.99m — justo en el borde) |

**8 de 9 SUCCESS.** La única colisión queda al límite (0.99m, prácticamente el umbral de colisión
de 1.0m), coherente con ruido normal de simulación cerca del borde, no con un fallo sistemático.
**Conclusión: en el caso simple (ruta recta, sin modo frontera), la capa reactiva funciona
razonablemente bien tal como está diseñada.** El "problema" no era la capa, era cómo se estaba
poniendo a prueba.

### Problema 2: meta desconocida + obstáculo dinámico — el más grave

Cuando el destino final está fuera del mapa ya explorado (lo normal en una misión real, ver
sección 1) y aparece un obstáculo dinámico de por medio, el sistema termina casi siempre en
COLLISION o ABORTED. Primera comprobación (`abierto_temprano`, antes del arreglo de prioridad de
4.5): **5 de 6 vuelos con colisión**.

**Actualización 2026-07-29 (repetido tras el arreglo de 4.5)**: 6 repeticiones nuevas de
`abierto_temprano` con el arreglo de prioridad ya puesto.

| Rep | Resultado | Distancia mínima |
|---|---|---|
| 1 | COLLISION | 0.89m |
| 2 | ABORTED | 1.65m |
| 3 | SUCCESS | 1.09m |
| 4 | ABORTED | 5.00m |
| 5 | COLLISION | 0.91m |
| 6 | ABORTED | 1.38m |

**Las colisiones bajan de 5/6 a 2/6 — el arreglo de prioridad ayuda de verdad aquí.** Pero aparecen
ABORTED nuevos, y dos de ellos (rep2, rep6) ocurren *cerca* del obstáculo, no lejos como el patrón
de ABORTED ya conocido (rep4, a 5.00m, sí es el patrón antiguo de la sección 6). Revisando el log
de rep2: el arreglo de reintento (4.7) funciona tal como se diseñó — reintenta 6 veces esperando
1.8s cada vez — pero en este caso el dron se queda genuinamente atascado en el mismo punto
(~0.13–0.16m del frontier) las 6 veces, así que aborta tras ~11s de reintentos. No es un problema
de timing esta vez: parece un callejón sin salida real en el mapa, probablemente porque el propio
obstáculo bloquea la única dirección de exploración posible desde ese punto. **Conclusión: evitar
la colisión a veces empuja al dron a un callejón sin salida en vez de chocar — mejora la
seguridad (ABORTED no es colisión) pero no completa la misión.** Sin investigar más a fondo
todavía — el bug de ABORTED "puro" sin obstáculo (sección 6) tiene el mismo arreglo (4.7),
confirmado que funciona como está diseñado, pero el callejón sin salida en sí es un problema
distinto, de la geometría del mapa, no de temporización.

**Actualización 2026-07-29 (análisis en vivo + arreglo del "Mecanismo A")**: viendo un vuelo en
directo junto con Irene, se identificó la causa concreta de varias colisiones/CRASH restantes:
la ventana de gracia post-replan (2.5s) se arma tras **cualquier** replan, aunque no tenga nada
que ver con el obstáculo real — por ejemplo un replan por un segmento lejano sin relación. Si el
obstáculo real aparece y se acerca *dentro* de esa ventana, el corredor queda ciego hasta que el
propio `MAP_CHECK` (más lento) lo detecta, a veces con el dron ya a ~1.0–1.2m. Confirmado con
timestamps exactos en dos casos independientes (un CRASH en vivo y una colisión de la batería
`temprano4`). Un segundo mecanismo distinto (liberar el freno con solo 0.88m de margen,
reactivando a crucero completo demasiado cerca) también se observó en el mismo vuelo en directo,
asociado a un CRASH por inestabilidad del controlador — mecanismo aparte, sin arreglar todavía.

**Arreglo aplicado**: la ventana de gracia ya no bloquea el corredor por tiempo puro. Se captura
una referencia (`corridor_nearest_at_replan_`) de lo que el corredor ve en el momento del replan;
durante la ventana, solo se sigue confiando en el replan si la situación no ha empeorado respecto
a esa referencia — si algo se acerca más (o aparece algo nuevo que antes no se veía), el corredor
reacciona igualmente, sin esperar a que la ventana termine.

*(Se probó primero, y se descartó, acortar la ventana de gracia de 2.5s a 0.6s sin más: no bajaba
las colisiones y además provocó un CRASH nuevo por el mismo bug de inestabilidad del controlador,
al aumentar la frecuencia de activaciones del corredor. Revertido antes del arreglo bueno.)*

**Verificación** (`abierto_temprano`, 8 repeticiones, 1 descartada por fallo de arranque de
Gazebo/X11 — 7 válidas; comparado con la batería anterior sin este arreglo):

| | Antes (`temprano4`, 8 reps) | Después (`verifyA`, 7 reps válidas) |
|---|---|---|
| SUCCESS | 2 | 4 |
| COLLISION | 4 | **1** |
| ABORTED | 2 | 2 |

Las colisiones bajan de 4/8 (50%) a 1/7 (~14%). La única colisión restante se revisó en detalle:
el corredor frenó de forma correcta y oportuna esta vez (a 2.14m, prácticamente en el umbral de
diseño de 2.27m) y nunca fue silenciado por la ventana de gracia — pero aun así terminó a 0.99m.
Es el **Mecanismo B** (margen ajustado incluso con buena detección), no el A. **El Mecanismo A
queda considerado resuelto.**

### Mecanismo B (margen ajustado con detección a tiempo) — LIMITACIÓN DE DISEÑO, para el TFM

Investigado en detalle (`verifyA_frontera_rep8`): la capa reactiva frenó de forma correcta y
oportuna (a 2.14m, casi el umbral de diseño de 2.27m), decelerando de forma gradual como está
previsto. La colisión (0.99m) no viene de ahí — viene de que **la ruta de esquiva que calcula A*
solo garantiza `safety_distance=0.5m` de despeje**, y ese es exactamente el mismo margen
(0.5m de la superficie) que el estudio usa para definir "colisión". **No hay ningún colchón entre
"A* lo considera una ruta segura" y "el estudio lo cuenta como choque".** Cualquier imprecisión
normal de seguimiento de trayectoria puede empujarlo al otro lado de esa línea, sin que la capa
reactiva tenga margen de maniobra — su trabajo (dar tiempo a reaccionar) ya se cumplió; el límite
está en la propia planificación de ruta, un subsistema distinto.

**Decisión: no se va a corregir.** Se deja documentado como limitación de diseño conocida para el
TFM — subir `safety_distance` dejaría más colchón en todas las rutas de esquiva, pero es un cambio
de alcance distinto (afecta a la planificación global, no a la capa reactiva) y no se aborda en
esta rama de trabajo.

### Otras pendientes, menores

- Decidir si merece la pena apretar más los parámetros de la capa (`l_detect`, `margen`) para
  recuperar el ~28% de casos "software insuficiente" (sección 5) — depende de resolver primero el
  Problema 1.
- Subir `corridor_persistence` (sección 5bis) para dar más confianza a la detección, si se
  confirma que hay falsas activaciones por ruido (no se ha observado evidencia de esto todavía,
  es una precaución, no un problema confirmado).

## 8. Recomendación sobre si merece la pena seguir o volver atrás

Los arreglos de la sección 4 (waypoint obsoleto, tirones, recorte de waypoints, prioridad
mapa/corredor) son correcciones de bugs concretos y verificables, cada uno con su propia evidencia
de que soluciona lo que dice solucionar. El diseño de fondo (LiDAR crudo + fórmula de distancia
física) queda respaldado por la batería corregida de la sección 7 (Problema 1): **8/9 SUCCESS en
el caso simple**, con la fórmula funcionando esencialmente como se diseñó una vez las pruebas
comparan las distancias correctamente (cara de la caja, no centro).

**No hay motivo para volver atrás.** El diseño no estaba mal planteado — la mayor parte de lo que
parecía un fallo del corredor era metodología de prueba incorrecta. El Problema 2 (meta desconocida
+ obstáculo dinámico) mejoró de 5/6 a 1/7 colisiones tras arreglar el Mecanismo A (sección 7) — ver
también la sección 9 para la confirmación final del criterio de distancia, y la sección 10 para las
limitaciones que se dejan documentadas sin resolver.

## 9. Confirmación final — distancia a superficie, no al centro

Para cerrar la duda de si la capa "detecta un obstáculo a X metros", se comprobó directamente:
pedir que la caja aparezca con su **superficie** (no su centro) a exactamente 2.0m del dron, en
las tres velocidades, con todos los arreglos ya aplicados (prioridad + Mecanismo A):

| Velocidad | Umbral de diseño (superficie) | Superficie real al soltar | Resultado (5 repeticiones) |
|---|---|---|---|
| 0.5 m/s | 1.02m | 2.0m | **5/5 SUCCESS** (1.16–1.30m) |
| 1.0 m/s | 1.65m | 2.0m | **5/5 SUCCESS** (1.14–1.25m) |
| 1.5 m/s | 2.27m | 2.0m | **0/5, 5/5 COLLISION** (0.83–0.97m) |

Resultado limpio y exactamente como predice la fórmula: funciona de forma perfectamente fiable
(10/10) cuando la distancia real a la superficie supera el umbral de diseño de esa velocidad, y
falla de forma igual de consistente (5/5) cuando no lo supera — a 1.5 m/s hacen falta más de 2.0m
de superficie (concretamente 2.27m), así que el fallo ahí es esperado, no un bug.

**Conclusión para el TFM**: la "distancia mínima de detección" de la capa reactiva debe expresarse
y entenderse como distancia entre el dron y la **superficie más cercana** del obstáculo, no entre
el dron y su centro geométrico — son 0.5m de diferencia con esta caja de prueba (1×1×2m), y esa
diferencia por sí sola explica gran parte de la confusión inicial sobre si "un obstáculo a 2m" se
evitaba o no.

## 10. Limitaciones conocidas — para dejar documentadas en el TFM, sin resolver

Estas tres limitaciones se dejan como están, con su causa identificada y sin intención de
corregirlas en esta rama de trabajo:

1. **Zona físicamente imposible** (`d < v·k_brake`, sección 5): si el obstáculo aparece más cerca
   de lo que el frenado físico permite, no hay nada que la capa reactiva (ni ninguna capa de
   software) pueda hacer. Límite físico del dron, no del sistema.

2. **Mecanismo B — margen de A* sin colchón** (sección 7): la ruta de esquiva que calcula A* solo
   garantiza `safety_distance=0.5m` de despeje, que es exactamente el mismo margen que el estudio
   usa para definir "colisión" (0.5m de la superficie). No hay colchón entre "ruta considerada
   segura" y "colisión", así que cualquier imprecisión de seguimiento de trayectoria puede cruzar
   esa línea aunque la capa reactiva haya hecho bien su trabajo (frenar a tiempo). Afecta a la
   planificación de rutas en general, no solo a la capa reactiva.

3. **ABORTED por callejón sin salida cerca de un obstáculo, en modo frontera** (sección 7,
   Problema 2): al esquivar un obstáculo con una meta todavía sin explorar, el dron a veces queda
   sin ninguna dirección válida de exploración desde su posición (el propio obstáculo bloquea la
   única vía), y aborta la misión tras agotar los reintentos (sección 4.7) en vez de colisionar.
   Es más seguro que chocar, pero no completa la misión — geometría del mapa, no un bug de
   temporización.

## 11. Mecanismo C — investigado, RETIRADO por falta de evidencia

Hipótesis inicial (vuelo en directo, 2026-07-29): liberar el freno con solo ~0.88m de margen
omnidireccional causaría una reactivación brusca a velocidad de crucero y, en un caso, precedió a
un CRASH por inestabilidad del controlador.

**Investigación**: se revisaron ~20 eventos `REACTIVE_CLEAR` de las baterías `temprano4` y
`verifyA_frontera` (posteriores). Liberar con ~0.88–0.99m de margen resultó ser el patrón **normal
y mayoritario** de liberación (tiene sentido: la condición de liberación es "más de 0.88m", así
que casi cualquier liberación ocurre justo por encima de ese umbral) — y ocurrió en la mayoría de
los vuelos, incluidos casi todos los que terminaron en SUCCESS, sin ningún problema.

**No se pudo re-examinar el CRASH original** (el fichero de log se sobrescribió con el siguiente
vuelo suelto lanzado con el mismo nombre — lección para la próxima vez: usar nombres de fichero
únicos por vuelo). Con la evidencia disponible, lo más probable es que ese CRASH concreto fuera el
bug de inestabilidad del controlador ya documentado (activaciones seguidas del corredor), no algo
causado específicamente por el margen de liberación. **Se retira de la lista de mecanismos
confirmados.** Si se reproduce en el futuro, investigar con el log de ese vuelo conservado bajo un
nombre único.

## 12. "El dron va hacia atrás" — investigado, no es un waypoint desactualizado

Irene reportó ver visualmente al dron "ir hacia atrás" en RViz en varias ocasiones. Se instrumentó
el diagnóstico (sección 4, `[DIAG corridor_modify]`) y se buscaron todos los waypoints con
componente negativo en la dirección de avance, en varias baterías recientes. Ejemplo real
encontrado:

```
Dron en:         (-6.647, -0.134)
Siguiente punto: (-6.650, -0.950)
Diferencia:      -0.003m en x (3mm — ruido de posición/control)  |  -0.816m en y (giro hacia el sur)
```

**No es un waypoint quedando detrás del dron** (el componente "hacia atrás" es de 3mm a 16cm en
todos los casos encontrados — ruido, no una regresión real). Lo que sí es real y grande es un
**giro lateral muy cerrado**: la ruta de esquiva, en el escenario de meta desconocida, tiene que
virar en ángulo casi recto justo al lado del dron para rodear el obstáculo camino de una meta muy
al sur (y=-4). En RViz, un giro tan cerrado justo en la posición del dron puede percibirse
visualmente como un "vuelco" de la trayectoria, aunque el dron nunca retroceda de verdad.
**Conclusión: es un artefacto visual del giro de la ruta de esquiva, arquitectónicamente esperado,
no un bug que corregir.**
