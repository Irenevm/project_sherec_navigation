# Entendiendo por qué el dron a veces choca (y por qué a veces no)

> Este documento no es un informe científico — es la explicación que te daría un compañero
> sentado a tu lado delante de los datos, para que entiendas de verdad qué está pasando dentro del
> dron cuando aparece un obstáculo delante de él. Si quieres los números exactos, tablas y
> metodología, están en `navigation_analysis.md`. Aquí lo que quiero es que, al terminar de leer,
> tengas la imagen mental completa.

## La pregunta que de verdad importa

Cuando decimos "el dron chocó", en realidad estamos mezclando dos situaciones completamente
distintas, y la diferencia entre ellas es el corazón de todo este estudio:

1. **"El algoritmo falló."** Había tiempo de sobra para esquivar el obstáculo, pero algo en el
   software no funcionó como debía (un bug, una detección que no saltó cuando debía).
2. **"Era físicamente imposible."** El obstáculo apareció tan cerca que, aunque el algoritmo
   hubiera sido perfecto — detección instantánea, replanificación instantánea — el dron ya no
   tenía margen suficiente para frenar o desviarse a tiempo. No es un bug, es física: masa,
   velocidad e inercia.

Casi todo lo que hemos medido en este estudio consiste en averiguar, para cada choque, en cuál de
las dos categorías cae. Y la respuesta, como vas a ver, es que **las dos cosas pasan**, pero en
proporciones y circunstancias muy distintas.

## Lo que de verdad pasa entre "el obstáculo aparece" y "el dron reacciona"

Imagina el obstáculo apareciendo de golpe delante del dron (en nuestros experimentos literalmente
lo teletransportamos ahí, para simular algo que aparece de repente — una persona que cruza, otro
robot, lo que sea). A partir de ese instante, pasan varias cosas en cadena, cada una con su propio
retraso:

**Paso 1 — El LiDAR tiene que "verlo".**
El LiDAR del dron gira dando vueltas completas 10 veces por segundo. No es una cámara que ve todo
de golpe: es un rayo que barre. Así que, en el peor caso, pasa hasta una décima de segundo antes de
que el rayo pase exactamente por donde está el obstáculo. Además, nuestros datos muestran que el
sistema necesita ver el obstáculo de forma "sostenida" (no un solo destello) para fiarse de que es
real y no ruido — eso añade otro poco de tiempo. En total, en las pruebas, **desde que el
obstáculo existe hasta que el LiDAR lo confirma pasa alrededor de 1 a 1.5 segundos**, de forma
bastante constante, mientras el obstáculo esté a menos de 4-5 metros.

**Paso 2 — Esa "visión" tiene que convertirse en un mapa.**
El LiDAR no le dice directamente al planificador de rutas "hay algo ahí". Lo que hace es ir
"pintando" una casilla de un mapa de cuadrícula (como un tablero de ajedrez de 30x30 cm... bueno,
en realidad de 10x10 cm) cada vez que un rayo rebota en ese punto. Cuando esa casilla acumula
suficiente "confianza" de que hay algo sólido ahí, se marca como ocupada. Esto es rápido — en
nuestras pruebas, **entre 0.3 y 0.5 segundos** desde que el LiDAR ve el obstáculo hasta que la
casilla del mapa queda marcada como ocupada.

Hasta aquí, vamos por **1.5-2 segundos** en total, y esta parte apenas cambia con la distancia:
tarda lo mismo si el obstáculo está a 1 metro que a 4.

**Paso 3 — El planificador tiene que darse cuenta de que su ruta actual pasa por esa casilla.**
Aquí es donde la cosa se pone interesante, y donde estaba nuestra sorpresa más importante del
estudio. Uno pensaría que el planificador revisa el mapa constantemente y en cuanto ve una casilla
ocupada en su camino, reacciona. Pero no es así del todo.

El sistema tiene un "vigilante" (lo llamamos `MAP_CHECK`) que se despierta cada medio segundo a
mirar si el camino sigue libre. Medio segundo no suena a mucho. Pero investigando el código
encontramos algo que no esperábamos: **ese vigilante solo empieza a revisar el camino en detalle
después de que el dron haya volado 1.5 metros desde la última vez que replanificó** la ruta. Es
como una especie de "periodo de calentamiento" tras cada replanificación.

¿Por qué importa esto? Porque **1.5 metros a 0.3 m/s son 5 segundos**, pero **1.5 metros a
1.5 m/s son solo 1 segundo**. Es decir: esta parte de la cadena no es un tiempo fijo — es una
**distancia fija convertida en tiempo según lo rápido que vueles**. Cuanto más rápido vuela el
dron, menos tiempo tarda en "recorrer" ese calentamiento, así que en la práctica esta etapa es más
rápida a velocidades altas que a velocidades bajas. Esto explica algo que a primera vista parece
contraintuitivo: **no es que ir más rápido sea automáticamente "más peligroso" en esta etapa
concreta** — de hecho, en esta etapa concreta, ir más rápido ayuda. El problema, como veremos, está
en otro sitio.

**Paso 4 y 5 — Replanificar y aceptar la nueva ruta.**
Una vez que el vigilante detecta el problema, calcular una ruta nueva (el algoritmo A*) y que el
controlador de vuelo la acepte son pasos rápidos — unas pocas décimas de segundo en total. No es
aquí donde se pierde tiempo.

## Entonces, ¿por qué a veces no da tiempo?

Aquí está la clave de todo. Aunque el "calentamiento" de 1.5 metros se recorre más rápido cuanto
más rápido vuela el dron, **el dron TAMBIÉN se acerca al obstáculo más rápido**. Y esa carrera —
entre "cuánto tiempo necesito para detectar" y "cuánto tiempo tengo antes de estrellarme" — es la
que de verdad decide si hay colisión o no.

Vamos con números concretos, que es la mejor forma de que esto se entienda de verdad:

### Ejemplo 1 — Volando a 0.5 m/s (lento)

Si el obstáculo aparece a **1.5 metros** del dron: a 0.5 m/s, el dron tarda `1.5 / 0.5 = 3
segundos` en llegar hasta él. Y resulta que, sumando LiDAR + mapa + el calentamiento de
`MAP_CHECK` (que a esta velocidad tarda más, pero el dron también va más lento, así que hay más
margen), el sistema logra reaccionar justo a tiempo — es la frontera que medimos: **por debajo de
~1.45-1.50 metros, choca casi siempre; por encima, esquiva casi siempre**.

### Ejemplo 2 — Volando a 1.0 m/s

Aquí el dron llega al doble de rápido a cualquier punto dado, así que necesitas más distancia de
margen para compensar. Medimos que el punto de corte sube a **~2.0-2.05 metros**. Fíjate que no se
ha duplicado (no es 3 metros, el doble de 1.5) — ha subido, pero menos que proporcionalmente, y en
la siguiente sección verás por qué.

### Ejemplo 3 — Volando a 1.5 m/s

El punto de corte sube otra vez, hasta **~2.88-2.98 metros** — de nuevo, sube, pero cada vez el
"margen de tiempo" que realmente tienes en el momento justo del corte es menor:

- A 0.5 m/s, en el punto de corte tienes casi **3 segundos** de margen.
- A 1.0 m/s, tienes solo **~2 segundos**.
- A 1.5 m/s, tienes solo **~1.9-2.0 segundos**.

**Esto es el hallazgo más importante para entender el problema de fondo: a más velocidad, no solo
hace falta más distancia para estar a salvo (algo intuitivo), sino que el margen de tiempo real
disponible en el límite se va reduciendo (menos intuitivo).** La razón es la que vimos en el
Paso 3: el "calentamiento" de 1.5 metros se convierte en menos tiempo cuanto más rápido vas, así
que ese colchón de seguridad se "encoge" justo cuando más lo necesitarías (porque también te
acercas más rápido al obstáculo).

## Cuándo es "el algoritmo falló" y cuándo es "físicamente imposible"

Con todo lo anterior ya puedes distinguir las dos situaciones:

- **Por debajo del punto de corte medido** (por ejemplo, un obstáculo a 1 metro volando a
  1.0 m/s): esto es un caso de **"físicamente imposible"**. El dron necesita ~2 segundos completos
  desde que el obstáculo aparece hasta que reacciona, y a 1 m/s solo tiene 1 segundo antes de
  llegar a él. No hay margen de mejora del software que pueda arreglar esto sin cambiar algo
  estructural (la distancia de detección, el diseño de reacción, etc.) — el obstáculo apareció
  demasiado cerca, punto.

- **Por encima del punto de corte, pero aun así choca:** aquí es donde entra el verdadero **bug**
  que encontramos (lo explico en detalle más abajo). En algunos casos, el dron tenía de sobra
  tiempo y distancia para esquivar, y aun así chocó — porque el sistema, por una razón interna
  concreta, **dejó de vigilar el camino durante ese rato**. Eso sí es "el algoritmo falló", no
  física.

## El bug: el dron a veces "deja de mirar" el camino

Este es el hallazgo que más vale la pena que le lleves a tus tutores, porque no es solo una
curiosidad — es una vulnerabilidad real.

Cuando el planificador no logra encontrar un camino directo hasta el destino final (algo que puede
pasar, por ejemplo, justo después de despegar, cuando el mapa todavía tiene zonas "desconocidas"
porque el dron aún no ha mirado hacia allí), el sistema no se rinde: en vez de ir al destino final,
elige un punto intermedio al que sí sabe cómo llegar, y vuela hacia ahí primero — a esto lo
llamamos "modo frontera".

El problema es que **mientras está en modo frontera, el vigilante (`MAP_CHECK`) deja de revisar si
aparecen obstáculos nuevos en el camino**. No es que tarde más — es que directamente esa
comprobación está apagada durante ese tramo. Así que si un obstáculo aparece justo en ese momento,
por muy lejos que esté y por muy despacio que vuele el dron, **el sistema ni se entera** hasta que
sale del modo frontera (llega al punto intermedio, o vuelve a ser posible ir directo al destino
final).

Fuimos a buscar por qué entra en ese modo tan a menudo justo tras despegar, y encontramos la causa
exacta: el código que decide si una casilla del mapa es "atravesable" para calcular rutas trata,
por defecto, **las casillas que todavía no ha visto el LiDAR como si fueran obstáculos**. Tiene
sentido como precaución ("si no sé qué hay ahí, mejor no pasar"), pero tiene un efecto secundario:
justo tras despegar, con la mayor parte del mapa todavía sin explorar, el algoritmo puede
literalmente no encontrar ningún camino libre hacia el destino — no porque haya un obstáculo real,
sino porque casi todo el mapa cuenta como "posible obstáculo" al no haberse visto aún. Y eso
dispara el modo frontera de forma completamente artificial.

Probamos el arreglo obvio: decirle al algoritmo que trate las casillas desconocidas como libres en
vez de ocupadas. Y funcionó, en el sentido de que el modo frontera espurio se redujo mucho. Pero
apareció un problema nuevo y peor: como ahora el algoritmo cree que puede pasar por zonas que en
realidad no ha visto, cada vez que recalcula la ruta encuentra un camino ligeramente distinto a
través de esas zonas "optimistas" — y eso le hace **replanificar constantemente, casi cada medio
segundo, sin converger nunca en una ruta estable**, algo que se veía clarísimo en el visor 3D
(RViz): el dron dudando de ruta de forma casi nerviosa cerca de obstáculos reales. Eso también es
peligroso, así que decidimos **revertir el arreglo** y quedarnos con el comportamiento original,
documentando bien el problema para que se pueda abordar con más calma en el futuro — probablemente
la solución correcta no sea un simple interruptor, sino separar "qué casillas usa el algoritmo
para calcular la ruta" de "qué casillas vigila el sistema para detectar obstáculos nuevos", que hoy
están acopladas.

## Una curiosidad que investigamos después: el LiDAR "viéndose los pies"

Cuando hicimos la sonda de detección pura a distancias grandes (5, 6, 7, 8 metros), nos encontramos
con algo raro: nuestra métrica de "cuánto tarda el LiDAR en ver el obstáculo" dejaba de funcionar
a partir de los 6 metros — no es que tardara mucho, es que directamente no lográbamos medirla.

Investigando el porqué, encontramos que la lectura que usábamos para esa métrica (la distancia
mínima que reporta el LiDAR hacia delante) se quedaba **fija en unos 4.2 metros incluso antes de
soltar el obstáculo**, con el obstáculo todavía aparcado fuera de la vista. Es decir, esa lectura
no tenía nada que ver con nuestra caja — era otra cosa.

La explicación, una vez la calculamos, es bastante elegante: el LiDAR no mira solo hacia adelante
en un plano horizontal — tiene un cierto campo de visión también hacia arriba y hacia abajo (unos
15 grados en cada dirección). El "rayo" más bajo de ese abanico, el que apunta 15° hacia abajo,
acaba chocando contra el propio suelo a una distancia que depende de a qué altura vuela el dron:
cuanto más alto vuela, más lejos tiene que "caer" ese rayo para tocar el suelo. Con la altura de
vuelo que usábamos (~1 metro), esa distancia sale matemáticamente en unos 4.2 metros — exactamente
lo que veíamos. El LiDAR, en cierto modo, se estaba "viendo los pies" contra el suelo, y esa
lectura tapaba cualquier obstáculo real que estuviera más lejos que esos 4.2 metros, hasta que el
dron se acercaba lo suficiente como para que el obstáculo real quedara más cerca que el suelo.

La buena noticia es que esto **no afecta a ninguna de las conclusiones sobre cuándo choca o no
choca el dron**: esa parte del sistema (el mapa de ocupación que usa el planificador) filtra este
tipo de retornos de suelo antes de decidir si una celda está ocupada, así que el planificador nunca
se confunde por esto. Solo afectaba a una métrica auxiliar que usábamos para diagnosticar, y que
solo es fiable para obstáculos más cerca de esos ~4 metros "fantasma".

## Resumen para quedarte con la idea

- Hay una cadena de pasos (LiDAR → mapa → vigilante → replan) y cada uno tarda algo, pero el
  que de verdad domina el tiempo total es el "calentamiento" del vigilante — y ese calentamiento
  es, en realidad, una distancia (1.5 metros) disfrazada de tiempo.
- Por eso volar más rápido no es "linealmente más peligroso": necesitas más distancia de margen,
  pero el tiempo de reacción disponible en el límite en realidad se reduce, no se mantiene igual.
- Los números que medimos: a 0.5 m/s hace falta que el obstáculo aparezca a más de ~1.5 m; a
  1.0 m/s, a más de ~2.0 m; a 1.5 m/s, a más de ~2.9 m. Por debajo de esas distancias, no es culpa
  del software — es que ya no había tiempo físico de reaccionar.
- Aparte de eso, existe un bug real: en ciertos momentos (sobre todo justo tras despegar) el
  sistema apaga por completo la vigilancia de nuevos obstáculos, sin que tenga que ver con la
  distancia o la velocidad — eso sí son colisiones evitables, y vale la pena arreglarlo, aunque el
  arreglo directo que probamos trae su propio problema y necesita más pensamiento.
