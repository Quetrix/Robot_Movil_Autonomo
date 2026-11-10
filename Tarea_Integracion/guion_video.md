# Guion del video — Tarea 8 (MT9005): comportamientos de Braitenberg

Duración objetivo: **4 min** (máximo 5). Unas 550 palabras de narración en total.

## Preparación (antes de grabar)

- **Dos grabaciones al mismo tiempo:**
  - el celular en trípode a ~1.5 m, en diagonal desde arriba, con toda el área de obstáculos en cuadro;
  - la pantalla de la laptop con el panel del script (OBS, o Win + Alt + R en Windows).

  En la edición, la pantalla va como recuadro en una esquina. Si no hay tiempo para editar, filma el robot con la laptop visible dentro del cuadro y la letra de la terminal grande.
- Terminal maximizada y letra grande. Correr con registro: `python3 braitenberg.py --loco ... --log video.csv`.
- Área de al menos 2 × 2 m, cajas más altas que los sensores y de 25 cm o más de ancho, y una pared o una fila de cajas.
- Haz todo el checklist en banco del README (sección 8.1) antes de la primera toma.
- Una persona maneja el teclado con el dedo en `0`; otra filma; otra narra y acomoda las cajas.
- **Prioridad si falta tiempo:** el enunciado pide al menos 2 comportamientos. Graben primero **3b** y **2a** (los que se usan en la pregunta 4) y después 3a y 2b.

## Tomas

| # | Tiempo | Imagen | Narración |
|---|---|---|---|
| 1 | 0:00–0:15 | Robot completo, de frente | "Tarea 8 de MT9005, comportamientos de Braitenberg. Grupo: [nombres]. Vamos a mostrar la arquitectura base del robot y cuatro vehículos de Braitenberg probados con obstáculos." |
| 2 | 0:15–0:45 | Acercamientos: cabezal, ESP32 de locomoción, Pi | "El robot tiene tres módulos. El ESP32 de locomoción hace la cinemática de las ruedas Mecanum y un control PI por rueda; además tiene un sensor IR frontal de seguridad. El cabezal de percepción es otro ESP32 con dos Sharp a más y menos sesenta grados y un TF-Luna al frente; publica las distancias por su propia red WiFi. La Raspberry Pi 5 lee esas distancias, corre la máquina de estados y le manda al ESP32 de locomoción la velocidad por USB." |
| 3 | 0:45–1:45 | Pantalla: `braitenberg.py` (ver "Partes del código") | Ver "Explicación del código" |
| 4 | 1:45–2:15 | Robot en el suelo + panel | **2a Miedo** (ver "Demostraciones") |
| 5 | 2:15–2:45 | | **3b Explorador** |
| 6 | 2:45–3:15 | | **3a Amor** |
| 7 | 3:15–3:45 | | **2b Agresión** |
| 8 | 3:45–4:15 | Robot explorando en 3b | Pregunta 4 |
| 9 | 4:15–4:25 | Robot detenido | "Esta máquina de estados es la base del proyecto final: ahí las transiciones van a venir de los conos, los obstáculos y el ArUco, en vez del teclado." |

## Explicación del código (toma 3, ~60 s)

Muestra cada parte en pantalla mientras la explicas.

| Parte del código | Qué mostrar | Narración |
|---|---|---|
| `class Robot`, `transicion` y `paso` | estados `ESPERA`, `MIEDO_2A`…`EXPLORADOR_3B`, `PARO` | "La arquitectura es una máquina de estados en un bucle temporizado de 100 milisegundos. Hay un estado de espera, uno por cada vehículo y un estado de paro. Por ahora las transiciones vienen del teclado, y hay una de seguridad: si en un segundo no llegan datos nuevos del cabezal, el robot pasa a PARO." |
| `estimulo()` | la fórmula | "Cada Sharp da una distancia que se normaliza a un estímulo entre cero y uno: uno a 10 centímetros o menos, cero a 60 centímetros o más." |
| `miedo_2a` … `explorador_3b` | las 4 funciones de 1 línea | "Cada vehículo conecta los dos sensores con dos ruedas virtuales, izquierda y derecha. La conexión es directa (sensor izquierdo a rueda izquierda) o cruzada, y es excitatoria (el estímulo suma velocidad) o inhibitoria (el estímulo la resta). Las cuatro combinaciones dan los vehículos 2a, 2b, 3a y 3b." |
| `diferencial_a_robot()` y `Locomocion.mover()` | `v = (vI+vD)/2`, `w = (vD−vI)/B`, `H 0 v w` | "Esas dos ruedas virtuales se convierten en una velocidad hacia adelante y una velocidad angular, como un robot diferencial. Se envían al ESP32 con el comando H, con la componente lateral en cero, y el ESP32 calcula las cuatro ruedas Mecanum." |
| capa de seguridad en `paso()` | `tf_bloqueo`, `ir_frente` | "Encima hay una capa de seguridad. El comando H se repite diez veces por segundo y el ESP32 frena solo si deja de llegar. El avance se reduce a medida que el TF-Luna se acerca a un obstáculo y se anula por debajo de 30 centímetros, o si el IR frontal detecta algo. En el 2a y el 3b, además, gira en el lugar, porque los Sharp miran a los lados y no ven lo que está justo enfrente." |

## Demostraciones (tomas 4 a 7, ~30 s cada una)

En cada toma: decir el nombre en voz alta **antes** de presionar la tecla, y que se vea `ESTADO:` en el panel.

**2a — Miedo (tecla `1`).** Caja en diagonal adelante-izquierda a ~40 cm; luego repetir por la derecha.
> "Comportamiento 2a, Miedo: conexión directa y excitatoria. El sensor del lado del obstáculo acelera la rueda de su mismo lado, así que el robot gira alejándose y acelera. Aquí la caja está a la izquierda: sube sI, la rueda izquierda va más rápido y el robot se va hacia la derecha."

**3b — Explorador (tecla `4`).** Varias cajas y una pared; dejarlo recorrer ~20 s. Incluye un momento con una caja justo enfrente.
> "Comportamiento 3b, Explorador: conexión cruzada e inhibitoria. Al acercarse a un obstáculo desacelera y gira para alejarse, y después sigue recorriendo. Cuando la caja queda justo enfrente, el TF-Luna anula el avance y el robot gira en el lugar hasta encontrar espacio libre."

**3a — Amor (tecla `3`).** Caja ancha en diagonal adelante-izquierda.
> "Comportamiento 3a, Amor: conexión directa e inhibitoria. El estímulo frena la rueda de su propio lado, así que el robot gira hacia el obstáculo, se acerca cada vez más despacio y se queda detenido frente a él."

**2b — Agresión (tecla `2`).** Caja en diagonal adelante-izquierda. Dedo en `0`.
> "Comportamiento 2b, Agresión: conexión cruzada y excitatoria. El estímulo acelera la rueda opuesta, así que el robot gira hacia el obstáculo y acelera contra él. La capa de seguridad lo frena con el TF-Luna y lo detiene a 30 centímetros; sin ella lo embestiría."

## Pregunta 4 (toma 8, ~30 s)

> "Para evitar obstáculos usaríamos el 3b, Explorador. La conexión inhibitoria hace que desacelere al acercarse, lo que le da tiempo a la percepción de reaccionar, y la conexión cruzada lo hace girar alejándose sin quedarse detenido. El 2a también evita, pero acelera justo cuando está cerca del obstáculo, que en un pasillo es arriesgado.
>
> Para explorar usaríamos también el 3b: recorre el espacio, se acerca con cuidado a lo que encuentra y sigue, en vez de quedarse frente al obstáculo como el 3a o chocarlo como el 2b. En espacios abiertos el 2a es una alternativa: nunca baja de la velocidad base y se aleja de los obstáculos, así que cubre más terreno, aunque de forma más brusca."

## Si algo sale mal durante la grabación

- Si el robot va a chocar: `0`, reacomodar y repetir la toma.
- Si un vehículo no se ve claro en el suelo (por ejemplo, el 2b o el 3a con cajas pequeñas), usa una caja más ancha o una pared. Como último recurso, grábalo con el robot levantado y la mano como estímulo, diciéndolo explícitamente, y muestra en el suelo al menos el 2a y el 3b.
- Si el panel muestra `PARO`: el cabezal perdió el enlace. Acercar el robot a la laptop o revisar el power bank del cabezal, `0` y volver a la tecla del vehículo.
