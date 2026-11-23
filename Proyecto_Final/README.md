# Proyecto final: programa de la Raspberry Pi 5

Navegación, visión, odometría, diagnóstico y grabación de datos para la prueba del 2 de diciembre. **Para correr las pruebas y calibraciones paso a paso: [`GUIA_PRUEBAS.md`](GUIA_PRUEBAS.md).** Los dos ESP32 son esclavos de la Pi por USB y no usan WiFi. El análisis completo y las decisiones están en [`../Documentos/Fase_final_analisis.md`](../Documentos/Fase_final_analisis.md).

```
                  power bank 5 V / 3 A (USB-C)
                             │
 Arducam USB ─────────▶ Raspberry Pi 5 ◀──── SSH por WiFi (laptop)
                        robot.py: visión + falda + odometría + navegación + registro
                ┌────────────┴─────────────┐
   USB 230400 ↕ V vx vy wz (20 Hz)           USB 230400 ↕ B / F / K
   L: conteos, ref, PWM, flags (50 Hz)       T: TF-Luna (100 Hz)  S: Sharp + IR trasero (25 Hz)
 ESP32 LOCOMOCIÓN (L1)                      ESP32 PERCEPCIÓN (L3)
 4 motores + encoders, PI + FF,             servo MG996R (barrido continuo), TF-Luna,
 botón de arranque/paro, KY-032 frontal     2 Sharp fijos, KY-032 trasero (opcional)
```

## Archivos

| Archivo | Qué hace |
|---|---|
| `robot.py` | Programa de la prueba: espera el botón, recorre las puertas, esquiva y se estaciona en el ArUco |
| `navegacion.py` | Máquina de estados por comportamientos (RUTA, BUSCAR, ALINEAR, CRUZAR, EVITAR, ESTACIONAR) |
| `mapa.py` | Croquis del enunciado como datos: puertas, ruta, inicio, fin, obstáculos fijos (`python3 mapa.py` lo dibuja) |
| `vision.py` | Cámara, conos (algoritmo de `Pruebas/Deteccion_Conos`) con posición en el piso y ArUco |
| `falda.py` | Matriz de ángulos y distancias con TF-Luna + Sharp, compensada con la odometría |
| `odometria.py` | Odometría Mecanum desde los conteos acumulados de los encoders |
| `enlaces.py`, `protocolo.py` | Puertos USB: identificación, hilos lectores, checksum, contadores de pérdida, reconexión |
| `registro.py` | Graba cada corrida en `registros/<fecha_hora>/` |
| `diagnostico.py` | `puertos`, `salud`, `monitor`, `teleop`, `servo`, `camara` |
| `calibrar.py` | `ff`, `pi`, `odometria`, `lag`, `sharp`, `camara`; cada una guarda su CSV |
| `exportar.py` | Pasa un registro a CSV por tipo de trama y escribe `resumen.txt` con las pérdidas; `--zip` para compartirlo |
| `sim.py` | Simulador de los dos ESP32 y la cámara para probar sin robot |
| `config.py`, `geometria_robot.py`, `config_conos.json` | Parámetros, geometría de sensores y rango de color de los conos |
| `herramientas_conos/` | Pasos 1–3 de `Deteccion_Conos` (cámara, calibración de color con ventana, detección en vivo); para la laptop |
| `pruebas/` | Pruebas sin robot: `python3 -m unittest discover pruebas` |

## Preparar la Pi (una sola vez)

Todo en la terminal de la Pi (SSH), dentro de `~/Robot_Movil_Autonomo`:

```bash
sudo apt install -y python3-opencv python3-serial python3-numpy
sudo usermod -aG dialout $USER                 # acceso a /dev/ttyUSB*; cerrar sesión y volver a entrar
echo "usb_max_current_enable=1" | sudo tee -a /boot/firmware/config.txt
sudo systemctl disable --now ModemManager 2>/dev/null   # si existe, abre los puertos y manda basura
sudo reboot
```

`usb_max_current_enable=1` es necesario: con una fuente de 5 V / 3 A la Pi 5 limita el USB a 600 mA en total, y la cámara, los dos ESP32 y los sensores que cuelgan de ellos rondan ese valor. Al pasarlo se reinician los USB y se pierden datos. Usen un cable USB-C corto y grueso hacia el power bank. Revisen con `python3 diagnostico.py salud` antes de cada prueba. Debe decir `throttled=0x0`.

OpenCV de apt (4.6 en Bookworm) sirve: `vision.py` usa la API de ArUco vieja o nueva según la versión.

## Uso

```bash
cd ~/Robot_Movil_Autonomo/Proyecto_Final
python3 diagnostico.py puertos        # debe listar un LOCO y un PERC
python3 diagnostico.py monitor --barrer   # tasas, pérdidas, RPM, TF, Sharp, falda (Ctrl+C sale)
python3 diagnostico.py teleop         # mover con el teclado; r arma, espacio para
python3 robot.py                      # la prueba: arranca con el botón
python3 robot.py --sim                # en la laptop, sin robot
```

El botón del robot (en el ESP32 de locomoción) alterna **armado / desarmado**. Armar arranca la misión. Desarmar es el paro de emergencia: los motores se cortan en el ESP32 sin depender de la Pi, y al volver a armar la misión sigue donde iba. El LED del ESP32 de locomoción está encendido mientras está armado y parpadea rápido si hubo stall.

## Protocolo serial

230400 baud, líneas ASCII terminadas en `\n`, sufijo `*HH` = XOR de todos los bytes anteriores en hexadecimal. Las tramas que salen de los ESP32 siempre lo llevan. En los comandos es opcional, para poder escribirlos a mano en el monitor serial. Si el sufijo viene, se verifica. Las líneas que empiezan con `#` son texto para humanos. Cada tipo de trama lleva un número de secuencia propio, así que la Pi cuenta las tramas perdidas.

**Pi → locomoción**

| Comando | Efecto |
|---|---|
| `V,vx,vy,wz` | velocidades enteras en mm/s, mm/s y °/s; X adelante, Y izquierda, antihorario + (convención del enunciado). Sin `V` durante 500 ms el robot frena con rampa (watchdog) |
| `X` | parada inmediata |
| `A,1` / `A,0` | armar / desarmar (para pruebas; en la prueba lo hace el botón) |
| `P,p0,p1,p2,p3` | PWM en lazo abierto −255…255 por rueda (calibración de feedforward) |
| `Q,1` / `Q,0` | IR frontal habilitado / ignorado |
| `?` | responde `I,LOCO,<versión>` |

**Locomoción → Pi**

| Trama | Campos |
|---|---|
| `L` (50 Hz) | seq, ms, c0…c3 (conteos ×4 acumulados, + = rueda hacia adelante), r0…r3 (referencia RPM), p0…p3 (PWM con signo), flags |
| `D` (1 Hz) | seq, ms, lazo máx [µs], líneas RX malas, tramas TX descartadas, watchdogs, stalls |

flags de `L`: bit0 armado, bit1 IR frontal con obstáculo, bit2 parado por watchdog, bit3 stall, bit4 frenando, bit5 lazo abierto, bit6 IR frontal deshabilitado. Ruedas: 0 FL, 1 FR, 2 RL, 3 RR.

**Pi → percepción**

| Comando | Efecto |
|---|---|
| `B,amp,vel` | barrido continuo ±amp grados a vel °/s (`B,60,150` por defecto). `amp` = 0 lo deja fijo al centro |
| `F,ang` | mantener un ángulo fijo |
| `R` | quitar la señal del servo |
| `Z,lag_ccw,lag_cw` | retardo del servo por sentido, en ms (lo mide `calibrar.py lag`) |
| `K` | keepalive. Si pasan 3 s sin ninguna línea, el cabezal vuelve al centro y se detiene |
| `?` | responde `I,PERC,<versión>` |

**Percepción → Pi**

| Trama | Campos |
|---|---|
| `T` (100 Hz) | seq, ms de la medición, ángulo estimado [centésimas de grado, + izquierda], distancia [mm], amplitud, flags (bit0 inválida, bit1 cerca del extremo del barrido, bit2 barriendo, bit3 sin ángulo) |
| `S` (25 Hz) | seq, ms, Sharp izq [mm], der [mm], izq [mV], der [mV], IR trasero (1 = obstáculo). 0 mm = nada entre 100 y 800 mm |
| `D` (1 Hz) | seq, ms, lazo máx [µs], RX malas, TX descartadas, errores I2C, lecturas TF válidas en el último segundo, modo (0 sin señal, 1 fijo, 2 barrido) |

Carga del enlace: ~5 kB/s cada uno, ~20 % de 230400 baud. Si el buffer de salida del ESP32 se llena, la trama se descarta entera y se cuenta, y el lazo de control nunca espera al USB.

## Pinout final

### ESP32 de locomoción (sin cambios respecto al cableado verificado el 9 nov)

| Rueda | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B |
|---|---|---|---|---|---|
| FL | 13 | 14 | 27 | 15 | 35 |
| FR | 23 | 22 | 21 | 32 | 33 |
| RL | 17 | 16 | 4 | 25 | 26 |
| RR | 19 | 18 | 5 | 36 (VP) | 39 (VN) |

| Señal | GPIO | Notas |
|---|---|---|
| Botón arranque/paro | 12 | a GND, `INPUT_PULLUP`. No agregar pull-up externo (pin de arranque: debe estar en LOW al encender) |
| KY-032 frontal OUT | 34 | VCC a **3.3 V**, LOW = obstáculo. Si no está conectado: `IR_FRENTE_HABILITADO = False` en `config.py` |
| LED de estado | 2 | integrado |
| USB a la Pi | 1 / 3 | UART0 |

Encoders a 3.3 V. Queda libre solo GPIO0 (pin de arranque, no usar). No conviene recablear a 9 días de la prueba: está verificado y usa 23 de los 24 GPIO disponibles. Sin WiFi, GPIO36/39 quedan a salvo del error de silicio del ESP32 que mete pulsos falsos en esos pines con el radio o el ADC1 encendidos. Por eso este ESP32 nunca debe usar WiFi ni `analogRead`.

### ESP32 de percepción

| Señal | GPIO | Notas |
|---|---|---|
| Sharp izquierdo Vo | 34 | ADC1, 11 dB, mediana de 5 lecturas |
| Sharp derecho Vo | 35 | ADC1 |
| TF-Luna SDA (pin 2) / SCL (pin 3) | 21 / 22 | I2C a 400 kHz, dirección 0x10 |
| TF-Luna pin 5 | GND | modo I2C |
| TF-Luna pin 6 | 27 | *data ready*: la medición se lee cuando sube, sin perder ni repetir tramas |
| Servo MG996R, señal | 25 | 50 Hz. Alimentación propia, **nunca** del ESP32 ni de la Pi. GND común |
| KY-032 trasero OUT (opcional) | 32 | pull-up interno: si no está conectado se lee "libre". VCC a 3.3 V |
| LED | 2 | encendido = barriendo; parpadeo = sin enlace con la Pi |

Libres para lo que haga falta: 4, 5, 13, 14, 16–19, 23, 26, 33.

### Alimentación

| Carga | Fuente |
|---|---|
| Raspberry Pi 5 | power bank 5 V / 3 A por USB-C (`usb_max_current_enable=1`) |
| Cámara, ESP32 ×2 y sus sensores (TF-Luna, Sharp, KY-032, encoders) | USB de la Pi |
| Motores | 2 × 18650 → 2 drivers |
| Servo | hoy LiPo + PowerBoost 1000C. Recomendado: buck/UBEC ≥ 3 A a 6 V desde las 2 × 18650 y 1000 µF en el conector del servo (ver `claude/perception-head-redesign-spec.md` en el proyecto). Sus picos de ~2.5 A superan al PowerBoost: esa es la causa de los cortes y saltos del servo |

Todos los GND unidos.

## Registro de datos

Cada `robot.py`, `diagnostico.py monitor/teleop` y `calibrar.py` crea `registros/AAAAMMDD_HHMMSS/` con:

- `crudo_loco.txt` y `crudo_perc.txt`: cada línea del USB, en ambos sentidos, con el tiempo de la Pi. Con esto se reconstruye todo lo demás (odometría con otro radio, otra fusión, etc.).
- `nav.csv` a 10 Hz: estado, objetivo, pose mundo y odom, comando, pasillo libre, conos, puerta y ArUco vistos, IR, armado.
- `eventos.txt`: transiciones, puertas vistas y cruzadas con la corrección de pose, avisos de los ESP32 y estadísticas de los enlaces al final.
- `fotos/`: una foto cada 2 s y al ver una puerta o el ArUco.

Una corrida de 5 min ocupa ~10 MB. Para traerlas a la laptop: `scp -r fede@robo-pi-5.local:~/Robot_Movil_Autonomo/Proyecto_Final/registros/<carpeta> .`

## Plan de pruebas (en este orden)

Cada paso tiene un criterio para darlo por bueno. Lo marcado [MEDIR]/[CALIBRAR] en `config.py`, `geometria_robot.py` y los `.ino` se llena aquí.

1. **Alimentación y enlaces.** Cargar los dos firmwares (Arduino IDE, core esp32 3.x, "ESP32 Dev Module"). Preparar la Pi. Correr `diagnostico.py monitor --barrer` con las ruedas girando por `teleop` durante 5 min. Criterio: L ≈ 50/s, T ≈ 100/s, S ≈ 25/s. `malas`, `perdidas`, `tx_perdidas` y `watchdogs` en 0, y `throttled=0x0`.
2. **Feedforward** (robot levantado): `calibrar.py ff`, pegar las tablas en `locomocion.ino`. Las tablas actuales se midieron con los motores a 5 V. Con las 2 × 18650 el driver entrega bastante más voltaje con el mismo PWM, y el integrador (±30 PWM) no alcanza a corregirlo. Criterio: en `monitor`, la RPM medida queda a ±5 % de la referencia a 40, 80 y 120 RPM.
3. **Odometría** (en el piso): `calibrar.py odometria`. `RADIO = 0.024` no coincide con una rueda de ~60 mm (r ≈ 0.030): puede ser un radio efectivo ya medido, pero hay que confirmarlo. Criterio: una recta de 2 m repetida 3 veces cae a ±3 cm.
4. **Servo y TF-Luna.** Medir con transportador las anclas −60/0/+60 (`SERVO_US_*`). Luego `calibrar.py lag`. Criterio: el canto de una caja cae en el mismo ángulo a ±2° en los dos sentidos del barrido.
5. **Sharp.** `calibrar.py sharp` y pegar las tablas. Medir el yaw real en la escala y la distancia pivote–cara (`geometria_robot.py`).
6. **Cámara.** Montarla a **15° hacia abajo** (ver `Fase_final_analisis.md`). Medir H_L4 y ponerlo en `geometria_robot.py`, luego `python3 geometria_robot.py` dice la inclinación óptima exacta. `calibrar.py camara`. Recalibrar el color **en el lugar de la prueba** con la cámara conectada a la laptop (`herramientas_conos/paso2_calibrar_color.py`) y copiar `config_conos.json` a la Pi. El rango actual tiene V mínimo 222: solo acepta naranja muy iluminado y probablemente falle en el pasillo.
7. **Tramos.** Corridas de `robot.py` con 1–2 puertas y una caja. Revisar `eventos.txt` (las correcciones de pose al cruzar deben ser de pocos cm a pocas decenas de cm) y las fotos.
8. **Croquis real.** Si dejan medir el recorrido antes de la prueba, poner las posiciones reales de las puertas en `mapa.py` (la figura no está a escala).
9. **ArUco.** Pedir al profesor diccionario, tamaño y si va en pared o piso. Poner `ARUCO_LADO_M` y `D_ESTACIONAR`.
