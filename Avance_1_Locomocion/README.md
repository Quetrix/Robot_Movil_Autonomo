# Avance 1 — Módulo de locomoción (ESP32)

Firmware del ESP32 que controla las 4 ruedas Mecanum: lectura de encoders en cuadratura, cinemática inversa, control PI + feedforward por rueda, botón de arranque/pausa, sensores IR discretos de seguridad y un comando serial de velocidad cartesiana.

## Contenido de la carpeta

| Ruta | Qué es |
|---|---|
| `locomocion_esp32_sin_IR/locomocion_esp32_sin_IR.ino` | **Versión de prueba actual.** Igual a la principal, pero sin los KY-032, con el pinout del cableado actual y con modos de movimiento continuo, temporizado y mantener tecla. Ver "Variante sin IR". |
| `locomocion_esp32_sin_IR/teleop_locomocion.py` | Programa para la PC: mover el robot manteniendo teclas (WASD + QE) o enviar comandos por consola. |
| `locomocion_esp32/locomocion_esp32.ino` | Reconstrucción del firmware que tenía el ESP32 en el Avance 1 (ver "Origen y verificación"). Usa el pinout del Avance 1, no el cableado actual. |
| `firmware_respaldo/respaldo_robot.bin` | Volcado completo (4 MB) de la flash del ESP32 usado en el Avance 1. Es la referencia exacta. |
| `historial/robot_mecanum_FINAL_completo/` | Versión anterior (antes estaba en `Codigo Arduino/`). Sin LED ni comando `V`; sensor frontal en GPIO2. |
| `historial/robot_mecanum_chatgpt_reconstruccion/` | Intento de reconstrucción hecho con ChatGPT. **No cargar**: tiene el sensor trasero en GPIO15 y el encoder FL-A en GPIO34, al revés del hardware real. |

## Origen y verificación de la versión principal

Ninguno de los dos `.ino` que había coincidía con el binario `respaldo_robot.bin`, que se compiló el 4 oct 2026 a las 21:24 con arduino-esp32 3.3.12 en placa *ESP32 Dev Module*. La versión principal usa como base la reconstrucción de ChatGPT, que es la que tiene la misma estructura que el binario, y se corrigió para que coincida con él:

- **Verificado byte a byte contra el binario:** todas las tablas de pines (`EN`, `IN1`, `IN2`, `ENC_A`, `ENC_B`), los signos, `CPR`, `KP`, `KI`, las tablas de feedforward (`RPM_FF_FWD`, `PWM_FF_FWD`, `PWM_REV`, `RPM_REV`), la tabla de cuadratura, la geometría (r = 0.024 m, L = 0.15 m), las velocidades derivadas, las constantes de la Tarea 5 (V = 0.159 m/s, 6.2 s), las rampas (12 / 35 PWM), el umbral de stall (245) y todos los mensajes seriales.
- **Verificado a nivel de código máquina:** se compiló la reconstrucción con el mismo core y se comparó función por función con el binario. 45 de 54 funciones salieron idénticas. Se encontraron y corrigieron dos diferencias: `actualizarLED()` (en el binario el LED refleja directamente los sensores) y `procesarSensores()` (en el binario no existe el bloque extra del sensor trasero).
- **Pendiente de confirmar:** `entrarPaso`, `actualizarSecuencia`, `actualizarGiro360`, `actualizarPI`, `procesarComando`, `establecerVelocidadManual` y `setup` mostraron diferencias pequeñas antes de las correcciones. Lo más probable es que sean artefactos del desensamblado o cambios en textos, pero no se volvió a compilar para confirmarlo. Antes de la entrega final, carguen la reconstrucción y corran la secuencia completa. Si algo se comporta distinto, `respaldo_robot.bin` sigue siendo la referencia.

### Restaurar el binario exacto

```bash
esptool --chip esp32 --port COMx --baud 921600 write-flash 0x0 respaldo_robot.bin
```
(Con esptool v4 el subcomando es `write_flash`.) SHA-256: `ca245fa9595277781ea13e6c637ac0d650b1c4de868e03349855072ef82b5015`.

### Compilar la reconstrucción

Arduino IDE o arduino-cli, core **esp32 by Espressif 3.3.12**, placa **ESP32 Dev Module**, opciones por defecto. No usa librerías externas. El nombre de la carpeta tiene que coincidir con el del `.ino`.

## Pinout (ESP32-WROOM-32 DevKit + expansion board)

### Motores y encoders (cableado actual, verificado rueda por rueda el 9 nov 2026)

| Rueda | Índice | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B | Signo motor | Signo encoder |
|---|---|---|---|---|---|---|---|---|
| Delantera izquierda (FL) | 0 | GPIO13 | GPIO14 | GPIO27 | GPIO15 | GPIO35 | +1 | −1 |
| Delantera derecha (FR) | 1 | GPIO23 | GPIO22 | GPIO21 | GPIO32 | GPIO33 | +1 | +1 |
| Trasera izquierda (RL) | 2 | GPIO17 | GPIO16 | GPIO4 | GPIO25 | GPIO26 | −1 | −1 |
| Trasera derecha (RR) | 3 | GPIO19 | GPIO18 | GPIO5 | GPIO36 (VP) | GPIO39 (VN) | +1 | +1 |

En el firmware del Avance 1 (`locomocion_esp32.ino` y `respaldo_robot.bin`) RL usaba 19/18/5, RR usaba 17/16/4 y el signo de motor de FL era −1. Con el cableado actual eso cruza las ruedas traseras y produce deriva; no carguen esa versión sin aplicar estos cambios.

PWM: LEDC a 5 kHz, 8 bits (0–255), en el pin Enable del driver. La dirección se fija con IN1/IN2 (`IN1=HIGH, IN2=LOW` es sentido eléctrico positivo). Con comando cero quedan EN = 0 e IN1 = IN2 = LOW. Los signos corrigen en software el sentido de cada motor y encoder, de modo que un comando positivo mueve el robot hacia adelante en las cuatro ruedas y la RPM medida sale positiva.

### Drivers (2 canales, mismo modelo adelante y atrás)

Colores del arnés de control en ambos drivers: ENA naranja, IN1 rojo, IN2 café, IN3 negro, IN4 blanco, ENB gris.

| Driver | Canal | Rueda | `EN` / `IN1` / `IN2` del código |
|---|---|---|---|
| Delantero | MOTOR A (ENA, IN1, IN2) | FL | 13 / 14 / 27 |
| Delantero | MOTOR B (ENB, IN3, IN4) | FR | 23 / 22 / 21 |
| Trasero | MOTOR A (ENA, IN1, IN2) | RR | 19 / 18 / 5 |
| Trasero | MOTOR B (ENB, IN3, IN4) | RL | 17 / 16 / 4 |

Si al reconectar un motor se invierte su sentido, corríjanlo en `MOTOR_SIGN` (o intercambiando rojo y blanco); si su RPM sale con el signo contrario, en `ENCODER_SIGN` (o intercambiando amarillo y verde).

### Conector del motor (6 hilos, de izquierda a derecha)

| Hilo | Serigrafía | Función | Conexión |
|---|---|---|---|
| Rojo | M1 | Motor | salida del driver |
| Negro | GND | GND del encoder | GND común |
| Amarillo | C1 | Encoder A | GPIO `ENC_A` |
| Verde | C2 | Encoder B | GPIO `ENC_B` |
| Azul | 3.3V | VCC del encoder | **3.3 V** (a 5 V las salidas A/B quedan a 5 V y el ESP32 no las tolera) |
| Blanco | M2 | Motor | salida del driver |

### Alimentación

VMS de ambos drivers a la batería de motores; el ESP32 se alimenta aparte (USB). El GND de la batería, los drivers, los encoders y el ESP32 deben estar unidos. Los encoders y cualquier sensor que entregue señal al ESP32 van a 3.3 V. El feedforward se calibró con los motores a 5 V (PowerBoost); con 2 × 18650 (7.4–8.4 V) las ruedas giran más rápido con el mismo PWM y el PI compensa la diferencia.

### Seguridad, interfaz y LED

| Señal | GPIO | Configuración |
|---|---|---|
| Botón arranque / pausa | GPIO12 | `INPUT_PULLUP`, botón a GND, activo en LOW, debounce de 80 ms |
| IR discreto frontal (KY-032, OUT) | GPIO0 | Solo en `locomocion_esp32`. `INPUT`, interrupción `FALLING`; LOW = obstáculo. Libre en la variante sin IR |
| IR discreto trasero (KY-032, OUT) | GPIO34 | Solo en `locomocion_esp32`. `INPUT`, interrupción `FALLING`; LOW = obstáculo. Libre en la variante sin IR |
| LED de estado (LED integrado) | GPIO2 | Salida. HIGH con ambos sensores libres, LOW si alguno detecta |
| Serial (USB) | GPIO1 / GPIO3 | UART0 a 115200 baud |

### Restricciones eléctricas del pinout

- **GPIO libres.** Con los KY-032 no queda ninguno además de UART0 (USB). En la variante sin IR quedan GPIO0 (strapping: debe estar en HIGH al arrancar) y GPIO34 (solo entrada, sin pull-up). La conexión con la Raspberry Pi va por el USB.
- Con la señal de un KY-032 desconectada en `locomocion_esp32`, GPIO34 queda flotando, se lee como obstáculo y bloquea todo movimiento.
- **Strapping pins:** GPIO0 tiene que estar en HIGH al energizar; si el sensor frontal detecta algo en ese momento, el ESP32 entra en modo descarga. GPIO12 tiene que estar en LOW al arrancar; el botón va a GND, así que no hay problema, pero no le agreguen un pull-up externo. GPIO2 y GPIO15 también son strapping pins y funcionan con el uso actual.
- GPIO34, 35, 36 y 39 son solo entrada y no tienen pull-up interno. Los encoders y el KY-032 deben entregar salida activa o tener pull-up en el módulo.

## Funcionamiento

### Lazo principal

`loop()` lee el serial, el botón y los sensores y actualiza el LED y la secuencia. Cada **20 ms** (`TS_MS`) ejecuta: medición de RPM → verificación del giro de 360° → comando de la Tarea 5 → rampa de frenado → PI + feedforward → detección de stall → fin de frenado. Cada **200 ms** imprime telemetría CSV mientras el robot está activo:

```
ms,FL_ref,FR_ref,RL_ref,RR_ref,FL_rpm,FR_rpm,RL_rpm,RR_rpm,FL_pwm,FR_pwm,RL_pwm,RR_pwm,theta_deg,sensorF,sensorA,seq,pausa
```

### Encoders (4X)

Hay interrupción `CHANGE` en ambos canales de cada encoder y se decodifica con una tabla de transición de 16 estados (cuadratura 4X), protegida con `portMUX`. La velocidad se calcula como RPM = Δcuentas · 60 / (CPR · dt) y luego se filtra con EMA (α = 0.30).

| Rueda | FL | FR | RL | RR |
|---|---|---|---|---|
| CPR (cuentas/rev en 4X, medido) | 903.50 | 939.93 | 903.21 | 905.21 |

### Cinemática

Convención interna: `vx` positivo hacia la derecha, `vy` positivo hacia adelante y `ωcw` positivo en sentido horario. Unidades: m/s y rad/s. r = 0.024 m, L = a + b = 0.08 + 0.07 = 0.15 m.

```
ω_FL = (vy + vx + L·ωcw) / r        ω_FR = (vy − vx − L·ωcw) / r
ω_RL = (vy − vx + L·ωcw) / r        ω_RR = (vy + vx − L·ωcw) / r
```
Si alguna rueda supera el límite de RPM, las cuatro se escalan en la misma proporción para conservar la dirección del movimiento. Límites: 80 RPM en los movimientos básicos y 120 RPM en el comando `V` y en la Tarea 5. El giro de 360° integra Δθ = r/(4L) · (Δφ_FL − Δφ_FR + Δφ_RL − Δφ_RR) a partir de los encoders.

### Control por rueda: PI + feedforward

`u = FF(ref) + Kp·e + I`, saturado al semiplano del signo de la referencia ([0, 255] o [−255, 0]).

| Parámetro | Valor | Nota |
|---|---|---|
| Kp / Ki | 0.50 / 0.45 | iguales en las 4 ruedas |
| Límite del término integral | ±30 PWM | anti-windup condicional |
| Retardo de integración | 300 ms | desde cada nuevo comando |
| Rampa de PWM | 12 PWM/ciclo normal, 35 PWM/ciclo al frenar | |
| Frenado | referencia → 0 a 650 RPM/s | termina con \|rpm\| < 3 en las 4 ruedas o tras 1.2 s |
| Stall | RPM promedio < 10 y PWM promedio > 245 durante 20 ciclos (0.4 s), después de 1.5 s | parada inmediata |

Feedforward hacia adelante (PWM para 80 / 100 / 120 RPM, con interpolación lineal):

| | FL | FR | RL | RR |
|---|---|---|---|---|
| 80 RPM | 216.5 | 220.2 | 217.4 | 218.3 |
| 100 RPM | 230.7 | 233.7 | 231.1 | 231.1 |
| 120 RPM | 241.4 | 245.3 | 243.4 | 242.6 |

Feedforward en reversa (RPM medidas a PWM 195 / 210 / 225 / 240 / 255):

| | 195 | 210 | 225 | 240 | 255 |
|---|---|---|---|---|---|
| FL | 70.34 | 79.30 | 104.26 | 132.82 | 158.99 |
| FR | 62.77 | 79.16 | 104.64 | 133.63 | 159.11 |
| RL | 75.79 | 82.54 | 105.85 | 134.24 | 159.60 |
| RR | 74.35 | 84.51 | 109.53 | 138.97 | 164.60 |

Por debajo del primer punto de la tabla, el feedforward se escala linealmente hacia 0 durante el frenado, el movimiento manual y la Tarea 5. En los demás casos se mantiene en el valor del primer punto.

### Secuencia de demostración (botón)

1. Primera pulsación: espera 3 s.
2. 3 s adelante, 3 s atrás, 3 s izquierda y 3 s derecha a 80 RPM (≈ 0.201 m/s). Entre pasos hay frenado controlado.
3. Giro horario de 360° a 80 RPM (ω ≈ 1.34 rad/s). Termina cuando la estimación por encoders llega a 2π.
4. Tarea 5: traslación global en línea recta con rotación simultánea, ω = 0.50 rad/s antihorario y V = ω/π ≈ 0.159 m/s durante 6.2 s (≈ 178°). El ángulo se calcula por tiempo, no por encoders. Comando: `vx = V·sin θ`, `vy = V·cos θ`.
5. Al terminar, el botón queda listo para reiniciar.

Durante la secuencia, una pulsación pausa y la siguiente reanuda. Los tiempos no corren mientras el robot está pausado o frenando.

### Sensores IR discretos

La ISR solo levanta una bandera. Se confirma el obstáculo si la señal sigue en LOW después de 20 ms, y además hay un respaldo por lectura directa por si se pierde una interrupción. Al confirmar: imprime `# OBSTACULO: FRENTE/ATRAS`, pausa la secuencia o frena el movimiento manual y activa la alarma. El sensor se considera libre después de 60 ms continuos en HIGH. Con la alarma activa y los sensores libres, una pulsación imprime `# ALARMA RECONOCIDA` y reanuda la secuencia si se había pausado por un obstáculo.

### Comandos seriales (115200 baud, terminar cada línea con `\n`)

| Comando | Acción |
|---|---|
| `W` `S` `A` `D` | adelante / atrás / izquierda / derecha a 80 RPM (solo fuera de la secuencia) |
| `R` | giro de 360° |
| `X` | parada inmediata y cancelación de la secuencia |
| `V vx vy omega` | velocidad cartesiana: `vx` derecha+ [m/s], `vy` adelante+ [m/s], `omega` **antihorario+** [rad/s], límite de 120 RPM. `V 0 0 0` frena con el PI. Se bloquea durante la secuencia, con obstáculo o mientras frena. |

Líneas de hasta 79 caracteres; si una línea es más larga se descarta con `# LINEA SERIAL MUY LARGA`.

## Variante sin IR (`locomocion_esp32_sin_IR`)

Misma base que `locomocion_esp32` (PI + feedforward, cinemática, botón, secuencia, LED, telemetría), con estos cambios:

- Sin KY-032: no hay paro por obstáculo, alarma ni bloqueo de comandos. La seguridad queda en el botón y en `X`.
- Pinout y signos del cableado actual (tabla de arriba).
- LED en GPIO2: encendido = detenido o en pausa, apagado = en marcha; dos destellos en cada pulsación del botón.
- Un comando de movimiento enviado durante el frenado lo cancela y retoma en vez de rechazarse.
- Telemetría sin las columnas de sensores: `ms,FL_ref,FR_ref,RL_ref,RR_ref,FL_rpm,FR_rpm,RL_rpm,RR_rpm,FL_pwm,FR_pwm,RL_pwm,RR_pwm,theta_deg,seq,pausa`.

### Comandos (115200 baud, cada línea termina en `\n`)

| Comando | Modo | Acción |
|---|---|---|
| `W` `S` `A` `D` | continuo | adelante / atrás / izquierda / derecha a 80 RPM hasta `X` u otro comando |
| `W 2000` (igual para S, A, D) | temporizado | lo mismo durante 2000 ms y luego frena con el PI |
| `V vx vy omega` | continuo | velocidad cartesiana: `vx` derecha+ [m/s], `vy` adelante+ [m/s], `omega` antihorario+ [rad/s], límite 120 RPM por rueda |
| `V vx vy omega ms` | temporizado | lo mismo durante `ms` y luego frena |
| `H vx vy omega` | mantener | igual a `V`, pero frena si no llega otro `H` en 300 ms. Pensado para enviar mientras una tecla está presionada |
| `V 0 0 0` | | frenado controlado con el PI |
| `R` | | giro horario de 360° medido con encoders |
| `X` | | parada inmediata y cancelación de la secuencia |
| `?` | | lista de comandos |

Desde la secuencia del botón, los comandos de movimiento quedan bloqueados hasta que termine o se cancele con `X`.

### Teleoperación desde la PC (`teleop_locomocion.py`)

```bash
pip install pyserial pynput
python teleop_locomocion.py --port COM3                 # mantener teclas
python teleop_locomocion.py --port COM3 --modo consola  # escribir comandos
```

En modo teclas, mientras haya teclas presionadas envía `H` a 10 Hz y al soltarlas envía `V 0 0 0`. W/S adelante/atrás, A/D lateral, Q/E giro antihorario/horario (se pueden combinar), `+`/`-` velocidad lineal (0.20 m/s por defecto), `]`/`[` velocidad angular (1.0 rad/s), espacio = `X`, Esc = salir. Al salir siempre envía `X`. El programa abre el puerto sin reiniciar el ESP32. pynput lee el teclado de todo el sistema y necesita sesión gráfica; por SSH usar `--modo consola`. `--csv` muestra la telemetría.

### Compilar

Arduino IDE o arduino-cli con el core **esp32 3.x** (probado con 3.3.12), placa **ESP32 Dev Module**. En PlatformIO usar la plataforma pioarduino (`platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip`), porque el código usa `ledcAttach()`.

## Pendientes para la entrega final

1. **Protocolo con navegación.** El enunciado pide recibir 3 enteros (vx, vy, θ̇) en mm/s y °/s desde la Raspberry Pi, que actúa como maestro. Hoy el firmware recibe `V` en flotantes, en m/s y rad/s, con `vx` hacia la derecha. Si se mantiene la trama acordada `X,Y,T` (X adelante, Y izquierda, Θ antihorario), la conversión es `vx = −Y/1000`, `vy = X/1000`, `ω_ccw = T·π/180`. La secuencia de demostración del botón debe pasar a un modo esclavo que siga manteniendo el botón y los IR como paro.
2. **Odometría.** Hoy los encoders solo se integran para el giro de 360°. Para la navegación hace falta la cinemática directa: `vy = r/4·(ω_FL+ω_FR+ω_RL+ω_RR)`, `vx = r/4·(ω_FL−ω_FR−ω_RL+ω_RR)`, `ωcw = r/(4L)·(ω_FL−ω_FR+ω_RL−ω_RR)`. Esa estimación se debe reportar a la Pi.
3. **Radio de rueda.** El código usa r = 24 mm (Ø 48 mm). Midan la rueda Mecanum real: si es de Ø 60 mm, todas las velocidades y la odometría tienen un error de 25 %.
4. **LED.** En el binario, `actualizarLED()` sobrescribe en cada ciclo el parpadeo y el enclavamiento, así que esas funciones no tienen efecto visible. Definan qué debe indicar el LED en la versión final.
