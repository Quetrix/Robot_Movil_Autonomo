# Avance 2 — Módulo de percepción (ESP32)

Firmware `Percepcion_MG996_180/` (versión `mg996-matriz-1.1`). Un ESP32 mueve un cabezal con un servo posicional MG996R y en cada pose lee 2 Sharp GP2Y0A21YK0F y 1 TF-Luna. Con esas lecturas construye una memoria angular y publica la matriz `[ángulo; distancia]` por serial y por una página web propia.

Compilar con Arduino IDE (core esp32 3.x), placa ESP32 Dev Module. Solo usa librerías del core (`WiFi`, `WebServer`, `Wire`, `Preferences`). Hay que abrir el `.ino` dentro de su carpeta, junto con todos los `.h`.

## Archivos

| Archivo | Contenido |
|---|---|
| `Percepcion_MG996_180.ino` | Máquina de adquisición, comandos, salida serial y servidor web |
| `Config.h` | **Todos los parámetros** (pines, tiempos, rangos, pesos, calibraciones) |
| `Sensors.h` | Driver I2C del TF-Luna (modo single-shot) y lectura ADC de los Sharp (mediana de 9 muestras) |
| `PositionalServo.h` | Conversión de ángulo a pulso con 3 anclas (−60°, 0°, +60°) y PWM LEDC a 50 Hz / 16 bits |
| `Calibration.h`, `CalDefaults.h` | Tablas voltaje→distancia de los Sharp: valores por defecto en código o guardados en NVS (`gp21v1`) |
| `CoreMath.h` | Interpolación, curva preliminar del Sharp, corrección del TF-Luna y transformación cabezal→robot |
| `AngularMemory.h` | Memoria angular por bins y fusión ponderada |
| `Dashboard.h` | Página HTML del panel web |

## Pinout

| Elemento | Pin del sensor | GPIO ESP32 | Notas |
|---|---|---|---|
| Sharp izquierdo (β = +60°) | Vo | GPIO34 | ADC1, 12 bits, atenuación 11 dB, sin divisor (`DIVIDER_RATIO = 1`) |
| Sharp derecho (β = −60°) | Vo | GPIO35 | ADC1, 12 bits, atenuación 11 dB, sin divisor |
| TF-Luna (β = 0°) | 2 SDA | GPIO21 | I2C a 100 kHz, dirección 0x10 |
| TF-Luna | 3 SCL | GPIO22 | |
| TF-Luna | 5 Configuración | GND | en GND el sensor entra en modo I2C (manual TF-Luna) |
| TF-Luna | 6 Multiplexing output | GPIO27 | data-ready: pasa a HIGH cuando hay una medición nueva |
| Servo MG996R posicional | señal | GPIO25 | 50 Hz; al arrancar queda sin señal |

Los Sharp tienen un filtro de capacitores en una perfboard. Usan ADC1, así que no hay conflicto con el WiFi.

## Funcionamiento

1. **Arranque sin movimiento.** El servo arranca sin señal; `CENTER` lo coloca en 0° (frente) y `SCAN` empieza el barrido.
2. **Barrido.** Va de −90° a +90° en pasos de 5° y vuelve en sentido contrario. En cada pose espera a que el servo se asiente (1000 ms en la primera, 100 ms en las demás), dispara una medición del TF-Luna (registro 0x24) y lee los Sharp después de una ventana óptica de 55 ms. Avanza a la siguiente pose cada 180 ms como mínimo, así que un barrido de 37 poses tarda ≈ 6.7 s.
3. **Cada medición** se transforma a coordenadas del robot (x, y, radio desde el eje y rumbo) con el ángulo comandado del cabezal más el ángulo de montaje β del sensor.
4. **Memoria angular.** Usa bins de 2° entre −150° y +150°; cada dato vale 12 s. Guarda por separado IR_I, IR_D, TF y el resultado fusionado (solo en RAM).
5. **Fusión.** En la dirección del TF-Luna combina las tres fuentes con pesos nominales 0.1 / 0.1 / 0.8 (IR_I / IR_D / TF). Descarta una fuente si difiere en más de 100 mm o 20 % del TF.
6. **Publicación.** Cada 500 ms imprime por serial la matriz final:

```
# MATRIZ FINAL captura=<n> t=<ms>ms vigencia=12000ms
angulo_estimado_deg;distancia_ponderada_mm
```

### Comandos (serial a 115200 o web)

`CENTER`, `SCAN`, `STOP` (mantiene el ángulo), `ANGLE <grados>` (se acerca en pasos de 5°), `RELEASE` (quita la señal al servo y borra la memoria), `CLEAR` (borra la memoria). Solo por serial: `MATRIX`, `DETAIL` (tabla por sensor con estado y pesos) y `MATRIX_ALL` (memoria completa por bin).

### Panel web

El ESP32 crea un punto de acceso WiFi: SSID `Percepcion-ESP32`, clave `percepcion26`, panel en `http://192.168.4.1`. Endpoints: `/` (panel), `/data` (JSON completo), `/matrix` (JSON de la matriz) y `/action` (POST con `cmd` y `deg`). Un `SCAN` iniciado desde la web se detiene si la página deja de consultar durante 1.5 s.

## Parámetros principales (`Config.h`)

| Parámetro | Valor | Estado |
|---|---|---|
| Rango válido Sharp / TF | 100–800 mm / 200–8000 mm | |
| Anclas del servo −60° / 0° / +60° | 1167 / 1500 / 1833 µs | **sin calibrar** (aproximación de 1000–2000 µs) |
| `SERVO_SIGN` | +1 | verificar el sentido físico |
| Origen de cada sensor (`ORIGIN_X/Y_MM`) | 0 | **sin medir** (`GEOMETRY_MEASURED = false`) |
| Tablas de calibración de los Sharp | vacías | se usa la curva preliminar (`IR_ALLOW_PRELIMINARY`); en el serial aparece marcada con `~` |
| Corrección del TF-Luna | escala 1.0068, offset −1.13 cm (entre 200 y 2100 mm) | de una calibración anterior |

## Pendientes para la entrega final

- Calibrar las anclas del servo, medir los orígenes de los sensores respecto al eje y cargar las tablas de los Sharp (mínimo 3 puntos por sensor).
- El mensaje de `ANGLE` dice "−60 a +60", pero los límites reales (`HEAD_MIN/MAX_DEG`) son ±90°. Hay que unificarlos.
- Integración: la Raspberry Pi necesita la matriz por serial o WiFi, y todavía no hay un formato de trama definido para eso (la salida serial actual está pensada para lectura humana).
