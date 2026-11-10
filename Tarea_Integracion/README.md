# Tarea 8 — Comportamientos de Braitenberg (locomoción + percepción + Raspberry Pi)

`braitenberg.py` corre en la Raspberry Pi 5. Lee los dos Sharp y el TF-Luna del cabezal de percepción, ejecuta una máquina de estados con 4 vehículos de Braitenberg y manda velocidades al ESP32 de locomoción. Es la arquitectura base que se reutiliza en el proyecto final.

| Módulo | Firmware | Enlace con la Pi |
|---|---|---|
| ESP32 de locomoción | [`Avance_1_Locomocion/locomocion_esp32_ir_frontal`](../Avance_1_Locomocion/locomocion_esp32_ir_frontal/) (variante sin IR + KY-032 frontal en GPIO34) | USB, 115200 baud |
| ESP32 del cabezal | [`Avance_2_Percepcion/Percepcion_MG996_180`](../Avance_2_Percepcion/Percepcion_MG996_180/) (sin cambios, fijo en `CENTER`) | WiFi propio `Percepcion-ESP32` → `http://192.168.4.1/data`. Respaldo: USB con `DETAIL` |

**Resumen si ya está todo armado y configurado:** robot levantado → encender cabezal → encender Pi → laptop a la red `Percepcion-ESP32` → `ssh fede@192.168.4.200` → `cd ~/Robot_Movil_Autonomo/Tarea_Integracion && python3 braitenberg.py --loco /dev/serial/by-id/<locomocion>` → probar en banco → bajar al suelo → teclas `1`–`4`, `0`/espacio para esperar, `q` para salir.

## Contenido

| Archivo | Qué es |
|---|---|
| `braitenberg.py` | Programa de la Pi: lectura, máquina de estados, 4 vehículos, capa de seguridad y panel en la terminal |
| `simulador_sin_robot.py` | Imita al ESP32 de locomoción (puerto serial virtual), al cabezal (HTTP y serial) y un mundo 2D con obstáculos. Sirve para ensayar sin el robot |
| `prueba_simulada.py` | Corre `braitenberg.py` contra el simulador en 14 escenarios y revisa el resultado (~40 s) |
| `guion_video.md` | Guion del video de entrega |

---

## 1. Cómo funciona

```
 ESP32 cabezal ──WiFi GET /data (5 Hz)──▶ Raspberry Pi 5 ──USB "H 0 v w" (10 Hz)──▶ ESP32 locomoción
 Sharp izq (+60°), Sharp der (−60°),        braitenberg.py                            cinemática Mecanum,
 TF-Luna (0°); cabezal fijo en CENTER                                                 PI por rueda, KY-032
```

Cada 100 ms (`Robot.paso()`):

1. **Lectura.** Distancia de cada sensor en mm (`samples[i].mm` del JSON). Sin lectura válida = `None`.
2. **Estímulo normalizado.** `s = clip((600 − d)/(600 − 100), 0, 1)`: 1 = muy cerca, 0 = lejos o sin lectura. Si el Sharp está fuera de rango con voltaje alto (≥ 2.3 V, más cerca de 100 mm) se toma `s = 1`.
3. **Máquina de estados.** Decide qué vehículo está activo.
4. **Vehículo.** Calcula las velocidades de dos ruedas virtuales, `vI` y `vD`.
5. **Diferencial virtual.** `v = (vI + vD)/2`, `ω = (vD − vI)/B` con B = 0.20 m. Si la rueda derecha va más rápido, el robot gira a la izquierda (antihorario), así que `ω > 0`, que es la misma convención que `omega` del firmware.
6. **Capa de seguridad** (tiene prioridad sobre el vehículo).
7. **Comando** `H 0 v ω` al ESP32: `vx = 0`, `vy = v` (adelante), `omega = ω` (antihorario +). El firmware hace la cinemática inversa Mecanum y el PI de cada rueda.

### Máquina de estados

| Estado | Qué hace | Se entra con |
|---|---|---|
| `ESPERA` | frena (`V 0 0 0`) y no envía nada más | inicio, tecla `0` o espacio, percepción recuperada |
| `MIEDO_2A` / `AGRESION_2B` / `AMOR_3A` / `EXPLORADOR_3B` | ejecuta el vehículo | teclas `1` / `2` / `3` / `4` |
| `PARO` | frena | 1 s sin capturas nuevas del cabezal estando en un vehículo |

`q` sale enviando `X` (parada inmediata). En el proyecto final, las transiciones por teclado se cambian por eventos (cono detectado, obstáculo, ArUco).

### Vehículos (V0 = 0.20 m/s ≈ 80 RPM, K = 0.15 m/s)

| Vehículo | Conexión | Ruedas virtuales | Resultado esperado |
|---|---|---|---|
| 2a Miedo | directa, excitatoria | vI = V0 + K·sI, vD = V0 + K·sD | se aleja del obstáculo y acelera |
| 2b Agresión | cruzada, excitatoria | vI = V0 + K·sD, vD = V0 + K·sI | gira hacia el obstáculo y acelera (lo frena el TF a 25 cm) |
| 3a Amor | directa, inhibitoria | vI = V0·(1 − sI), vD = V0·(1 − sD) | gira hacia el obstáculo, desacelera y se detiene frente a él |
| 3b Explorador | cruzada, inhibitoria | vI = V0·(1 − sD), vD = V0·(1 − sI) | desacelera al acercarse y gira para alejarse; sigue recorriendo |

Si las dos ruedas virtuales quedan por debajo de 0.04 m/s, se frena.

### Capa de seguridad

| Regla | Dónde | Efecto |
|---|---|---|
| `H` a 10 Hz en vez de `V` | Pi + firmware | si el script, la Pi, el SSH o el USB fallan, el ESP32 frena solo a los 300 ms |
| Sin capturas nuevas del cabezal durante 1 s | Pi | estado `PARO` |
| TF-Luna < 250 mm (o "menos de 20 cm") | Pi | se anula el avance (`v ≤ 0`); se libera por encima de 350 mm. En **2a y 3b**, además, gira en el lugar a 1 rad/s hacia el lado más libre, porque los Sharp miran a ±60° y no ven lo que está justo enfrente |
| KY-032 frontal detecta | firmware | anula `vy > 0` por su cuenta (retroceder, lateral y giro siguen permitidos); la Pi lo muestra y lo trata igual que el TF |

### Pregunta 4

- **Para evitar obstáculos: 3b Explorador.** La conexión inhibitoria reduce la velocidad a medida que el obstáculo se acerca, y la conexión cruzada lo hace girar alejándose. Llega despacio, con tiempo para que la percepción reaccione, y se aparta sin detenerse. El 2a también se aparta, pero acelera justo cuando está cerca, que es lo contrario de lo que conviene en un pasillo.
- **Para explorar: 3b** (o 2a en espacios abiertos). El 3b recorre el espacio, se acerca con cuidado a lo que encuentra y sigue: no se queda detenido como el 3a ni choca como el 2b. El 2a cubre más terreno porque nunca baja de V0 y huye de los obstáculos, pero se mantiene lejos de ellos y es más brusco.

---

## 2. Armado físico

Partes sobre el chasís de 120 × 180 mm: 4 motores con rueda Mecanum, 2 drivers, ESP32 de locomoción sobre su expansion board, portapilas 2 × 18650, KY-032 frontal, cabezal de percepción (servo MG996R + 2 Sharp + TF-Luna + su ESP32 y su power bank), Raspberry Pi 5 y su power bank.

1. **Ruedas y motores.** Ya están verificados: no desmontes las ruedas. Si alguna se suelta, va en la misma esquina (cada Mecanum es izquierda o derecha); después verifica con `A` y `D` que el robot se desplace de lado sin girar.
2. **Peso abajo y al centro.** Portapilas y power banks lo más bajo y centrado posible, sujetos con cinchos o velcro. Que nada toque las ruedas.
3. **Drivers y ESP32 de locomoción.** Fijos al chasís; cables de motor y encoder recogidos lejos de las ruedas.
4. **KY-032 frontal.** Centrado en el frente, mirando hacia adelante, entre unos 3 y 6 cm del suelo para que vea cajas de 10 cm sin ver el piso. Si ve el piso (LED del módulo encendido sin obstáculo), inclínalo un poco hacia arriba.
5. **Cabezal.** Eje del servo vertical. Con `CENTER` el TF-Luna debe apuntar exactamente hacia adelante y los Sharp a ±60°. Las anclas del servo no están calibradas: si en `CENTER` el cabezal queda torcido, corrígelo mecánicamente (girando el brazo del servo) antes de seguir.
6. **Campo de visión libre.** Ni la Pi, ni los power banks, ni los cables pueden quedar delante de los Sharp o del TF. Con el robot en un espacio vacío, el panel debe mostrar `sI = 0.00` y `sD = 0.00`.
7. **Raspberry Pi.** Fija, con el cable USB al ESP32 sujeto con un cincho cerca de ambos conectores para que no se desconecte con la vibración.
8. **Altura de los sensores.** Mide a qué altura quedan los rayos de los Sharp y del TF: los obstáculos de prueba deben ser más altos que eso.

## 3. Conexiones eléctricas

Todo con la batería de motores **desconectada**.

### 3.1 Alimentación

| Fuente | Alimenta | Conexión |
|---|---|---|
| 2 × 18650 en serie (7.4–8.4 V) | motores | + → **VMS** de ambos drivers; − → **GND** de ambos drivers |
| USB de la Pi | ESP32 de locomoción, encoders y KY-032 | micro-USB del DevKit → puerto USB-A de la Pi. Encoders y KY-032 a **3.3 V** desde el ESP32 |
| Power bank 5 V / 3 A | Raspberry Pi 5 | USB-C de la Pi |
| Power bank propio | cabezal completo | como quedó en el Avance 2; no se conecta a nada más |

- **GND común:** − de la batería ↔ GND de los dos drivers ↔ GND del ESP32 de locomoción. El cabezal es inalámbrico y no necesita GND común.
- No alimentes la expansion board por su jack DC ni por su USB-C mientras el DevKit está conectado a la Pi: el ESP32 de locomoción se alimenta solo por el USB de la Pi.
- La expansion board tiene un jumper que selecciona 5 V o 3.3 V para los pines VCC de los rieles. Mide con el multímetro el VCC al que van los encoders y el KY-032: debe dar **3.3 V**.
- Con una fuente de 3 A la Pi 5 limita el USB a unos 600 mA en total; el ESP32 de locomoción con los encoders y el KY-032 está bastante por debajo de eso.
- Si el portapilas tiene interruptor, úsalo como corte físico de los motores. Si no, desconectar un borne de la batería es el paro físico.

### 3.2 ESP32 de locomoción (pinout verificado el 9 nov 2026)

**Motores, drivers y encoders**

| Rueda | Driver / canal | EN (PWM) | IN1 | IN2 | Encoder A (amarillo) | Encoder B (verde) |
|---|---|---|---|---|---|---|
| Delantera izq. (FL) | delantero, MOTOR A (ENA/IN1/IN2) | GPIO13 | GPIO14 | GPIO27 | GPIO15 | GPIO35 |
| Delantera der. (FR) | delantero, MOTOR B (ENB/IN3/IN4) | GPIO23 | GPIO22 | GPIO21 | GPIO32 | GPIO33 |
| Trasera izq. (RL) | trasero, MOTOR B (ENB/IN3/IN4) | GPIO17 | GPIO16 | GPIO4 | GPIO25 | GPIO26 |
| Trasera der. (RR) | trasero, MOTOR A (ENA/IN1/IN2) | GPIO19 | GPIO18 | GPIO5 | GPIO36 (VP) | GPIO39 (VN) |

Colores del arnés de control en ambos drivers: ENA naranja, IN1 rojo, IN2 café, IN3 negro, IN4 blanco, ENB gris.

Conector de cada motor (6 hilos): rojo M1 y blanco M2 → salida del driver; negro → GND común; amarillo C1 → encoder A; verde C2 → encoder B; **azul → 3.3 V** (a 5 V las salidas del encoder dañan al ESP32).

**Resto de señales**

| Señal | Pin del módulo | GPIO / conexión |
|---|---|---|
| KY-032 frontal | `GND` | GND |
| | `+` | **3.3 V** |
| | `OUT` | **GPIO34** (LOW = obstáculo) |
| | `EN` | sin conectar; el jumper EN del módulo debe estar puesto |
| Botón | — | GPIO12 a GND. **No se usa en esta tarea** (arranca la secuencia de demostración y bloquea los comandos) |
| LED integrado | — | GPIO2: encendido = detenido, apagado = en marcha |
| USB | — | a la Raspberry Pi |

GPIO0 queda libre, pero es pin de arranque: no conectarle nada.

### 3.3 Cabezal de percepción (referencia, ya armado en el Avance 2)

| Elemento | Pin del sensor | GPIO |
|---|---|---|
| Sharp izquierdo (+60°) | Vo | GPIO34 |
| Sharp derecho (−60°) | Vo | GPIO35 |
| TF-Luna (0°) | 2 SDA / 3 SCL | GPIO21 / GPIO22 |
| TF-Luna | 5 configuración | GND (modo I2C) |
| TF-Luna | 6 data-ready | GPIO27 |
| Servo MG996R | señal | GPIO25 |

### 3.4 Revisión antes de energizar

1. Polaridad de la batería en VMS/GND de los drivers, medida con el multímetro (7.4–8.4 V).
2. Continuidad del GND común entre batería, drivers y ESP32.
3. Azul de los 4 encoders y `+` del KY-032 a 3.3 V, no a 5 V.
4. Ningún cable cerca de las ruedas ni del servo.

## 4. Flashear el ESP32 de locomoción (desde la laptop)

1. Arduino IDE 2 con el core **esp32 by Espressif 3.x** (probado con 3.3.12). Placa **ESP32 Dev Module**, opciones por defecto.
2. Abrir `Avance_1_Locomocion/locomocion_esp32_ir_frontal/locomocion_esp32_ir_frontal.ino`.
3. **Batería de motores desconectada.** ESP32 por USB a la laptop → elegir el puerto → Subir. Si se queda en `Connecting...`, mantener presionado BOOT hasta que empiece a escribir.
4. Monitor serie a 115200, fin de línea "Nueva línea". Al reiniciar debe aparecer `ROBOT MECANUM - LOCOMOCION IR FRONTAL` y `# IR FRENTE: LIBRE`.
5. Pasar la mano frente al KY-032: `# IR FRENTE: OBSTACULO` y, al retirarla, `# IR FRENTE: LIBRE`. Si no cambia, ajusta el potenciómetro de distancia del módulo hasta que detecte a unos 15 cm.
6. Escribir `?`: la ayuda debe incluir la línea del IR frontal.

**Cabezal:** ya tiene `Percepcion_MG996_180` del Avance 2. Para verificarlo, conecta la laptop a `Percepcion-ESP32` (clave `percepcion26`), abre `http://192.168.4.1`, presiona CENTER y mueve la mano frente a cada sensor. Cierra esa página antes de correr el script (un `SCAN` iniciado desde la web movería el cabezal). Si hiciera falta reflashearlo: misma placa, abrir el `.ino` dentro de su carpeta con todos los `.h`.

## 5. Configurar la Raspberry Pi (una vez, con internet)

Desde la laptop, con la Pi en su red normal:

```bash
ssh fede@robo-pi-5.local

# Paquetes (sin pip global)
sudo apt update && sudo apt install -y python3-serial git

# Permiso para los puertos serie: si "dialout" no aparece en la lista
groups
sudo usermod -aG dialout fede        # luego salir (exit) y volver a entrar por SSH

# Código
git clone https://github.com/Quetrix/Robot_Movil_Autonomo.git   # o: cd Robot_Movil_Autonomo && git pull
cd ~/Robot_Movil_Autonomo/Tarea_Integracion
python3 prueba_simulada.py           # ~40 s, sin robot: debe terminar en "TODO OK"
```

Si el repo es privado y `git clone` pide credenciales, copia la carpeta desde la laptop: `scp -r Robot_Movil_Autonomo fede@robo-pi-5.local:~/`.

**ESP32 de locomoción a la Pi.** Conecta el USB y anota su nombre:

```bash
ls -l /dev/serial/by-id/
# ejemplo: usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

**Perfil WiFi del cabezal con IP fija.** Así la Pi siempre queda en 192.168.4.200 dentro de esa red y no hay que buscar su IP:

```bash
sudo nmcli con add type wifi ifname wlan0 con-name cabezal ssid "Percepcion-ESP32" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "percepcion26" \
  ipv4.method manual ipv4.addresses 192.168.4.200/24 ipv4.never-default yes \
  ipv6.method disabled 802-11-wireless.powersave 2 \
  connection.autoconnect yes connection.autoconnect-priority 10
nmcli con show                       # anota el nombre de tu red normal
```

Crear el perfil no corta la conexión. Por la prioridad, si el cabezal ya está encendido cuando arranca la Pi, la Pi se conecta a él sola. Con el cabezal apagado vuelve a su red normal. `powersave 2` desactiva el ahorro de energía del WiFi (menos latencia).

**Opcional, para el respaldo C (sección 7):** guarda también el hotspot de la laptop: `sudo nmcli dev wifi connect "<hotspot>" password "<clave>"`.

## 6. Encender y conectarse en el lugar de la prueba

1. **Robot levantado** (sobre una caja, con las ruedas en el aire).
2. Encender el **cabezal** (su power bank) y esperar unos 5 s a que aparezca la red `Percepcion-ESP32`.
3. Pasar la Pi a la red del cabezal:
   - Si la Pi está apagada: enciéndela y espera unos 40 s; se conecta sola por la prioridad.
   - Si ya está encendida y conectada por SSH en la red normal (dos líneas separadas; la primera pide la clave de sudo si hace falta):
     ```bash
     sudo -v
     sudo nohup nmcli con up cabezal >/dev/null 2>&1 &
     ```
     El SSH se corta: es lo esperado. Desde ahí la Pi no tiene internet ni Pi Connect.
4. Conectar la **laptop** a `Percepcion-ESP32`. Windows dirá "sin internet": es normal. Si Windows se cambia solo a otra red, desactiva "conectar automáticamente" en esa otra.
5. Probar el enlace y entrar:
   ```bash
   ping 192.168.4.200
   ssh fede@192.168.4.200
   curl -s http://192.168.4.1/data | head -c 200     # en la Pi: debe salir JSON con "samples"
   ```
6. Conectar la **batería de motores**.

Para volver a la red normal: apagar el cabezal (la Pi regresa sola en un minuto aproximadamente) o, desde la Pi, `sudo nmcli con up "<tu red normal>"`.

## 7. Correr el script

```bash
cd ~/Robot_Movil_Autonomo/Tarea_Integracion
ls /dev/serial/by-id/
python3 braitenberg.py --loco /dev/serial/by-id/<locomocion> --log corrida1.csv
```

Con un solo ESP32 conectado, también sirve `--loco "$(ls /dev/serial/by-id/* | head -1)"`.

| Tecla | Acción |
|---|---|
| `1` | 2a Miedo |
| `2` | 2b Agresión |
| `3` | 3a Amor |
| `4` | 3b Explorador |
| `0` o espacio | ESPERA (frena) |
| `q` | salir (envía `X`) |
| Ctrl+C | igual que `q` |

Opciones: `--log archivo.csv` guarda un registro a 10 Hz (t, estado, distancias, sI, sD, vI, vD, v, ω, irF, bloqueo, comando), útil para el informe. `--invertir-sharp` intercambia izquierdo y derecho. `--sin-pantalla` imprime líneas en vez del panel.

El panel muestra estado, distancia y estímulo de cada sensor, TF e IR, `vI`, `vD`, `v`, `ω`, el comando enviado, la regla de seguridad activa y los últimos eventos (incluidos los mensajes `[loco] # IR FRENTE: ...`). Maximiza la ventana (mínimo 80 × 26) y agranda la letra para el video.

**No uses `tmux` ni `nohup` para el script.** Si se cae el SSH, el script debe morir para que el robot frene solo.

### Respaldos si el enlace por la red del cabezal no funciona

No está verificado que el softAP del ESP32 reenvíe tráfico entre dos clientes (laptop → Pi). Si `ping 192.168.4.200` falla desde la laptop aunque ambas estén en `Percepcion-ESP32`, usa uno de estos:

- **B. Cabezal por USB a la Pi.** La Pi se queda en su red normal y entras con `ssh fede@robo-pi-5.local`. Conecta el USB del ESP32 del cabezal a la Pi y corre `python3 braitenberg.py --loco /dev/serial/by-id/<locomocion> --perc /dev/serial/by-id/<cabezal>`. Para saber cuál es cuál, conéctalos de uno en uno y mira `ls /dev/serial/by-id/`; si los dos tienen el mismo nombre, usa `/dev/serial/by-path/`. El USB de la Pi es solo para datos: el servo y los sensores siguen con el power bank del cabezal. Si ese power bank entra por el mismo USB del DevKit, este respaldo no aplica tal cual.
- **C. Hotspot de la laptop.** Si la red de la universidad no permite conexiones entre equipos, activa el hotspot de Windows, conecta la Pi a él (perfil guardado en la sección 5) y usa el respaldo B para el cabezal.

## 8. Checklist de pruebas

### 8.1 En banco (robot levantado, ruedas en el aire)

| # | Prueba | Resultado esperado |
|---|---|---|
| 1 | Arrancar el script | panel con `firmware loco: ir_frontal`, `percepcion: OK`, `cabezal: 0.0 deg hold` |
| 2 | Espacio vacío alrededor | `sI = 0.00`, `sD = 0.00` (si no, algo del robot está en el campo de visión) |
| 3 | Mano a ~30 cm a la **izquierda** a 60° | sube `sI` (no `sD`). Si sube `sD`, usa `--invertir-sharp` |
| 4 | Mano a < 25 cm frente al TF | `BLOQUEO (< 250 mm)`; se libera al alejarla a más de 35 cm |
| 5 | Mano frente al KY-032 | `IR frente: OBSTACULO` y evento `[loco] # IR FRENTE: OBSTACULO` |
| 6 | Tecla `4` sin obstáculos | las 4 ruedas giran hacia adelante a la misma velocidad; `H 0 0.200 0.000` |
| 7 | Con `4`, mano a la izquierda | `w` negativo: las ruedas izquierdas giran más rápido que las derechas (gira a la derecha, se aleja) |
| 8 | Con `4`, mano frente al TF | `avance anulado + giro`: ruedas izquierdas y derechas en sentidos opuestos |
| 9 | Con `1`, mano a la izquierda | igual que 7 pero acelerando (`v` > 0.20) |
| 10 | Con `3`, mano a la izquierda | las ruedas izquierdas se frenan (gira hacia la mano) |
| 11 | `0` | frenado controlado; `q` envía `X` |
| 12 | En `4`, desconectar el USB de la Pi | las ruedas se detienen en menos de 0.3 s (timeout de `H`). Reconectar y reiniciar el script |
| 13 | En `4`, apagar el cabezal | a 1 s pasa a `PARO` y frena |

### 8.2 En el suelo con obstáculos

Espacio plano de al menos 2 × 2 m. Cajas de cartón **más altas que los rayos de los sensores** y, de preferencia, de 25 cm o más de ancho. Siempre con alguien con el dedo en `0`.

| Vehículo | Montaje | Qué debe verse |
|---|---|---|
| 2a Miedo (`1`) | caja en diagonal adelante-izquierda, a ~40 cm | gira a la derecha alejándose y acelera. Repetir por la derecha |
| 2b Agresión (`2`) | caja en diagonal adelante-izquierda | gira hacia la caja y acelera; el TF la detiene a ~25 cm. Si va a chocar, `0` |
| 3a Amor (`3`) | caja ancha en diagonal | gira hacia la caja, desacelera y se detiene frente a ella |
| 3b Explorador (`4`) | 2–3 cajas y una pared | desacelera al acercarse y se aparta; con una caja justo enfrente se detiene, gira en el lugar y sigue |

Si el robot va muy rápido para el espacio, baja `V0` en `braitenberg.py` (p. ej. 0.15). Si las ruedas no arrancan con estímulos intermedios, súbelo a 0.25. Si gira poco, sube `K_EXC` o baja `B_VIRTUAL`.

## 9. Si algo falla

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `Permission denied: /dev/ttyUSB0` | usuario fuera de `dialout` | `sudo usermod -aG dialout fede`, salir y volver a entrar |
| No aparece nada en `/dev/serial/by-id/` | cable solo de carga o mal conectado | otro cable; `dmesg \| tail` al conectar. Si el puerto aparece y desaparece: `sudo apt remove brltty` |
| `firmware loco: SIN IR FRONTAL` | el ESP32 tiene otra versión | flashear `locomocion_esp32_ir_frontal` (sección 4) |
| `percepcion: SIN DATOS` → `PARO` | la Pi no está en la red del cabezal o el cabezal está apagado | `nmcli -t -f ACTIVE,SSID dev wifi`; `curl http://192.168.4.1/data`; power bank del cabezal; acercar el robot |
| `cabezal: servo sin senal` | el cabezal se reinició | el script reenvía `CENTER` cada 3 s; revisar el power bank del cabezal |
| `ssh` no conecta por `Percepcion-ESP32` | el softAP no reenvía entre clientes, o IP distinta | `ping 192.168.4.200`; respaldo B o C (sección 7) |
| Gira al revés en 2a/3b (va hacia el obstáculo) | Sharp izquierdo y derecho intercambiados | prueba 3 del banco; `--invertir-sharp` |
| `sI` o `sD` no bajan de cero con espacio libre | una parte del robot o el piso está en el campo de visión | despejar, o subir/inclinar el cabezal |
| Ruedas no se mueven y aparece `# V BLOQUEADO: SECUENCIA ACTIVA` | se presionó el botón físico | salir con `q` (envía `X`) y volver a correr |
| Eventos `[loco] # H SOLTADO` durante un vehículo | pasaron más de 300 ms sin `H` (Pi saturada o USB) | cerrar otros programas; revisar el cable USB |
| `# ERROR: CAIDA GENERAL / STALL` | ruedas bloqueadas o batería baja | revisar ruedas; cargar o cambiar las 18650 |
| `IR frente: OBSTACULO` todo el tiempo | KY-032 ve el piso, sol directo o el jumper EN no está puesto | potenciómetro, inclinación, jumper; revisar `OUT` en GPIO34 |
| TF siempre `--` | nada dentro de 8 m, superficie oscura o en ángulo, o el cabezal no mira al frente | revisar con la mano a 50 cm; `CENTER` |
| El robot choca cajas pequeñas que quedan en diagonal | zona ciega entre 0° y ±60° (ver Limitaciones) | usar cajas más anchas o paredes para la demostración |

Paro de emergencia en cualquier momento: `0` o `q`. Si el teclado no responde, desconecta la batería de motores.

## 10. Ensayo sin robot

En la laptop (Linux/macOS o WSL) o en la Pi:

```bash
python3 simulador_sin_robot.py --escenario arena      # imprime LOCO /dev/pts/N y la URL del cabezal
python3 braitenberg.py --loco /dev/pts/N --perc http://127.0.0.1:8080
python3 prueba_simulada.py                             # prueba automática, ~40 s
```

Escenarios: `izq`, `der`, `frente`, `caja_izq`, `arena`. El simulador traza los rayos de los sensores desde el centro del robot, imita el timeout de `H` y el recorte del KY-032, y con `--traza archivo.csv` guarda la trayectoria. `prueba_simulada.py` revisa el sentido de giro de los 4 vehículos, el bloqueo frontal con giro, el modo serial del cabezal, el paso a `PARO`, el frenado si se mata el script, `--invertir-sharp` y la detección de un firmware sin IR.

## 11. Limitaciones conocidas

- **Zonas ciegas.** Con el cabezal fijo, los rayos son tres líneas angostas (+60°, 0°, −60°). Un objeto pequeño que se acerca entre 0° y ±60° no se ve. En la simulación, el 3b recorrió 30 s una arena con dos cajas de 16 cm sin chocar en 4 de 5 corridas; en la otra rozó una caja que se acercó en diagonal (a −44°). El 2b y el 3a pueden tocar cajas pequeñas porque van hacia ellas. Para el proyecto final se necesita el barrido del cabezal (`SCAN`).
- La curva del Sharp es preliminar (sin calibrar) y las anclas del servo tampoco están calibradas: las distancias son aproximadas.
- El TF-Luna no mide por debajo de 200 mm; ese caso se toma como bloqueo.
- El feedforward se calibró con los motores a 5 V y ahora van a 7.4–8.4 V: el PI compensa, pero conviene revisar la velocidad real en el suelo.
- El firmware `ir_frontal` se probó en la PC con un stub de Arduino; falta compilarlo con el core esp32 en el Arduino IDE (sección 4) y probarlo en el robot.
