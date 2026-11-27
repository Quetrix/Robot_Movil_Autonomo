# Guía de pruebas y calibración (fase final)

Cada paso dice **dónde** se escribe el comando, **qué debes ver** y **qué archivo me mandas**. Seguir en orden: cada calibración se apoya en la anterior.

Dónde se ejecuta cada comando:

| Marca | Dónde | Cómo se ve el prompt |
|---|---|---|
| **[L]** | PowerShell en la laptop | `PS C:\Users\esteb>` |
| **[P]** | la Raspberry Pi, por SSH desde la laptop | `fede@robo-pi-5:~/Robot_Movil_Autonomo/Proyecto_Final $` |
| **[IDE]** | Arduino IDE en la laptop | — |

## 0. Cómo se conecta todo

**No hace falta un firmware aparte para medir.** El firmware final de locomoción ya tiene el comando `P` (PWM en lazo abierto) y manda los conteos de los encoders a 50 Hz. El firmware de percepción manda todas las lecturas crudas. Las mediciones son scripts de Python que hablan con esos firmwares y guardan CSV.

Esos scripts corren igual en dos montajes:

- **Montaje normal (recomendado): los ESP32 en los USB de la Pi.** La laptop entra a la Pi por WiFi (SSH) y no tiene cables al robot. Es el mismo montaje de la prueba, así que también verifica la alimentación y los USB reales.
- **Montaje de banco: un ESP32 conectado directo a la laptop.** Es obligatorio para **cargar el firmware**. Opcionalmente sirve para calibrar en la laptop, por ejemplo el feedforward con el robot sobre la mesa. Se agrega `--loco COM5` o `--perc COM6` **antes** del subcomando, por ejemplo `python calibrar.py --loco COM5 ff`. El número de COM aparece en Arduino IDE → Herramientas → Puerto.

Reglas:

1. Un puerto serial solo lo puede abrir un programa a la vez. Cierren el **Monitor Serie** del Arduino IDE antes de correr un script, y no corran dos scripts que usen el mismo ESP32.
2. Un ESP32 va conectado a la Pi **o** a la laptop, no a las dos.
3. **Los motores necesitan su batería (2 × 18650) aunque el ESP32 esté en la laptop.** El USB solo alimenta la lógica.

### Red de la Pi

Cómo conectarse a la Pi, cómo agregar una red WiFi propia y cómo funciona el hotspot de respaldo `Robot-MT9005`: ver [`ESTADO.md`](../ESTADO.md#red-de-la-pi-y-ssh), sección "Red de la Pi y SSH". Las claves no están en el repo (es público); las pasa el equipo por privado.

## 1. Preparación (una vez)

### 1.1 Código en la laptop y en la Pi

[L], en la carpeta del repo:
```powershell
cd C:\Users\esteb\Documents\Projects\Robot_Movil_Autonomo
git fetch
git checkout main
git pull
pip install pyserial opencv-contrib-python numpy
```

[L] `ssh fede@robo-pi-5.local` (o `fede@192.168.50.1` en el hotspot). Luego [P]:
```bash
cd ~/Robot_Movil_Autonomo
git fetch && git checkout main && git pull
cd Proyecto_Final
sudo apt install -y python3-opencv python3-serial python3-numpy
python3 -m unittest discover pruebas
```
**Debes ver:** `Ran 12 tests ... OK`.

### 1.2 Cargar los firmwares [IDE]

1. Conecta el ESP32 de **locomoción** a la laptop por USB.
2. Abre `Avance_1_Locomocion/locomocion/locomocion.ino`. Elige la placa "ESP32 Dev Module" (core esp32 3.x) y su puerto, y súbelo.
3. Abre el Monitor Serie a **115200** baud.
   - **Debes ver:** cada 20 ms una línea `L,123,4567,0,0,0,0,0,0,0,0,0,0,0,0,0*3A`.
   - Escribe `?` y Enter. **Debes ver:** `I,LOCO,loco-final-1.1*..` (versión vigente: ver `ESTADO.md`).
4. Cierra el Monitor Serie. Repite con `Avance_2_Percepcion/percepcion/percepcion.ino`.
   - **Debes ver:** líneas `T,...` (100 por segundo) y `S,...`, y al arrancar `# TF-Luna listo (continuo 100 Hz)`.
   - Si dice `TF-Luna SIN DATOS`: revisa SDA (21), SCL (22), el pin 5 del TF a GND y el pin 6 a GPIO27.
5. Vuelve a conectar los dos ESP32 a la Pi.

### 1.3 Corriente USB de la Raspberry Pi

La línea va en **`/boot/firmware/config.txt`**, el archivo de configuración de arranque de la Pi, en la tarjeta SD. Se edita por SSH y **queda guardado: sobrevive a reinicios y apagados**. Solo se pierde si se reflashea la SD.

[P]
```bash
tail -n 5 /boot/firmware/config.txt       # mira cómo termina el archivo
sudo nano /boot/firmware/config.txt
```

En nano, baja hasta el final del archivo. La línea tiene que quedar **debajo de `[all]`**. Si la última sección es otra (por ejemplo `[cm4]`), escribe primero `[all]` en una línea. Agrega:

```
usb_max_current_enable=1
```

Guarda con Ctrl+O y Enter, y sal con Ctrl+X. Luego `sudo reboot`. El SSH se corta; espera un minuto y vuelve a entrar.

Comprobación [P]: `vcgencmd get_config usb_max_current_enable`. **Debes ver:** `usb_max_current_enable=1`.

Otros ajustes de una sola vez [P]:
```bash
sudo usermod -aG dialout $USER                       # permiso para /dev/ttyUSB*; salir y volver a entrar
sudo systemctl disable --now ModemManager 2>/dev/null
```

## 2. Revisar alimentación y puertos

[P] `python3 diagnostico.py salud`. **Debes ver:** `throttled=0x0` y ninguna línea con `!!`.

| Valor de `throttled` | Significado |
|---|---|
| `0x50000` | Hubo bajo voltaje desde que encendió. Revisar el cable USB-C y el power bank |
| `0x50005` | Hay bajo voltaje ahora |

[P] `python3 diagnostico.py puertos`. **Debes ver:** una línea terminada en `LOCO` y otra en `PERC`. Si quieres fijarlas, copia esas rutas `/dev/serial/by-path/...` en `PUERTO_LOCO` y `PUERTO_PERC` de `config.py`.

### 2.1 Si `puertos` no identifica un ESP32

`puertos` imprime, debajo de cada puerto, el motivo real:

| Mensaje | Qué significa | Qué hacer |
|---|---|---|
| `SIN PERMISO` | El usuario no está en el grupo `dialout` | `sudo usermod -aG dialout $USER`, salir y volver a entrar por SSH; `groups` debe incluir `dialout` |
| `no se pudo abrir: ...` | Otro programa tiene el puerto, o hay un error del sistema (el texto lo dice) | `sudo fuser -v /dev/ttyUSB*` debe salir vacío; cerrar lo que aparezca |
| `no llego ningun byte` | El ESP32 no está corriendo su programa (en reset, en modo de carga o sin firmware) | Prueba directa de abajo |
| `llegaron N bytes pero ninguna identificacion` | El ESP32 transmite pero no se entiende la respuesta | Mándame esa línea completa (trae los primeros bytes) |

**Prueba directa, sin los scripts del proyecto** [P], para cada puerto (`/dev/ttyUSB0` y `/dev/ttyUSB1`):

```bash
stty -F /dev/ttyUSB0 115200 raw -echo
timeout 3 cat /dev/ttyUSB0 | head -c 400; echo
```

- **Salen líneas `L,...` o `T,...`:** el ESP32 y la velocidad están bien; el problema está en el script. Mándame la salida de `puertos`.
- **No sale nada:** mira el arranque a 115200 y, mientras corre, presiona el botón EN de la placa:
  ```bash
  stty -F /dev/ttyUSB0 115200 raw -echo
  timeout 5 cat /dev/ttyUSB0
  ```

  | Mensaje del arranque | Significado |
  |---|---|
  | `boot:0x13 (SPI_FAST_FLASH_BOOT)` | Arranque normal |
  | `boot:0x3 (DOWNLOAD_BOOT...)` o `waiting for download` | Quedó en modo de carga |
  | `flash read err` repetido | Un pin de arranque está mal polarizado (GPIO12 en alto al encender) |

## 3. Cómo se revisa que no se pierdan tramas

Hay tres niveles. Se usan juntos.

1. **En vivo:** [P] `python3 diagnostico.py monitor --barrer`. Cada medio segundo muestra:
   - **Tramas por segundo:** LOCO ≈ 50 y PERC ≈ 125 (100 T + 25 S).
   - **`malas` y `perdidas`:** líneas con checksum malo y huecos en el número de secuencia, contados en la Pi. Deben quedarse en 0.
   - **`ESP32: ... tx_perdidas`:** tramas que el propio ESP32 tuvo que descartar porque el USB no las sacó a tiempo. También 0.
   - **Watchdogs:** en locomoción, veces que frenó por no recibir comandos.

   Ctrl+C para salir.
2. **En cada registro:** todo lo que se corre deja una carpeta `registros/AAAAMMDD_HHMMSS/`. [P] `python3 exportar.py` (toma la más reciente) genera los CSV y `resumen.txt` con:
   - frecuencia real de cada trama frente a la esperada;
   - tramas perdidas por secuencia;
   - reinicios del ESP32 (la secuencia vuelve a 0, típico de un bajón de voltaje o un USB que se reconectó);
   - **mayor hueco** entre tramas (más de 100 ms en L o T es un problema);
   - los contadores de los ESP32 al final.
3. **Prueba de esfuerzo:** robot levantado, servo barriendo y motores girando 5 minutos. Es la condición con más ruido eléctrico y más consumo:
   - [P] `python3 diagnostico.py teleop`;
   - `r` para armar, `w` dos veces, y deja girar. Cambia de vez en cuando con `a`, `d`, `q`, `e`;
   - después de 5 min, espacio y `z`;
   - [P] `python3 exportar.py`.

   **Criterio:** perdidas 0, malas 0, reinicios 0, `tx_perdidas=0` y `throttled=0x0` al final (`diagnostico.py salud`).

> monitor y teleop no pueden correr al mismo tiempo, porque los dos abren los mismos puertos. teleop también graba todo, así que basta con exportarlo después.

## 4. Pasarme los datos

[P] `python3 exportar.py --zip`. Al final imprime la ruta del `.zip`, por ejemplo `registros/20261124_101500.zip`.

[L] (PowerShell ya trae `scp`):
```powershell
scp fede@robo-pi-5.local:~/Robot_Movil_Autonomo/Proyecto_Final/registros/20261124_101500.zip $HOME\Downloads\
```

En el hotspot, usa `fede@192.168.50.1` en lugar de `fede@robo-pi-5.local`. Adjunta el `.zip` en el chat, o solo el CSV de la calibración y `resumen.txt` si el zip es muy grande.

## 5. Calibraciones

Todas imprimen los valores para pegar y guardan su CSV en `registros/<fecha>/`. Si corres en la laptop, ejecútalas desde la carpeta `Proyecto_Final` del repo con el puerto antes del subcomando: `python calibrar.py --loco COM5 ff`.

### 5.1 Feedforward de los motores — `calibrar.py ff`

**Para qué:** el feedforward del firmware tiene dos partes (loco-final-1.1):
- **impulso de arranque** (`PWM_ARRANQUE_ADE/ATR`): PWM que hace falta para que cada rueda arranque desde reposo (con 2×18650: 105–165 según rueda y sentido);
- **curva de mantenimiento** (`FF_RPM_ADE/ATR` sobre `FF_PWM` = 95..255): RPM que da cada PWM con la rueda ya girando. Por debajo de `PWM_PARADA` (~85) la rueda se detiene.

Ya están medidas (26 nov, robot levantado, baterías cargadas). Repetir solo si cambian los motores, los drivers o la alimentación.

**Montaje:** robot **levantado**, ruedas libres, batería de motores cargada. Cada corrida tarda ~1.5 min.

[P]
```bash
python3 calibrar.py ff                       # subiendo: arranque desde reposo
python3 calibrar.py ff --bajando --paso 10   # bajando sin detener: curva de mantenimiento
```

Al final de cada una aparecen las constantes listas para pegar en `locomocion.ino`, y queda `ff.csv` o `ff_bajando.csv`.

**Mándame:** la salida de las dos corridas.

### 5.2 Verificación del PI — `calibrar.py pi`

**Para qué:** comprobar que con el feedforward nuevo cada rueda sigue su referencia.

**Montaje:** robot levantado. Unos 35 s.

[P] `python3 calibrar.py pi`. Prueba 8 movimientos de 3 s cada uno: adelante a 0.10, 0.20 y 0.28 m/s, atrás, izquierda, derecha y los dos giros.

**Debes ver:** por movimiento, `error medio FL +2.1% FR -1.4% ...`. **Criterio:** todos dentro de ±5 %. Un error grande y siempre del mismo signo en una rueda indica que su feedforward está mal.

**Mándame:** `pi.csv`, que trae referencia, RPM medida y PWM de cada rueda a 50 Hz.

### 5.3 Odometría — `calibrar.py odometria`

**Para qué:** radio efectivo de rueda y L = a + b reales. Sin esto, la pose acumula error de escala.

**Montaje:**
- en el piso donde será la prueba (el agarre cambia según el piso);
- 2.5 m libres al frente;
- cinta métrica y cinta adhesiva para marcar.

[P] `python3 calibrar.py odometria`.
1. Marca con cinta la posición del **centro** del robot y Enter. Avanza 2.0 m según su odometría y se detiene.
2. Mide con la cinta métrica cuánto avanzó **realmente** el centro y escríbelo en metros (ej. `1.87`).
3. Pega una cinta en el piso alineada con un borde del chasís y Enter. Da 2 vueltas según su odometría.
4. Escribe cuántos grados se pasó (+) o le faltaron (−). Ej. si se quedó 20° antes de completar las 2 vueltas: `-20`.

**Debes ver:** `RADIO_RUEDA = 0.0xxx` y `L_GEOM = 0.1xxx`. Cópialos en `config.py`. Si cambian más de 5 %, cámbialos también en `locomocion.ino` (`RADIO`, `L_GEOM`) y recarga el firmware.

**Criterio:** repite la prueba con los valores nuevos; la recta debe quedar a ±3 cm.

**Mándame:** `odometria.csv` de las dos corridas.

### 5.4 Anclas del servo (manual)

**Para qué:** que "60°" en el firmware sean 60° reales.

**Montaje:** transportador centrado en el eje del servo, alineado con el frente del robot.

[P]
```bash
python3 diagnostico.py servo F 0
python3 diagnostico.py servo F 60
python3 diagnostico.py servo F -60
```

En `F` el servo se queda quieto en ese ángulo; mide el ángulo real del soporte del TF en cada caso.

**Ajuste en `percepcion.ino`:** 1° ≈ 5.6 µs.
- Si con `F 0` el TF no mira al frente, corrige `SERVO_US_CENTRO`.
- Si con `F 60` llega a 55°, súmale 5 × 5.6 ≈ 28 a `SERVO_US_MAS60`. Lo mismo con `SERVO_US_MENOS60`.
- Si `F 60` gira hacia la **derecha**, cambia `SERVO_SIGNO` a −1.

Recarga el firmware y repite hasta quedar a ±1–2°. `python3 diagnostico.py servo R` le quita la señal.

### 5.5 Retardo (lag) del barrido — `calibrar.py lag`

**Para qué:** el ángulo de cada lectura del TF-Luna se estima con la consigna del servo de hace *lag* ms. Esta prueba mide ese retardo en cada sentido.

**Montaje:**
- una caja con un canto vertical a ~0.8 m al frente, cerca de 0°;
- detrás, nada en ~2 m (fondo lejano), para que el canto sea un salto claro de distancia;
- el servo con su alimentación conectada.

[P] `python3 calibrar.py lag`.
1. Primero mueve el servo de −15° a +15° en pasos de 0.5°, deteniéndose en cada uno: es el ángulo real del canto.
2. Luego barre ±30° a 150°/s durante 8 s.

**Debes ver:**
- `Borde estatico en x.xx grados (salto NNN mm)`: el salto debería ser de cientos de mm;
- después, `LAG antihorario NN ms, horario NN ms`.

**Probar el resultado:** fija los valores en `percepcion.ino` (`lagCcwMs`, `lagCwMs`), recarga y repite `calibrar.py lag`: la segunda corrida debe dar el borde barriendo a ±1–2° del estático. (`diagnostico.py servo Z` no sirve para esto: cada programa que abre el puerto reinicia el ESP32 y el valor se pierde.)

**Mándame:** `lag.csv` (muestras estáticas, subiendo y bajando).

> Si el servo sigue alimentado por el PowerBoost y se corta o tironea, esta medición sale inestable. Esa es la señal para pasarlo al UBEC de 6 V.

### 5.6 Sharp — `calibrar.py sharp`

**Para qué:** reemplazar la curva genérica de la hoja de datos por la de **estos** dos sensores.

**Montaje:** un cartón blanco plano, perpendicular al haz, y una regla. Sirve medir los dos Sharp a la vez si el cartón queda a la misma distancia de ambas caras. Si no, haz una corrida por sensor y usa solo la columna que corresponde.

[P] `python3 calibrar.py sharp`. Pide 100, 150, 200, 300, 400, 500, 600 y 800 mm. Pon el cartón, Enter, y mide 1.5 s.

**Debes ver:** `izq NNN mV der NNN mV` en cada distancia; los mV bajan al alejar. Al final aparece un bloque `const PuntoCal SHARP_TABLA...`: pégalo en `percepcion.ino` en lugar de la tabla vacía y recarga.

**Mándame:** `sharp.csv`, con todas las muestras, no solo la mediana.

### 5.7 Geometría de los Sharp (manual)

1. Lee en la escala grabada el yaw de cada soporte → `SHARP_YAW_DEG` en `geometria_robot.py`.
2. Mide la distancia del tornillo pivote a la ventana del sensor → `SHARP_CARA_DESDE_PIVOTE_MM`.

### 5.8 Cámara — altura, inclinación, FOV y color

**Montaje:**
1. Monta la cámara inclinada **12° hacia abajo** (óptimo para la lente a 222 mm; ya está en `geometria_robot.py`). Para fijarla puedes usar un transportador o una app de nivel en el celular apoyada sobre la carcasa de la cámara.
2. Las alturas ya están cargadas (L4 a 128 mm, TF a 156 mm, lente a 222 mm). Si cambian, `python3 geometria_robot.py` recalcula la inclinación óptima.

**Comprobar la imagen:** [P] `python3 diagnostico.py camara --segundos 10 --fotos 5`.
- **Debes ver:** ~15 fps y los conos con distancia y rumbo, por ejemplo `(1.48 m +12 piso)`.
- Las fotos quedan en `registros/<fecha>/fotos/`. Si salen borrosas, prueba `"autofoco": false` en `config_conos.json`.

**FOV e inclinación reales:** pon un cono a 1.5 m al frente del centro del robot y 0.5 m a la izquierda; nada más naranja a la vista.
- [P] `python3 calibrar.py camara --distancia 1.5 --lateral 0.5`.
- Imprime el FOV horizontal y la inclinación medidos. Cópialos en `config_conos.json` (`fov_horizontal_grados`), `config.py` (`CAMARA_FOV_H_DEG`) y `geometria_robot.py` (`CAMARA_INCLINACION_DEG`).

**Color (en el lugar de la prueba, con la luz real):** se hace en la laptop, porque necesita ventanas.
1. Desconecta la cámara de la Pi y conéctala a la laptop.
2. [L], en `Proyecto_Final`: `python herramientas_conos\paso1_probar_camara.py --listar` para ver el índice de la cámara.
3. [L] `python herramientas_conos\paso2_calibrar_color.py --camara 1`, con el índice que encontraste.
   - Clic sobre el cono, ajusta las barras hasta que en "máscara" el cono quede blanco y el fondo negro.
   - Prueba lejos, cerca y con sombra. `g` guarda.
4. Copia el archivo a la Pi [L]:
   ```powershell
   scp config_conos.json fede@robo-pi-5.local:~/Robot_Movil_Autonomo/Proyecto_Final/
   ```
   **Antes de copiarlo, abre el JSON y vuelve a poner `"camara": 0`**: en la laptop queda el índice de la laptop.
5. Reconecta la cámara a la Pi y repite `diagnostico.py camara`.

**Mándame:** 3–4 fotos del pasillo y del laboratorio con conos a distintas distancias. Sirven para revisar el color y la geometría sin estar ahí.

## 6. Registro de sensores para análisis

Para grabar todo lo que miden los sensores, por ejemplo al pasar una caja por el frente o con el robot quieto frente a una pared:

[P] `python3 diagnostico.py monitor --barrer`, hacer la prueba y Ctrl+C. Luego `python3 exportar.py --zip`.

Contenido de `perc_T.csv`, cada lectura del TF-Luna:
- `t_pi`: tiempo de llegada a la Pi;
- `ms`: tiempo de la medición en el ESP32;
- `ang_cdeg`: ángulo del cabezal en centésimas de grado, positivo a la izquierda;
- `dist_mm`, `amp` y `flags` (bit0 = inválida).

Contenido de `perc_S.csv`: los dos Sharp en mm y en mV, y el IR trasero.

Con el robot quieto frente a una pared a distancia conocida, estos CSV sirven para medir el ruido y el sesgo de cada sensor.

## 7. Pruebas de navegación

Solo después de 5.1–5.8.

[P] `python3 robot.py`. Espera el botón: presionarlo arranca, presionarlo de nuevo es paro de emergencia, y otra pulsación continúa la misión. En la terminal, una línea por segundo con estado, objetivo, pose, pasillo libre, enlaces y conos. Ctrl+C detiene el robot y cierra el registro.

Empiecen por tramos cortos: inicio → P1, luego P1 → P2 con una caja en el camino. Después de cada intento, [P] `python3 exportar.py --zip` y mándame el zip. Los archivos clave para entender qué pasó son `eventos.txt` (transiciones, puertas vistas, correcciones de pose) y `nav.csv`.

Para probar sin la cámara (solo odometría y esquivar): `python3 robot.py --sin-camara`.

## 8. Problemas comunes

| Síntoma | Causa probable |
|---|---|
| `puertos` no encuentra un ESP32 | Monitor Serie abierto en la laptop, cable solo de carga, o falta `dialout` |
| `perdidas` sube y `reinicios` > 0 | Bajo voltaje en los USB: `diagnostico.py salud`, `usb_max_current_enable`, cable USB-C |
| El robot no se mueve | No está armado (LED de locomoción apagado): botón o `r` en teleop |
| Se detiene cada 0.5 s | El programa de la Pi no manda `V` (se cerró o se trabó). En el monitor aparece `watchdogs` > 0 |
| IR frontal siempre en obstáculo | KY-032 desconectado (GPIO34 flotando): `IR_FRENTE_HABILITADO = False` en `config.py`, o tecla `i` en teleop |
| El cabezal vuelve al centro solo | 3 s sin mensajes de la Pi (keepalive): el script se cerró |
| `T` llega pero `tf_validas_s` = 0 | El TF no ve nada en 0.2–8 m, o hay un objeto a menos de 20 cm |
