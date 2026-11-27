# Estado del robot — documento de traspaso

**Última actualización:** 26 nov 2026, 20:45 (Esteban). **Prueba:** miércoles 2 dic, 8:30, F3-03. **Informe y video:** antes del viernes 4 dic, 7:30.

Este archivo es el punto de partida para cualquiera del equipo que trabaje con el robot. Dice qué funciona, qué falta, cómo conectarse y qué cuidar. **Cada prueba o cambio se anota aquí** (ver [cómo mantenerlo](#cómo-mantener-este-documento)). El paso a paso de cada prueba está en [`Proyecto_Final/GUIA_PRUEBAS.md`](Proyecto_Final/GUIA_PRUEBAS.md); la arquitectura, el protocolo y el pinout, en [`Proyecto_Final/README.md`](Proyecto_Final/README.md).

**Resumen:** los enlaces USB, la alimentación, el barrido del TF-Luna y el control de las ruedas (con el robot levantado) están verificados. Falta todo lo que se hace en el piso: control con carga, odometría, Sharp, cámara, y las corridas de navegación. El cable del servo es intermitente; hay un plan B con sensores fijos que se activa con una línea.

---

## Checklist

Marcas: ✅ hecho y verificado · 🟡 hecho sin verificar · ⬜ pendiente · ⛔ bloqueado por información externa.

### Enlaces, alimentación y Pi
- ✅ Los dos ESP32 por USB a la Pi, 115200 baud, sin WiFi. 0 tramas perdidas en todas las corridas del 26 nov.
- ✅ `usb_max_current_enable=1` en la Pi; `diagnostico.py salud` → `throttled=0x0` con todo conectado.
- ✅ Prueba de esfuerzo (sección 3 de la guía): `teleop` 4.3 min, servo barriendo y motores girando, sin pérdidas ni reinicios.
- ⬜ Instalar el script de red de respaldo en la Pi (ver [Red de la Pi](#red-de-la-pi-y-ssh)) y comprobarlo en dos lugares: uno con red conocida y otro sin ella.
- ⬜ Definir cómo arranca `robot.py` en la prueba sin laptop conectada. Hoy muere si se cierra el SSH (no maneja SIGHUP). Opciones: `nohup` o un servicio systemd que lo arranque y espere el botón.

### Locomoción
- ✅ Firmware `loco-final-1.1` cargado: feedforward remedido con 2×18650 e impulso de arranque.
- ✅ 5.1 Feedforward (robot levantado). Seguimiento ±2 RPM de 16 a 100 RPM, ambos sentidos.
- ⬜ 5.2 Verificación del PI **en el piso** (`calibrar.py pi`). Puede pedir más impulso de arranque o subir `I_TERM_MAX`.
- ⬜ 5.3 Odometría (`calibrar.py odometria`): `RADIO_RUEDA` y `L_GEOM` en `config.py` siguen sin calibrar.
- 🟡 KY-032 frontal conectado (GPIO34) y habilitado; no se ha probado frenando en el piso.

### Percepción
- ✅ Firmware `perc-final-1.3` cargado. TF-Luna a 102 Hz con el servo barriendo (antes 25 Hz; ver bitácora).
- ✅ 5.4 Anclas del servo: centro 1445 µs, ±60° en 1112/1778 µs. Coincide con las marcas de la placa.
- ⬜ 5.5 Lag del barrido (`calibrar.py lag`).
- ⬜ 5.6 Tablas de los Sharp (`calibrar.py sharp`) y 5.7 su ángulo real (están a 45° nominal).
- ⬜ Arreglar el cable del servo (conector triple + 3 jumpers, falla con algunas flexiones). Mientras tanto, ver [plan B](#plan-b-sensores-fijos).
- ⬜ KY-032 trasero: montado, sin conectar; es opcional.

### Cámara y visión
- ✅ Detección de conos probada en Windows; detección del ArUco de la prueba verificada con una foto (`DICT_ARUCO_ORIGINAL`, ID 1).
- 🟡 Cámara a 12° hacia abajo, lente a 222 mm del piso (medido 25 nov).
- ⬜ 5.8 FOV real de la cámara y color de los conos **en el lugar de la prueba** (el HSV actual se calibró en otro lugar).
- ⬜ Probar la detección de conos y ArUco en la Pi con `diagnostico.py camara`.

### Navegación y prueba final
- ✅ Simulador: completa la ruta con barrido y con sensores fijos (una corrida de cada uno; el simulador es idealizado).
- ⬜ Corridas reales por tramos: recta con una puerta, una puerta con obstáculo, estacionar en el ArUco.
- ⛔ Posiciones reales de las puertas en `mapa.py`: el croquis no está a escala y todo lo que no tiene cota está ESTIMADO.
- ⛔ Lado real del ArUco (`ARUCO_LADO_M` = 0.10, estimado de la foto), su altura y si está en pared o en piso.
- ⬜ `D_ESTACIONAR` (distancia final al marcador, 0.25 m) según dónde quede el marcador.

### Entrega
- ⬜ Informe IEEE (≤ 6 páginas): diagrama físico con dimensiones, diagrama eléctrico y lista de componentes, ecuaciones cinemáticas, diagrama de estados, visión, resultados experimentales (navegación, error de odometría, precisión de llegada).
- ⬜ Video.

---

## Configuración activa

| Qué | Valor | Dónde |
|---|---|---|
| Firmware locomoción | `loco-final-1.1` | `Avance_1_Locomocion/locomocion/locomocion.ino` |
| Firmware percepción | `perc-final-1.3` | `Avance_2_Percepcion/percepcion/percepcion.ino` |
| Enlace serie | 115200 baud, ESP32 a 80 MHz | firmwares y `config.py` |
| Puertos USB de la Pi | LOCO en el USB 2.0 **inferior**, PERC en el USB 2.0 **superior**, cámara en USB 3.0 | `config.py` (`PUERTO_LOCO`, `PUERTO_PERC`) |
| Barrido del TF-Luna | ±60° a 150 °/s (`BARRIDO_AMPLITUD_DEG = 60`) | `config.py` |
| Servo | centro 1445 µs, −60° = 1112 µs, +60° = 1778 µs; límite de firmware ±60° | `percepcion.ino` |
| PWM del servo | MCPWM (no LEDC: `ledcWrite` bloqueaba 37 ms) | `percepcion.ino` |
| Sharp | fijos a ±45° hacia afuera | `geometria_robot.py` |
| Cámara | 640×480, 12° hacia abajo, lente a 222 mm | `config.py`, `geometria_robot.py` |
| Conos | 23.5 cm de alto (medido), puertas de 50 cm (enunciado) | `config.py`, `config_conos.json` |
| ArUco | `DICT_ARUCO_ORIGINAL`, ID 1, lado ~0.10 m (estimado) | `config.py` |
| Rueda / geometría | radio 0.029 m, L = 0.15 m (sin calibrar) | `config.py` y `locomocion.ino` |
| Velocidades | crucero 0.20 m/s, cruce 0.15, aproximación 0.08, rueda máx. 120 RPM | `config.py` |
| Feedforward | curva de mantenimiento 95..255 PWM + impulso de arranque por rueda | `locomocion.ino` |

### Plan B: sensores fijos

Si el servo falla: fijar el soporte del TF-Luna mirando al frente (con tornillo o cinta, que no dependa del servo) y poner `BARRIDO_AMPLITUD_DEG = 0` en `config.py`. La Pi manda `B,0,…`, el firmware queda fijo en 0° y la falda usa el TF-Luna como un rayo al frente. No hay que tocar firmware. En el simulador la ruta se completa igual. Riesgo: el estado EVITAR se desplaza a ±60°, donde solo ve el Sharp de 45° (hasta ~0.8 m). Si en la pista esquiva mal, hay que ajustar `navegacion.py`.

---

## Conexión y arranque

### Encender el robot
1. Robot **levantado** (sobre una caja) si se van a mover las ruedas en una prueba de banco.
2. Power bank a la Pi (USB-C). Esperar ~1 min a que arranque y se conecte a la red (ver abajo).
3. Los dos ESP32 y la cámara ya van en los USB de la Pi. **No intercambiar** LOCO (inferior) y PERC (superior).
4. Conectar el JST de las 2×18650 (motores y buck del servo) solo cuando se vaya a usar. Se desconecta para programar o cuando no se usa.
5. El botón arma y desarma (paro de emergencia). En las pruebas por SSH también se arma con `r` en `teleop`.

### Red de la Pi y SSH

La Pi (usuario `fede`, nombre `robo-pi-5`) usa NetworkManager. Al arrancar intenta las redes WiFi guardadas, en orden de prioridad. Si no encuentra ninguna, levanta su propio hotspot `Robot-MT9005`. Las claves se pasan por privado, no están en el repo.

**Entrar por SSH** desde PowerShell en la laptop:

| La Pi está en… | Comando |
|---|---|
| la misma red que la laptop | `ssh fede@robo-pi-5.local` (si el nombre no resuelve, usar la IP: ver abajo) |
| su hotspot `Robot-MT9005` | conectar la laptop a `Robot-MT9005` y `ssh fede@192.168.50.1` (sin internet) |
| wTEC u otra red institucional | puede que la laptop no vea a la Pi (aislamiento de clientes, mDNS bloqueado). Usar la IP o Tailscale si está instalado (`tailscale status` en la Pi) |

Para saber la IP o la red actual, en la Pi: `hostname -I` y `nmcli -t -f NAME,DEVICE con show --active`. Sin SSH, conectar monitor (micro-HDMI) y teclado.

**Agregar una red propia** (en la Pi, por SSH o con monitor):
```bash
sudo nmcli dev wifi list                                      # redes visibles
sudo nmcli dev wifi connect "NOMBRE_RED" password "CLAVE"     # se guarda y queda con autoconexión
sudo nmcli con modify "NOMBRE_RED" connection.autoconnect-priority 50
nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY con show  # revisar
```
Correrlo conectado por SSH desde **otra** red corta la sesión, porque la Pi tiene un solo radio WiFi. Hacerlo con monitor, o estando ya en esa red.

**Hotspot solo como respaldo** (una vez, en la Pi; pendiente de instalar):
```bash
cd ~/Robot_Movil_Autonomo && git pull
sudo cp Proyecto_Final/sistema/red_respaldo.sh /usr/local/bin/
sudo cp Proyecto_Final/sistema/red-respaldo.service /etc/systemd/system/
sudo nmcli con modify robot-ap connection.autoconnect no
sudo systemctl daemon-reload && sudo systemctl enable red-respaldo.service
```
Con esto, al arrancar la Pi espera hasta 60 s a conectarse a alguna red guardada. Si lo logra, queda con internet y sin hotspot. Si no, levanta `robot-ap`. Una vez en hotspot no vuelve a buscar redes hasta reiniciar. Para volver antes: `sudo nmcli con up "NOMBRE_RED"`. Para ver qué decidió al arrancar: `journalctl -t red-respaldo -b`.

Para forzar el hotspot aunque haya red: `sudo nmcli con up robot-ap`. Si en algún momento se quiere el hotspot siempre, deshacer con `sudo systemctl disable red-respaldo.service` y `sudo nmcli con modify robot-ap connection.autoconnect yes`.

### Actualizar el código
- **Laptop:** GitHub Desktop → rama `main` → Fetch origin → Pull.
- **Pi:** `cd ~/Robot_Movil_Autonomo && git pull`.
- **Firmware:** Arduino IDE, placa "ESP32 Dev Module" (core esp32 3.x), ESP32 conectado a la laptop. Al terminar, de vuelta al mismo USB de la Pi. Comprobar la versión: en el Monitor Serie a 115200, escribir `?`; responde `I,LOCO,loco-final-…` o `I,PERC,perc-final-…`. En la Pi también queda en `crudo_*.txt` de cada registro.

### Comandos de uso diario (en la Pi, carpeta `~/Robot_Movil_Autonomo/Proyecto_Final`)

| Comando | Para qué |
|---|---|
| `python3 diagnostico.py salud` | temperatura y alimentación (`throttled=0x0` es lo correcto) |
| `python3 diagnostico.py puertos` | qué ESP32 hay en cada USB |
| `python3 diagnostico.py monitor --barrer` | enlaces, TF, Sharp y odometría en vivo; Ctrl+C sale |
| `python3 diagnostico.py teleop` | manejar con el teclado (`r` arma, `w/s/a/d/q/e` suman velocidad, espacio frena, `z` sale) |
| `python3 diagnostico.py servo F 30` | mantener el servo en un ángulo (Ctrl+C sale) |
| `python3 diagnostico.py camara` | fps, conos y ArUco detectados; guarda fotos |
| `python3 exportar.py --zip` | convierte el último registro a CSV + `resumen.txt` y lo comprime |
| `python3 robot.py` | programa de la prueba: espera el botón |
| `python3 robot.py --sim` | simulador completo en cualquier PC |

Para copiar un zip a la laptop (PowerShell): `scp fede@<IP o robo-pi-5.local>:~/Robot_Movil_Autonomo/Proyecto_Final/registros/<carpeta>.zip $HOME\Downloads\`.

---

## Cuidados y fallos conocidos

- **Abrir el puerto reinicia el ESP32.** Cada programa que se conecta reinicia la placa (~0.6 s de arranque): el servo queda sin señal y se pierde cualquier ajuste mandado por comando (`Z`, `F`). Los valores permanentes van en el firmware.
- **Un programa por puerto.** Cerrar el Monitor Serie del Arduino IDE antes de correr scripts; `monitor`, `teleop` y `calibrar` no pueden correr a la vez.
- **Salir con `z` en `teleop`.** Ctrl+C también detiene y guarda, pero imprime una traza `KeyboardInterrupt` (inofensiva).
- **Las "líneas malas" del primer segundo** en `resumen.txt` son basura del arranque del ESP32. Solo preocupan si aparecen después.
- **Cable del servo intermitente.** Antes de una corrida, mover el cabezal a mano por todo el recorrido con el monitor abierto y verificar que el servo responde. GND del servo directo al OUT− del buck; un solo cable OUT− → GND del ESP32.
- **Buck del servo a 6.0 V.** Con las 18650 por debajo de ~7 V no alcanza en los picos. Cargar las baterías antes de las pruebas.
- **El servo tiene una zona muerta de ~±2°** (de −9° a −12° de las anclas viejas no se movía). El lag por sentido (5.5) absorbe parte.
- **Feedforward medido con el robot levantado y las baterías cargadas.** En el piso o con la batería baja, el impulso de arranque puede no alcanzar. Si una rueda se queda trabada, es lo primero que hay que revisar (ver la bitácora del 26 nov).
- **El límite de ±60° del servo es del firmware** (`SERVO_LIMITE_DEG`). Pedir 70° deja el servo en 60°.
- **Enunciado:** el robot no puede quedarse más de 1 min sin avanzar; `T_SIN_AVANCE_S = 30` fuerza una evasión antes.
- **El repo es público.** No subir claves, IPs de casa ni datos personales.

---

## Datos de la pista

Del enunciado ([`Documentos/Enunciados/`](Documentos/Enunciados/)) y de lo medido:

- Ruta: pasillo principal → pasillo intermedio → laboratorio F3-03. Croquis en la figura 2 del enunciado, cargado en [`Proyecto_Final/mapa.py`](Proyecto_Final/mapa.py) (`python3 mapa.py` lo dibuja). El croquis no está a escala: se respetan las cotas y lo demás está ESTIMADO.
- Conos naranjas en parejas formando puertas de 50 cm, distribuidas "de manera uniforme". Los conos reales miden **23.5 cm** de alto (el enunciado decía ~18). Base y punta sin medir. Foto: [`Documentos/img/cono_prueba.jpg`](Documentos/img/cono_prueba.jpg).
- Obstáculos no mapeados: cajas u objetos opacos de al menos 10 cm de alto, en el pasillo intermedio y en el laboratorio. Chocar penaliza.
- ArUco final: **`DICT_ARUCO_ORIGINAL`, ID 1**, impreso en media hoja carta y pegado en la pared, no muy alto. Lado del cuadro negro ≈ 10 cm (estimado de la foto, sin medir). Foto: [`Documentos/img/aruco_prueba_id1.jpg`](Documentos/img/aruco_prueba_id1.jpg).
- Un botón de arranque y paro. Después del arranque no se permite ninguna interacción física ni por radiofrecuencia.

## Temas por consultar al profesor

1. Posición real de cada par de conos, o al menos la distancia entre puertas.
2. ArUco: tamaño exacto, altura sobre el piso, en qué pared y qué orientación final se espera (¿de frente a la pared a qué distancia?). ¿Será el ID 1?
3. Pose inicial: ¿dónde y con qué orientación se coloca el robot en "Inicio"?
4. ¿Cuenta como "interacción por radiofrecuencia" tener una sesión SSH abierta (sin mandar nada) durante la corrida? Define si `robot.py` se lanza por SSH o arranca solo.
5. ¿Puede haber obstáculos justo antes o después de una puerta, o solo entre puertas?

---

## Cómo trabajar con GitHub

- Todo se trabaja sobre **`main`**. La rama `final-limpieza` ya se unió (PR #3, 26 nov) y se puede borrar desde GitHub.
- **Antes de empezar:** Pull en la laptop y `git pull` en la Pi.
- **Al terminar un cambio:** commit con un mensaje que diga qué y por qué (por ejemplo "locomocion 1.2: sube I_TERM_MAX a 50 tras la prueba en piso"), push, y anotar el resultado en la bitácora de este archivo.
- Si cambian un firmware, suban la versión (`VERSION[]`) para poder confirmar en la Pi cuál está cargado.
- No se suben: `Proyecto_Final/registros/` (ya está en `.gitignore`), claves ni archivos personales. Los zip de registros se comparten por el chat.
- Si dos personas tocan el mismo archivo y GitHub Desktop marca conflicto: no forzar; resolverlo con quien hizo el otro cambio.

## Cómo mantener este documento

Después de cada prueba o cambio:
1. Actualizar el checklist y, si cambió un valor, la tabla de configuración.
2. Agregar una entrada arriba en la bitácora: fecha, quién, qué se hizo, resultado con números, y el nombre del registro (`AAAAMMDD_HHMMSS`).
3. Cambiar la fecha de "Última actualización".

---

## Bitácora de pruebas

Lo más reciente arriba.

### 26 nov 2026 (Esteban)
- **Unión a `main`** del PR #3 (`final-limpieza`). Conos a 23.5 cm y ArUco ORIGINAL/ID 1 en `config.py`. Script de red de respaldo agregado (sin instalar).
- **`teleop` a baja velocidad con `loco-final-1.1`** (registro `20261126_191124`, robot levantado): 16 RPM pedidas → 16 ± 2 medidas en las 4 ruedas; 33, 49, 66, 82 y 99 RPM con ±1–2 RPM, en ambos sentidos y en lateral. 0 tramas perdidas.
- **Feedforward remedido** (robot levantado, 2×18650 cargadas): desde reposo las ruedas arrancan con PWM 150–165 adelante y 105–165 atrás, según la rueda. Ya girando, se sostienen hasta ~PWM 90 (~20 RPM) y se paran en ~85. Ese salto explicaba que a baja velocidad unas ruedas giraran y otras no. → `loco-final-1.1`: curva de mantenimiento + impulso de arranque.
- **Prueba de esfuerzo** (registro `20261126_184725`): `teleop` 4.3 min con barrido y motores: 0 pérdidas, 0 reinicios, TF 102 Hz, `throttled=0x0`.
- **Servo:** era de posición (no de giro continuo). El frente real quedó en −10° de las anclas viejas → centro a 1445 µs (`perc-final-1.3`). ±60° coinciden con las marcas de la placa.
- **`diagnostico.py servo F …` no hacía nada:** el script cerraba el puerto antes de que el ESP32 terminara de arrancar. Corregido: espera al ESP32 y mantiene el comando hasta Ctrl+C.
- **TF-Luna a 25 Hz con barrido:** `ledcWrite` tardaba ~36.8 ms por llamada y bloqueaba el lazo. `perc-final-1.1` genera el PWM del servo con MCPWM → TF a 102 Hz, lazo máximo 2.2 ms.
- **Enlace serie:** 230400 baud corrompía bytes en percepción → todo a 115200. El driver cp210x de Linux a veces dejaba el chip a otra velocidad → `enlaces.abrir_puerto` lo fuerza (9600 → 115200).
- **Cable del servo intermitente** (conector triple + jumpers). Se dejó listo el plan B con sensores fijos.
