# Guía paso a paso — Tarea 8 (desde cero, sin experiencia con GitHub ni SSH)

Sigue los pasos en orden. Cada bloque de comandos dice **dónde** se escribe y **qué tienes que ver**. Copia los comandos tal cual, uno por uno, y pulsa Enter después de cada uno. Si algo no coincide con lo que dice "Debes ver", detente y busca el error en la sección 9.

---

## 0. Las tres "terminales" y cómo saber en cuál estás

Vas a usar tres lugares distintos. Lo que te dice dónde estás es el **prompt**: el texto que aparece antes del cursor.

| Lugar | Cómo se abre | Prompt típico | Para qué |
|---|---|---|---|
| **A. PowerShell en la laptop** | Tecla Windows → escribir `PowerShell` → Enter | `PS C:\Users\TuNombre>` | git (subir el repo) y conectarse por SSH |
| **B. SSH en la Raspberry Pi** | desde A, con `ssh fede@...` | `fede@robo-pi-5:~ $` | todo lo de la Pi |
| **C. Arduino IDE** | programa normal | — | flashear el ESP32 |

- Si ves `PS C:\...>`, los comandos se ejecutan en **tu laptop**.
- Si ves `fede@robo-pi-5:... $`, se ejecutan en **la Pi**, aunque la ventana sea la misma PowerShell.
- La parte entre `:` y `$` es la carpeta actual de la Pi. `~` significa `/home/fede`. Por ejemplo, `fede@robo-pi-5:~/Robot_Movil_Autonomo/Tarea_Integracion $` quiere decir que estás en esa carpeta.

Comandos básicos (sirven en las dos):

| Comando | Qué hace |
|---|---|
| `pwd` | muestra en qué carpeta estás |
| `ls` | lista lo que hay en la carpeta |
| `cd nombre` | entra a una carpeta; `cd ..` sube una; `cd ~` vuelve a la carpeta personal |
| Tecla **Tab** | autocompleta nombres de carpetas y archivos (úsala siempre: evita errores) |
| Flecha **↑** | repite comandos anteriores |
| **Ctrl+C** | detiene el programa que está corriendo |
| `exit` | sale de la Pi y vuelve a la laptop (de B a A) |
| Pegar | clic derecho en la ventana de PowerShell (Ctrl+V a veces también funciona) |

Al escribir una contraseña en la terminal **no aparece nada**, ni puntos ni asteriscos. Es normal: escríbela y pulsa Enter.

---

## 1. Subir el código a GitHub (laptop, una sola vez)

Necesitas el archivo `tarea8.bundle` que te mandé, guardado en **Descargas**.

### 1.1 Instalar git (si no lo tienes)

**Dónde:** A (PowerShell).

```powershell
git --version
```

- **Debes ver:** `git version 2.xx...`.
- Si dice que no se reconoce `git`: instálalo con `winget install --id Git.Git -e` (o descárgalo de git-scm.com con las opciones por defecto). Después **cierra y vuelve a abrir PowerShell** y repite `git --version`.

Solo la primera vez, configura tu nombre y correo:

```powershell
git config --global user.name "Tu Nombre"
git config --global user.email "tu_correo@ejemplo.com"
```

### 1.2 Bajar el repo, aplicar el bundle y subir

**Dónde:** A. Se usa una carpeta nueva, `Robot_Tarea8` dentro de Documentos, para no mezclar con copias viejas.

```powershell
cd $HOME\Documents
git clone https://github.com/Quetrix/Robot_Movil_Autonomo.git Robot_Tarea8
cd Robot_Tarea8
```

- **Debes ver:** `Cloning into 'Robot_Tarea8'... done.` y luego el prompt `PS C:\Users\TuNombre\Documents\Robot_Tarea8>`.
- Si dice que la carpeta ya existe, cambia `Robot_Tarea8` por `Robot_Tarea8b` en los tres comandos.

Comprueba que el bundle esté en Descargas:

```powershell
ls $HOME\Downloads\*.bundle
```

- **Debes ver:** `tarea8.bundle`. Si el navegador le puso otro nombre, por ejemplo `tarea8 (1).bundle`, usa ese nombre entre comillas en el siguiente comando.

```powershell
git pull "$HOME\Downloads\tarea8.bundle" main
git log --oneline -7
```

- **Debes ver:** `Fast-forward` y una lista de archivos. En el log, arriba de todo, 5 líneas que empiezan con `Tarea 8:` o `Locomocion:`. La de más arriba es `Tarea 8: guia paso a paso y scripts revisar.sh / correr.sh`.

```powershell
git push origin main
```

- La primera vez se abre una ventana del navegador para iniciar sesión en GitHub: inicia sesión con la cuenta que tiene permiso de escritura en `Quetrix/Robot_Movil_Autonomo`.
- **Debes ver:** al final, `main -> main`.
- **Para verificarlo:** abre `https://github.com/Quetrix/Robot_Movil_Autonomo/commits/main`; deben aparecer los commits de la Tarea 8.

| Si sale | Qué hacer |
|---|---|
| `rejected ... fetch first` | alguien subió algo antes: `git pull --rebase origin main` y repite `git push origin main` |
| `403` o `Permission denied` | esa cuenta no tiene permiso: inicia sesión con la dueña del repo o pide que te agreguen como colaborador |
| `not a git repository` | no estás dentro de la carpeta: `cd $HOME\Documents\Robot_Tarea8` |

Desde ahora, en la laptop el código está en **`Documentos\Robot_Tarea8`**. Usa esa carpeta en el Arduino IDE.

---

## 2. Conexiones entre las partes

### 2.1 Qué se conecta con qué

```
                  WiFi "Percepcion-ESP32" (lo crea el cabezal)
     ┌──────────────────────────┬─────────────────────────────┐
     │                          │                             │
 [Laptop]                [Raspberry Pi 5]               [ESP32 cabezal]
  SSH ────WiFi───────────▶  192.168.4.200   ◀──WiFi──    192.168.4.1
                            USB-C ◀─ power bank A          ◀─ power bank B
                            USB-A ──cable USB──┐            (servo, 2 Sharp,
                                               │             TF-Luna: Avance 2)
                                               ▼
                                     [ESP32 locomoción]
                                      micro-USB: datos + 5 V
                                      cables ──▶ 2 drivers ──▶ 4 motores
                                      cables ◀── 4 encoders, KY-032
                                      GND ──── GND drivers ──── − batería
                                      batería 2×18650 (+) ──▶ VMS drivers
```

- **Entre los dos ESP32 no hay ningún cable.** El cabezal habla solo con la Pi, por WiFi.
- **La Pi y el ESP32 de locomoción** se unen solo con **un cable USB**: del puerto USB-A de la Pi (los rectangulares) al micro-USB del DevKit. Ese cable lleva los datos y la alimentación del ESP32.
- **La laptop** no se conecta con cables a nada: entra a la Pi por WiFi (SSH).

### 2.2 Lista de cables, uno por uno (batería de motores desconectada)

| # | Desde | Hasta | Verifica |
|---|---|---|---|
| 1 | power bank A (5 V / 3 A) | USB-C de la Pi | — |
| 2 | USB-A de la Pi | micro-USB del ESP32 de locomoción | que sea cable de datos, no solo de carga |
| 3 | power bank B | cabezal (como quedó en el Avance 2) | — |
| 4 | KY-032 `GND` | GND del ESP32 de locomoción | — |
| 5 | KY-032 `+` | **3.3 V** del ESP32 | con el multímetro: 3.3 V, no 5 V |
| 6 | KY-032 `OUT` | **GPIO34** | — |
| 7 | KY-032 `EN` | nada | el jumper "EN" del módulo debe estar puesto |
| 8 | batería (+) | VMS de **los dos** drivers | polaridad con el multímetro: 7.4–8.4 V |
| 9 | batería (−) | GND de los dos drivers **y** GND del ESP32 | continuidad entre los tres |
| 10 | motores, drivers y encoders | ya cableados | no tocar; pinout en `README.md`, sección 3.2 |

La batería de motores (cable 8) se conecta **al final**, con el robot levantado (sección 6).

---

## 3. Flashear el ESP32 de locomoción (laptop, Arduino IDE)

1. **Batería de motores desconectada.** Conecta el ESP32 de locomoción **a la laptop** por USB (después lo pasas a la Pi).
2. Arduino IDE → Archivo → Abrir → `Documentos\Robot_Tarea8\Avance_1_Locomocion\locomocion_esp32_ir_frontal\locomocion_esp32_ir_frontal.ino`.
3. Herramientas → Placa → **ESP32 Dev Module** (core "esp32 by Espressif" 3.x). Herramientas → Puerto → el COM que aparece al conectar.
4. Botón **Subir** (flecha). Si se queda en `Connecting.....`, mantén presionado el botón **BOOT** del ESP32 hasta que empiece a escribir.
5. **Debes ver** al final: `Hard resetting via RTS pin...`.
6. Herramientas → Monitor Serie, a la derecha **115200** y **Nueva línea**. Presiona el botón EN/RST del ESP32. **Debes ver:** `ROBOT MECANUM - LOCOMOCION IR FRONTAL` y `# IR FRENTE: LIBRE`.
7. Pon la mano frente al KY-032. **Debes ver:** `# IR FRENTE: OBSTACULO`; al quitarla, `# IR FRENTE: LIBRE`. Si no cambia, gira el potenciómetro del módulo hasta que detecte a unos 15 cm.
8. **Cierra el Monitor Serie**, desconecta el USB de la laptop y conecta ese ESP32 a la Pi (cable 2).

---

## 4. Preparar la Raspberry Pi (con internet, una sola vez)

La Pi y la laptop tienen que estar en la **misma red WiFi normal** (la de tu casa, por ejemplo).

### 4.1 Entrar a la Pi

**Dónde:** A (PowerShell).

```powershell
ssh fede@robo-pi-5.local
```

- La primera vez pregunta `Are you sure you want to continue connecting (yes/no...)?`: escribe `yes` y Enter.
- Después pide la contraseña de `fede` (no se ve al escribir).
- **Debes ver:** el prompt `fede@robo-pi-5:~ $`. **Desde aquí, todo es en la Pi (B).**
- Si dice `Could not resolve hostname`: la Pi no está en la misma red o todavía está arrancando. Espera un minuto y repite. Alternativa: abre una terminal de la Pi desde Raspberry Pi Connect en el navegador (mientras tenga internet) y sigue desde 4.2.

### 4.2 Instalar y revisar

**Dónde:** B (prompt `fede@robo-pi-5:~ $`).

```bash
cd ~
sudo apt update
sudo apt install -y python3-serial git curl
python3 -c "import serial; print('pyserial', serial.__version__)"
```

- **Debes ver:** al final, `pyserial 3.x`.

```bash
groups
```

- **Debes ver** la palabra `dialout` en la lista. Si no está:
  ```bash
  sudo usermod -aG dialout fede
  exit
  ```
  Con eso vuelves a la laptop (A). Vuelve a entrar con `ssh fede@robo-pi-5.local` y repite `groups`.

### 4.3 Bajar el código a la Pi

**Dónde:** B.

```bash
cd ~
ls
```

- Si en la lista **no** aparece `Robot_Movil_Autonomo`:
  ```bash
  git clone https://github.com/Quetrix/Robot_Movil_Autonomo.git
  ```
- Si **sí** aparece:
  ```bash
  cd ~/Robot_Movil_Autonomo
  git pull
  cd ~
  ```

Verifica:

```bash
cd ~/Robot_Movil_Autonomo
git log --oneline -1
ls Tarea_Integracion
```

- **Debes ver:** `Tarea 8: guia paso a paso y scripts revisar.sh / correr.sh`, y en la carpeta `braitenberg.py`, `correr.sh`, `revisar.sh`, etc.
- Si `git clone` pide usuario y contraseña, el repo es privado. Sal con Ctrl+C y cópialo desde la laptop: en A, `scp -r $HOME\Documents\Robot_Tarea8 fede@robo-pi-5.local:~/Robot_Movil_Autonomo`.

### 4.4 Probar el programa sin el robot

**Dónde:** B.

```bash
cd ~/Robot_Movil_Autonomo/Tarea_Integracion
python3 prueba_simulada.py
```

- Tarda unos 40 s. **Debes ver:** varias líneas `PASS` y al final `TODO OK`.

### 4.5 Ver el ESP32 de locomoción

Con el ESP32 conectado por USB a la Pi (cable 2). **Dónde:** B.

```bash
ls -l /dev/serial/by-id/
```

- **Debes ver** una línea como `usb-Silicon_Labs_CP2102_..._if00-port0 -> ../../ttyUSB0` (o `usb-1a86_USB_Serial-if00-port0`).
- Si dice `No such file or directory`: el cable es solo de carga o está flojo. Prueba otro cable u otro puerto USB.

### 4.6 Guardar la red WiFi del cabezal (no corta la conexión)

**Dónde:** B. Es un solo comando de 5 líneas: cópialo completo, con las barras `\`.

```bash
sudo nmcli con add type wifi ifname wlan0 con-name cabezal ssid "Percepcion-ESP32" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "percepcion26" \
  ipv4.method manual ipv4.addresses 192.168.4.200/24 ipv4.never-default yes \
  ipv6.method disabled 802-11-wireless.powersave 2 \
  connection.autoconnect yes connection.autoconnect-priority 10
```

- **Debes ver:** `Connection 'cabezal' (...) successfully added.`
- Si dice que ya existe, no pasa nada: ya estaba creada.

### 4.7 Revisión general

**Dónde:** B.

```bash
bash ~/Robot_Movil_Autonomo/Tarea_Integracion/revisar.sh
```

- **Debes ver** `[OK]` en dialout, python3-serial, código, ESP32 y perfil `cabezal`. Lo del cabezal se omite porque todavía estás en tu red normal.

---

## 5. Pasar la Pi a la red del cabezal

1. Enciende el **cabezal** (power bank B) y espera unos 5 s.
2. Pasa la Pi a esa red. Elige **una** opción:
   - **Si sigues conectado por SSH (B):** escribe estas dos líneas, una por una. La primera puede pedir la contraseña.
     ```bash
     sudo -v
     sudo nohup nmcli con up cabezal >/dev/null 2>&1 &
     ```
     A los pocos segundos la ventana se congela o dice `Connection closed`. **Es lo esperado:** la Pi se cambió de red. Si queda congelada, cierra esa ventana y abre otra PowerShell.
   - **Si la Pi está apagada:** enciéndela (power bank A) y espera unos 40 s. Se conecta sola a la red del cabezal.
3. En la **laptop**, conéctate al WiFi **`Percepcion-ESP32`** (clave `percepcion26`). Windows dirá "Sin internet": es normal.
4. **Dónde:** A (PowerShell nueva):
   ```powershell
   ping 192.168.4.200
   ```
   - **Debes ver:** `Respuesta desde 192.168.4.200: bytes=32 ...`.
   - Si sale `Tiempo de espera agotado` 4 veces: este enlace no funciona. Usa el **respaldo B** de `README.md` (sección 7): cabezal por USB a la Pi, y la Pi y la laptop en tu red normal.
5. Entra a la Pi por la nueva dirección:
   ```powershell
   ssh fede@192.168.4.200
   ```
   Puede preguntar otra vez `yes/no`: escribe `yes`. Contraseña. **Debes ver:** `fede@robo-pi-5:~ $`.
6. **Dónde:** B:
   ```bash
   bash ~/Robot_Movil_Autonomo/Tarea_Integracion/revisar.sh
   ```
   - **Debes ver** además: `WiFi actual: Percepcion-ESP32`, `[OK] IP 192.168.4.200` y `[OK] el cabezal responde`.

No abras en la laptop la página `http://192.168.4.1` mientras corre el robot.

---

## 6. Correr el robot

1. **Robot levantado** sobre una caja, con las ruedas en el aire.
2. Conecta la **batería de motores** (cables 8 y 9).
3. **Dónde:** B:
   ```bash
   bash ~/Robot_Movil_Autonomo/Tarea_Integracion/correr.sh
   ```
   - **Debes ver:** `Locomocion: /dev/serial/by-id/...`, `Abriendo puertos (2.5 s)...` y después el panel. En la parte de abajo del panel debe decir `percepcion: OK` y `firmware loco: ir_frontal`.
   - Si los Sharp salen intercambiados en la prueba de la mano: sal con `q` y corre `bash ~/Robot_Movil_Autonomo/Tarea_Integracion/correr.sh --invertir-sharp`.
4. Teclas, **con la ventana de PowerShell seleccionada**: `1` Miedo, `2` Agresión, `3` Amor, `4` Explorador, `0` o espacio para esperar, **`q` para salir**.
5. Haz el checklist en banco (`README.md`, sección 8.1). Luego pulsa `0`, baja el robot al suelo y sigue con la sección 8.2 y el `guion_video.md`.

Cada corrida deja un registro `corrida_HHMMSS.csv` en `~/Robot_Movil_Autonomo/Tarea_Integracion/`.

---

## 7. Al terminar

1. En el panel, `q`. **Debes ver:** `Detenido (se envio X).`
2. Desconecta la **batería de motores**.
3. **Dónde:** B: `sudo poweroff`. Espera unos 10 s, hasta que el LED de la Pi deje de parpadear, y desconecta el power bank A.
4. Apaga el cabezal. La próxima vez que enciendas la Pi sin el cabezal, vuelve sola a tu red normal.

Para traer los CSV a la laptop, mientras sigues en la red del cabezal (en A, no en B):

```powershell
scp fede@192.168.4.200:~/Robot_Movil_Autonomo/Tarea_Integracion/corrida_*.csv $HOME\Documents\
```

---

## 8. Resumen para la próxima vez (todo ya configurado)

| Paso | Dónde | Qué |
|---|---|---|
| 1 | — | robot levantado; encender cabezal y luego Pi; esperar 40 s |
| 2 | laptop | WiFi `Percepcion-ESP32` |
| 3 | A | `ssh fede@192.168.4.200` |
| 4 | — | conectar la batería de motores |
| 5 | B | `bash ~/Robot_Movil_Autonomo/Tarea_Integracion/correr.sh` |

---

## 9. Errores comunes de terminal

| Mensaje | Significa | Solución |
|---|---|---|
| `No such file or directory` | estás en otra carpeta o el nombre está mal | `pwd` para ver dónde estás; usa la ruta completa con `~/...` y la tecla Tab |
| `command not found` | lo escribiste en la terminal equivocada (A en vez de B) o falta instalarlo | mira el prompt (sección 0) |
| `Permission denied (publickey,password)` | contraseña incorrecta | repite con cuidado (no se ve al escribir) |
| `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED` | la laptop recuerda otra Pi con esa dirección | en A: `ssh-keygen -R 192.168.4.200` (o `robo-pi-5.local`) y vuelve a entrar |
| `Permission denied: '/dev/ttyUSB0'` | falta el grupo dialout | sección 4.2 |
| `Hay mas de un ESP32 conectado` | el cabezal también está por USB | ese mismo mensaje muestra el comando a usar |
| Las teclas no hacen nada | la ventana no está seleccionada, o estás en el prompt y no en el panel | haz clic en la ventana; si ves el prompt, el programa no está corriendo |
| La ventana se congeló | se cayó el SSH (el robot frena solo en 0.3 s) | cierra la ventana, abre otra PowerShell y vuelve a hacer `ssh` |
