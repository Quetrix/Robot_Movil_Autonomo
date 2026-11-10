#!/usr/bin/env python3
"""
MT9005 - Tarea 8: vehiculos de Braitenberg en la Raspberry Pi 5.

Arquitectura base (se reutiliza en el proyecto final):

  ESP32 cabezal --WiFi GET /data (o USB DETAIL)--> Raspberry Pi --USB "H 0 v w" a 10 Hz--> ESP32 locomocion
  Sharp izq/der + TF-Luna                           |                                       PI por rueda + IR frontal
                                                    v
        lectura -> estimulos sI, sD -> maquina de estados -> comportamiento (vI, vD)
        -> diferencial virtual (v, w) -> capa de seguridad -> comando H

Firmware esperado:
  locomocion: Avance_1_Locomocion/locomocion_esp32_ir_frontal
  cabezal:    Avance_2_Percepcion/Percepcion_MG996_180 (queda fijo en CENTER)

Seguridad:
  - Se envia "H" (no "V") a 10 Hz: si el script, la Pi o el USB fallan, el ESP32 frena solo
    a los 300 ms.
  - Sin capturas nuevas de percepcion durante 1 s -> estado PARO.
  - TF-Luna < 250 mm (o "menos de 20 cm") -> se anula el avance (v <= 0); se libera > 350 mm.
    En 2a y 3b, ademas, gira en el lugar hacia el lado mas libre: los Sharp miran a +-60 grados
    y no ven lo que esta justo enfrente.
  - IR frontal KY-032: el firmware anula el avance por su cuenta; aqui se muestra y se trata
    igual que el TF.

Teclas (en la terminal SSH):
  1 = 2a Miedo   2 = 2b Agresion   3 = 3a Amor   4 = 3b Explorador
  0 o espacio = ESPERA (frena)     q = salir (envia X)
No usar el boton fisico: arranca la secuencia de demostracion y bloquea los comandos.

Uso:
  python3 braitenberg.py --loco /dev/serial/by-id/<locomocion>             (cabezal por WiFi)
  python3 braitenberg.py --loco /dev/serial/by-id/<locomocion> --perc /dev/serial/by-id/<cabezal>
  --log corrida.csv     guarda un registro a 10 Hz
  --sin-pantalla        imprime lineas en vez del panel (util si la terminal no soporta ANSI)
"""
import argparse
import json
import math
import re
import select
import signal
import sys
import threading
import time
import urllib.parse
import urllib.request

try:
    import serial
except ImportError:
    sys.exit("Falta pyserial: sudo apt install -y python3-serial")

# ---------------- Parametros (ajustar en pruebas) ----------------
D_MIN_MM = 100.0       # a esta distancia o menos el estimulo vale 1 (limite inferior del Sharp)
D_MAX_MM = 600.0       # mas alla el estimulo vale 0
IR_V_MUY_CERCA = 2.3   # Sharp fuera de rango con mas voltaje que esto = mas cerca de 100 mm
                       # (IR_PRELIM_NEAR_V de Config.h del cabezal)
TAU_ESTIMULO_S = 0.0   # memoria del estimulo: s = max(s_nuevo, s_anterior*exp(-dt/tau)).
                       # El rayo del Sharp es angosto y "pierde" el obstaculo al girar; 0 = sin memoria
V0 = 0.20              # velocidad base de cada rueda virtual [m/s] (~80 RPM)
K_EXC = 0.15           # ganancia excitatoria (2a, 2b) [m/s por unidad de estimulo]
B_VIRTUAL = 0.20       # trocha virtual [m]: mayor = giros mas suaves
V_PARO = 0.04          # si ambas ruedas virtuales quedan por debajo, se frena
TF_PARO_MM = 250.0     # TF por debajo de esto: se anula el avance
TF_LIBRE_MM = 350.0    # el bloqueo por TF se libera por encima de esto (histeresis)
W_GIRO_BLOQUEO = 1.0   # giro en el lugar cuando 2a/3b quedan bloqueados al frente [rad/s]
T_SENSOR_MS = 200      # periodo de consulta al cabezal
T_CMD_MS = 100         # periodo de envio de H (10 Hz; el firmware frena si pasan 300 ms)
T_PANTALLA_MS = 200    # refresco del panel
T_TIMEOUT_S = 1.0      # sin capturas nuevas -> PARO

SENSORES = ("IR_I", "IR_D", "TF")

TECLAS = {
    "0": "ESPERA",
    " ": "ESPERA",
    "1": "MIEDO_2A",
    "2": "AGRESION_2B",
    "3": "AMOR_3A",
    "4": "EXPLORADOR_3B",
}
DESCRIPCION = {
    "ESPERA": "detenido, esperando tecla",
    "PARO": "SIN DATOS DE PERCEPCION: detenido",
    "MIEDO_2A": "2a MIEDO       conexion directa, excitatoria",
    "AGRESION_2B": "2b AGRESION    conexion cruzada, excitatoria",
    "AMOR_3A": "3a AMOR        conexion directa, inhibitoria",
    "EXPLORADOR_3B": "3b EXPLORADOR  conexion cruzada, inhibitoria",
}
GIRO_SI_BLOQUEADO = ("MIEDO_2A", "EXPLORADOR_3B")


# ---------------- Sensores ----------------
def estimulo(d_mm, muy_cerca=False):
    """Distancia -> estimulo normalizado [0, 1] (1 = muy cerca). Sin lectura = 0."""
    if muy_cerca:
        return 1.0
    if d_mm is None:
        return 0.0
    s = (D_MAX_MM - d_mm) / (D_MAX_MM - D_MIN_MM)
    return min(1.0, max(0.0, s))


def muy_cerca(sensor, estado, volts=None):
    """Lectura invalida porque el objeto esta por debajo del rango minimo del sensor."""
    if sensor == "TF":
        return estado == "TF_menos_de_20cm_no_fiable"
    return estado == "IR_fuera_de_rango" and volts is not None and volts >= IR_V_MUY_CERCA


def a_float(texto):
    try:
        v = float(texto.replace("~", ""))
        return v if math.isfinite(v) else None
    except ValueError:
        return None


def abrir_serial(puerto):
    """Abre el puerto sin forzar DTR/RTS (igual que teleop_locomocion.py)."""
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = puerto, 115200, 0
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class PercepcionWiFi:
    """Cabezal por su WiFi propio: GET http://192.168.4.1/data en un hilo aparte."""

    def __init__(self, url):
        self.url = url.rstrip("/")
        self.d = dict.fromkeys(SENSORES)
        self.cerca = dict.fromkeys(SENSORES, False)
        self.t_dato = 0.0
        self.seq = None
        self.cabezal = "?"
        self.error = ""
        self._t_center = 0.0
        self._lock = threading.Lock()

    def _post(self, cmd):
        datos = urllib.parse.urlencode({"cmd": cmd}).encode()
        r = urllib.request.urlopen(self.url + "/action", data=datos, timeout=2)
        return r.read().decode(errors="ignore")

    def _center(self):
        self._t_center = time.monotonic()
        try:
            return self._post("CENTER")
        except Exception as e:
            return f"no se pudo enviar CENTER ({e})"

    def iniciar(self):
        print(f"Cabezal {self.url}: CENTER -> {self._center()}")
        threading.Thread(target=self._hilo, daemon=True).start()

    def _hilo(self):
        while True:
            t0 = time.monotonic()
            try:
                r = urllib.request.urlopen(self.url + "/data", timeout=1)
                self._procesar(json.loads(r.read()))
                self.error = ""
            except Exception as e:
                self.error = type(e).__name__     # sin dato: fresco() vence y la FSM pasa a PARO
            time.sleep(max(0.0, T_SENSOR_MS / 1000 - (time.monotonic() - t0)))

    def _procesar(self, j):
        muestras = j["samples"]
        estado = j.get("sensor_status") or [""] * 3
        volts = j.get("ir_volts") or [None, None]
        d, cerca = {}, {}
        for i, k in enumerate(SENSORES):
            s = muestras[i]
            d[k] = s.get("mm") if s.get("valid") else None
            cerca[k] = d[k] is None and muy_cerca(k, estado[i], volts[i] if i < 2 else None)
        with self._lock:
            self.d, self.cerca = d, cerca
        if j.get("seq") != self.seq:              # captura nueva del cabezal
            self.seq = j.get("seq")
            self.t_dato = time.monotonic()
        unido = bool(j.get("attached"))
        self.cabezal = f"{j.get('head_command_deg')} deg {j.get('mode')}" if unido else "servo sin senal"
        if not unido and time.monotonic() - self._t_center > 3.0:
            self._center()                        # el cabezal se reinicio: volver a centrar

    def pedir(self):
        pass

    def leer(self):
        pass

    def lectura(self):
        with self._lock:
            return dict(self.d), dict(self.cerca)

    def fresco(self):
        return time.monotonic() - self.t_dato < T_TIMEOUT_S


class PercepcionSerial:
    """Respaldo: cabezal por USB. Pide DETAIL y lee la tabla por sensor."""

    def __init__(self, puerto):
        self.ser = abrir_serial(puerto)
        self.buf = ""
        self.en_tabla = False
        self.d = dict.fromkeys(SENSORES)
        self.cerca = dict.fromkeys(SENSORES, False)
        self._tmp, self._estado = {}, {}
        self.t_dato = 0.0
        self.seq = None
        self._captura = None
        self.cabezal = "?"
        self.error = ""
        self._t_center = 0.0

    def iniciar(self):
        time.sleep(2.5)                      # el ESP32 puede reiniciarse al abrir el puerto
        self.ser.reset_input_buffer()
        self._center()

    def _center(self):
        self._t_center = time.monotonic()
        self.ser.write(b"CENTER\n")

    def pedir(self):
        self.ser.write(b"DETAIL\n")

    def leer(self):
        self.buf += self.ser.read(4096).decode(errors="ignore")
        *lineas, self.buf = self.buf.split("\n")
        for ln in lineas:
            ln = ln.strip()
            m = re.match(r"# Captura (\d+) .*cabezal_cmd=(\S+)deg modo=(\S+)", ln)
            if m:
                self._captura = int(m.group(1))
                self.cabezal = f"{m.group(2)} deg {m.group(3)}"
                if m.group(2) == "--" and time.monotonic() - self._t_center > 3.0:
                    self._center()
                continue
            if ln.startswith("sensor;head_cmd_deg"):
                self.en_tabla, self._tmp, self._estado = True, {}, {}
                continue
            if ln.startswith("IR_volts="):          # ultima linea de DETAIL: se confirma la captura
                try:
                    volts = [a_float(x) for x in ln.split()[0][9:].split(",")]
                except Exception:
                    volts = [None, None]
                self._confirmar(volts)
                continue
            if ln.startswith("FUENTES") or ln.startswith("#") or not ln:
                self.en_tabla = False
                continue
            if self.en_tabla:
                c = ln.split(";")
                if len(c) >= 9 and c[0] in SENSORES:
                    self._tmp[c[0]] = a_float(c[3])    # d_sensor_mm
                    self._estado[c[0]] = c[8]

    def _confirmar(self, volts):
        if len(self._tmp) < 3:
            return
        for i, k in enumerate(SENSORES):
            self.d[k] = self._tmp[k]
            v = volts[i] if i < 2 and i < len(volts) else None
            self.cerca[k] = self.d[k] is None and muy_cerca(k, self._estado.get(k, ""), v)
        if self._captura != self.seq:
            self.seq = self._captura
            self.t_dato = time.monotonic()

    def lectura(self):
        return dict(self.d), dict(self.cerca)

    def fresco(self):
        return time.monotonic() - self.t_dato < T_TIMEOUT_S


# ---------------- Actuadores ----------------
RUIDO = ("H MANTENER", "V MANUAL", "FRENADO PI", "Use:", "V vx vy", "IR frontal (GPIO34)", "DURACION")


class Locomocion:
    def __init__(self, puerto):
        self.ser = abrir_serial(puerto)
        self.buf = ""
        self.ir_frente = False
        self.firmware_ir = None          # True = locomocion_esp32_ir_frontal
        self._t_ayuda = None
        self.ultimo_cmd = "-"

    def enviar(self, texto):
        self.ser.write((texto + "\n").encode())

    def iniciar(self):
        time.sleep(2.5)
        self.ser.reset_input_buffer()
        self.enviar("X")
        self.enviar("?")                 # la ayuda del firmware ir_frontal menciona el IR

    def mover(self, v, w):
        """v adelante [m/s], w antihorario [rad/s]. vx = 0: diferencial virtual."""
        self.ultimo_cmd = f"H 0 {v:.3f} {w:.3f}"
        self.enviar(self.ultimo_cmd)

    def frenar(self):
        self.ultimo_cmd = "V 0 0 0"
        self.enviar(self.ultimo_cmd)

    def parar(self):
        self.ultimo_cmd = "X"
        self.enviar("X")

    def leer(self):
        """Procesa lo que llega del ESP32 y devuelve los mensajes que vale la pena mostrar."""
        self.buf += self.ser.read(4096).decode(errors="ignore")
        *lineas, self.buf = self.buf.split("\n")
        mostrar = []
        for ln in (x.strip() for x in lineas):
            if not ln:
                continue
            if ln[0].isdigit():                       # telemetria CSV
                c = ln.split(",")
                if len(c) == 17:                      # ir_frontal: ...,theta_deg,irF,seq,pausa
                    self.firmware_ir = True
                    self.ir_frente = c[14] == "1"
                elif len(c) == 16:
                    self.firmware_ir = False
                continue
            if "IR FRENTE: OBSTACULO" in ln:
                self.ir_frente = True
            elif "IR FRENTE: LIBRE" in ln:
                self.ir_frente = False
            if "IR frontal" in ln or "IR FRONTAL" in ln or "IR FRENTE" in ln:
                self.firmware_ir = True
            if "Use: W S A D" in ln:
                self._t_ayuda = time.monotonic()
            if ln.startswith("#") and not any(r in ln for r in RUIDO):
                mostrar.append(ln)
        if (self.firmware_ir is None and self._t_ayuda is not None
                and time.monotonic() - self._t_ayuda > 0.5):
            self.firmware_ir = False
            mostrar.append("# AVISO: el ESP32 no tiene el firmware ir_frontal")
        return mostrar


# ---------------- Comportamientos de Braitenberg ----------------
# sI, sD = estimulo izquierdo/derecho. Devuelven (vI, vD) de las ruedas virtuales [m/s].
def miedo_2a(sI, sD):       # directa, excitatoria: el lado estimulado acelera -> se aleja
    return V0 + K_EXC * sI, V0 + K_EXC * sD


def agresion_2b(sI, sD):    # cruzada, excitatoria: acelera el lado opuesto -> va hacia el estimulo
    return V0 + K_EXC * sD, V0 + K_EXC * sI


def amor_3a(sI, sD):        # directa, inhibitoria: frena el lado estimulado -> gira hacia el y se detiene
    return V0 * (1 - sI), V0 * (1 - sD)


def explorador_3b(sI, sD):  # cruzada, inhibitoria: frena el lado opuesto -> desacelera y se aparta
    return V0 * (1 - sD), V0 * (1 - sI)


COMPORTAMIENTOS = {
    "MIEDO_2A": miedo_2a,
    "AGRESION_2B": agresion_2b,
    "AMOR_3A": amor_3a,
    "EXPLORADOR_3B": explorador_3b,
}


def diferencial_a_robot(vI, vD):
    """Ruedas virtuales -> (v adelante [m/s], w antihorario+ [rad/s]).
    vD > vI hace girar a la izquierda, que es antihorario: w > 0, igual que omega del firmware."""
    return (vI + vD) / 2.0, (vD - vI) / B_VIRTUAL


# ---------------- Maquina de estados ----------------
class Robot:
    def __init__(self, loco, perc, eventos):
        self.loco, self.perc, self.eventos = loco, perc, eventos
        self.estado = "ESPERA"
        self.tf_bloqueo = False
        self.dir_bloqueo = None
        self.ultimo_giro = 1.0
        self.s_mem = [0.0, 0.0]
        self.t_paso = time.monotonic()
        self.t = dict(sI=0.0, sD=0.0, vI=0.0, vD=0.0, v=0.0, w=0.0, seguridad="")

    def _memoria(self, sI, sD):
        """Retencion con decaimiento exponencial del estimulo de cada Sharp."""
        ahora = time.monotonic()
        dt, self.t_paso = ahora - self.t_paso, ahora
        k = math.exp(-dt / TAU_ESTIMULO_S) if TAU_ESTIMULO_S > 0 else 0.0
        self.s_mem = [max(sI, self.s_mem[0] * k), max(sD, self.s_mem[1] * k)]
        return self.s_mem

    def transicion(self, nuevo, motivo=""):
        if nuevo == self.estado:
            return
        self.eventos.agregar(f"{self.estado} -> {nuevo} {motivo}")
        self.estado = nuevo
        self.dir_bloqueo = None
        if nuevo in ("ESPERA", "PARO"):
            self.loco.frenar()

    def evento_tecla(self, k):
        if k in TECLAS:
            self.transicion(TECLAS[k], "(teclado)")

    def _actualizar_tf(self, d_tf, cerca_tf):
        if cerca_tf or (d_tf is not None and d_tf < TF_PARO_MM):
            self.tf_bloqueo = True
        elif d_tf is None or d_tf > TF_LIBRE_MM:
            self.tf_bloqueo = False

    def paso(self):
        # Guardas de seguridad: sin percepcion no hay comportamiento.
        if self.estado in COMPORTAMIENTOS and not self.perc.fresco():
            self.transicion("PARO", "(sin datos de percepcion durante 1 s)")
        elif self.estado == "PARO" and self.perc.fresco():
            self.transicion("ESPERA", "(percepcion recuperada)")

        d, cerca = self.perc.lectura()
        sI, sD = self._memoria(estimulo(d["IR_I"], cerca["IR_I"]), estimulo(d["IR_D"], cerca["IR_D"]))
        self._actualizar_tf(d["TF"], cerca["TF"])
        if self.estado not in COMPORTAMIENTOS:
            self.t = dict(sI=sI, sD=sD, vI=0.0, vD=0.0, v=0.0, w=0.0, seguridad="")
            return

        vI, vD = COMPORTAMIENTOS[self.estado](sI, sD)
        v, w = diferencial_a_robot(vI, vD)
        seguridad = ""
        if max(vI, vD) < V_PARO:
            v = w = 0.0
            seguridad = "ruedas virtuales < 0.04 m/s: frena"

        bloqueo = self.tf_bloqueo or self.loco.ir_frente
        if bloqueo:
            motivo = "TF < 250 mm" if self.tf_bloqueo else "IR frente"
            if v > 0:
                v = 0.0
                seguridad = f"{motivo}: avance anulado"
            if self.estado in GIRO_SI_BLOQUEADO and abs(w) < W_GIRO_BLOQUEO:
                if self.dir_bloqueo is None:         # se decide al bloquearse y no cambia
                    if abs(w) > 0.05:
                        self.dir_bloqueo = math.copysign(1.0, w)
                    elif sI != sD:
                        self.dir_bloqueo = -1.0 if sI > sD else 1.0   # hacia el lado mas libre
                    else:
                        self.dir_bloqueo = self.ultimo_giro
                w = self.dir_bloqueo * W_GIRO_BLOQUEO
                seguridad = f"{motivo}: avance anulado + giro {'izq' if w > 0 else 'der'}"
        else:
            self.dir_bloqueo = None
        if abs(w) > 0.05:
            self.ultimo_giro = math.copysign(1.0, w)

        self.loco.mover(v, w)
        self.t = dict(sI=sI, sD=sD, vI=vI, vD=vD, v=v, w=w, seguridad=seguridad)


# ---------------- Interfaz ----------------
class Eventos:
    def __init__(self, t0, n=7, lineas=False):
        self.t0, self.n, self.lineas, self.lista = t0, n, lineas, []

    def agregar(self, texto):
        linea = f"{time.monotonic() - self.t0:6.1f} s  {texto}"
        self.lista = (self.lista + [linea])[-self.n:]
        if self.lineas:
            print(linea, flush=True)


def fmt_mm(d, cerca):
    if cerca:
        return " <min"
    return "   --" if d is None else f"{d:5.0f}"


def barra(s, n=20):
    k = int(round(s * n))
    return "#" * k + "." * (n - k)


def panel(robot, perc, loco, eventos, t):
    d, cerca = perc.lectura()
    x = robot.t
    fw = {True: "ir_frontal", False: "SIN IR FRONTAL", None: "?"}[loco.firmware_ir]
    enlace = "OK" if perc.fresco() else f"SIN DATOS {perc.error}"
    sep = "-" * 74
    tf = d["TF"]
    tf_txt = fmt_mm(tf, cerca["TF"])
    lineas = [
        f" MT9005 Tarea 8 - Vehiculos de Braitenberg                 t = {t:7.1f} s",
        sep,
        f" ESTADO: {robot.estado:14s} {DESCRIPCION[robot.estado]}",
        sep,
        "  sensor           dist [mm]   estimulo",
        f"  Sharp izq +60     {fmt_mm(d['IR_I'], cerca['IR_I'])}      sI = {x['sI']:.2f}  {barra(x['sI'])}",
        f"  Sharp der -60     {fmt_mm(d['IR_D'], cerca['IR_D'])}      sD = {x['sD']:.2f}  {barra(x['sD'])}",
        f"  TF-Luna 0         {tf_txt}      {'BLOQUEO (< 250 mm)' if robot.tf_bloqueo else 'libre'}",
        f"  IR frente KY-032  {'OBSTACULO' if loco.ir_frente else 'libre'}",
        sep,
        f"  ruedas virtuales  vI = {x['vI']:+.3f}   vD = {x['vD']:+.3f}  m/s",
        f"  robot             v  = {x['v']:+.3f} m/s   w = {x['w']:+.2f} rad/s (antihorario +)",
        f"  comando           {loco.ultimo_cmd}",
        f"  seguridad         {x['seguridad'] or '-'}",
        sep,
        f"  cabezal: {perc.cabezal}   percepcion: {enlace}   firmware loco: {fw}",
        "  1 Miedo 2a | 2 Agresion 2b | 3 Amor 3a | 4 Explorador 3b | 0/espacio Espera | q Salir",
        sep,
    ] + ["  " + e for e in eventos.lista]
    return "\033[H" + "".join(ln + "\033[K\n" for ln in lineas) + "\033[J"


def linea_simple(robot, perc, loco):
    d, cerca = perc.lectura()
    x = robot.t
    return (f"{robot.estado:14s} I={fmt_mm(d['IR_I'], cerca['IR_I'])} D={fmt_mm(d['IR_D'], cerca['IR_D'])} "
            f"TF={fmt_mm(d['TF'], cerca['TF'])} irF={int(loco.ir_frente)} sI={x['sI']:.2f} sD={x['sD']:.2f} "
            f"v={x['v']:+.3f} w={x['w']:+.2f} {x['seguridad']}")


class Teclado:
    def __init__(self):
        self.tty = sys.stdin.isatty()
        self.eof = False
        self.viejo = None
        if self.tty:
            import termios
            import tty
            self.viejo = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def leer(self):
        if self.eof or not select.select([sys.stdin], [], [], 0)[0]:
            return None
        k = sys.stdin.read(1)
        if k == "":
            self.eof = True
            return None
        return k

    def restaurar(self):
        if self.viejo is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.viejo)


def salir_por_senal(signum, frame):
    raise SystemExit(f"senal {signum}")


def main():
    ap = argparse.ArgumentParser(description="Vehiculos de Braitenberg (MT9005, Tarea 8)")
    ap.add_argument("--loco", default="/dev/ttyUSB0", help="puerto serial del ESP32 de locomocion")
    ap.add_argument("--perc", default="http://192.168.4.1",
                    help="URL http:// del WiFi del cabezal, o puerto serial si va por USB")
    ap.add_argument("--log", help="archivo CSV de registro")
    ap.add_argument("--sin-pantalla", action="store_true", help="lineas simples en vez del panel")
    a = ap.parse_args()

    signal.signal(signal.SIGTERM, salir_por_senal)
    signal.signal(signal.SIGHUP, salir_por_senal)

    perc = PercepcionWiFi(a.perc) if a.perc.startswith("http") else PercepcionSerial(a.perc)
    loco = Locomocion(a.loco)
    t0 = time.monotonic()
    eventos = Eventos(t0, lineas=a.sin_pantalla)
    print("Abriendo puertos (2.5 s)...")
    loco.iniciar()
    perc.iniciar()
    robot = Robot(loco, perc, eventos)
    log = open(a.log, "w") if a.log else None
    if log:
        log.write("t,estado,IR_I,IR_D,TF,sI,sD,vI,vD,v,w,irF,tf_bloqueo,comando\n")

    teclado = Teclado()
    if not a.sin_pantalla:
        sys.stdout.write("\033[2J")
    eventos.agregar("listo: 1=Miedo 2=Agresion 3=Amor 4=Explorador 0/espacio=Espera q=salir")
    t_sens = t_cmd = t_pant = t_lin = 0.0
    try:
        while True:
            ahora = time.monotonic()
            k = teclado.leer()
            if k == "q":
                break
            if k:
                robot.evento_tecla(k)
            if ahora - t_sens >= T_SENSOR_MS / 1000:
                perc.pedir()
                t_sens = ahora
            perc.leer()
            for msg in loco.leer():
                eventos.agregar(f"[loco] {msg}")
            if ahora - t_cmd >= T_CMD_MS / 1000:
                robot.paso()
                t_cmd = ahora
                if log:
                    d, _ = perc.lectura()
                    x = robot.t
                    log.write(f"{ahora - t0:.2f},{robot.estado},{d['IR_I']},{d['IR_D']},{d['TF']},"
                              f"{x['sI']:.3f},{x['sD']:.3f},{x['vI']:.3f},{x['vD']:.3f},{x['v']:.3f},"
                              f"{x['w']:.3f},{int(loco.ir_frente)},{int(robot.tf_bloqueo)},{loco.ultimo_cmd}\n")
            if a.sin_pantalla:
                if ahora - t_lin >= 1.0:
                    print(linea_simple(robot, perc, loco), flush=True)
                    t_lin = ahora
            elif ahora - t_pant >= T_PANTALLA_MS / 1000:
                sys.stdout.write(panel(robot, perc, loco, eventos, ahora - t0))
                sys.stdout.flush()
                t_pant = ahora
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        loco.parar()
        time.sleep(0.1)
        teclado.restaurar()
        if log:
            log.close()
        print("\nDetenido (se envio X).")


if __name__ == "__main__":
    main()
