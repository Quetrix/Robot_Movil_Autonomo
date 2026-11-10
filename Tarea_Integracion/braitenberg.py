#!/usr/bin/env python3
"""
MT9005 - Tarea 8: comportamientos de Braitenberg (Raspberry Pi 5).

Arquitectura base (sirve para el proyecto final):

    Percepcion ESP32 --USB--> [Pi: sensores -> maquina de estados -> comportamiento] --USB--> Locomocion ESP32
         DETAIL (IR_I, IR_D, TF)                                                          "V vx vy omega"

- Percepcion: cabezal fijo al frente (CENTER). Los dos Sharp (+60 deg izq, -60 deg der) son
  los dos "sensores" del vehiculo de Braitenberg. Se piden con DETAIL a ~5 Hz.
- Locomocion: se usa el comando "V vx vy omega" del firmware del Avance 1
  (vx derecha+, vy adelante+, omega antihorario+; m/s y rad/s). vx = 0 siempre:
  el robot se comporta como diferencial virtual con ruedas izquierda/derecha.
- Seguridad: los KY-032 siguen frenando el robot desde el ESP32 de locomocion; si la
  percepcion deja de responder, la maquina de estados pasa a PARO.

Teclas (en la terminal SSH):
    1 = 2a Miedo   2 = 2b Agresion   3 = 3a Amor   4 = 3b Explorador
    0 / espacio = ESPERA (detener)    q = salir
NO presione el boton fisico: en el firmware del Avance 1 inicia la secuencia de demo.

Uso:
    python3 braitenberg.py --loco /dev/serial/by-id/<locomocion> --perc /dev/serial/by-id/<percepcion>
"""
import argparse
import math
import select
import sys
import termios
import time
import tty

import serial

# ---------------- Parametros (ajustar en pruebas) ----------------
D_MIN_MM = 100.0     # a esta distancia o menos el estimulo es 1 (limite inferior Sharp)
D_MAX_MM = 600.0     # mas alla el estimulo es 0
V0 = 0.20            # velocidad base de cada rueda virtual [m/s] (~80 RPM)
K_EXC = 0.15         # ganancia excitatoria (2a, 2b) [m/s por unidad de estimulo]
B_VIRTUAL = 0.20     # trocha virtual [m]: mayor = giros mas suaves
V_PARO = 0.04        # si ambas ruedas virtuales quedan por debajo, se manda V 0 0 0
T_SENSOR_MS = 200    # periodo de consulta DETAIL
T_CMD_MS = 200       # periodo de envio de V
T_TIMEOUT_S = 1.0    # sin datos de percepcion -> PARO

ESTADOS = {
    "0": "ESPERA",
    "1": "MIEDO_2A",
    "2": "AGRESION_2B",
    "3": "AMOR_3A",
    "4": "EXPLORADOR_3B",
}


# ---------------- Sensores ----------------
def estimulo(d_mm):
    """Distancia -> estimulo normalizado [0, 1] (1 = muy cerca). Sin lectura = 0."""
    if d_mm is None:
        return 0.0
    s = (D_MAX_MM - d_mm) / (D_MAX_MM - D_MIN_MM)
    return min(1.0, max(0.0, s))


def a_float(texto):
    try:
        v = float(texto.replace("~", ""))
        return v if math.isfinite(v) else None
    except ValueError:
        return None


class Percepcion:
    """Lee la tabla DETAIL del firmware Percepcion_MG996_180."""

    def __init__(self, puerto):
        self.ser = serial.Serial(puerto, 115200, timeout=0)
        self.buf = ""
        self.en_tabla = False
        self.d = {"IR_I": None, "IR_D": None, "TF": None}
        self.t_dato = 0.0

    def iniciar(self):
        time.sleep(2.5)                      # el ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self.ser.write(b"CENTER\n")          # cabezal fijo al frente

    def pedir(self):
        self.ser.write(b"DETAIL\n")

    def leer(self):
        self.buf += self.ser.read(4096).decode(errors="ignore")
        *lineas, self.buf = self.buf.split("\n")
        for ln in lineas:
            ln = ln.strip()
            if ln.startswith("sensor;head_cmd_deg"):
                self.en_tabla = True          # inicio de la primera tabla de DETAIL
                continue
            if ln.startswith("FUENTES") or ln.startswith("#") or not ln:
                self.en_tabla = False
                continue
            if self.en_tabla:
                c = ln.split(";")
                if len(c) >= 4 and c[0] in self.d:
                    self.d[c[0]] = a_float(c[3])   # d_sensor_mm
                    if c[0] == "TF":
                        self.t_dato = time.monotonic()

    def fresco(self):
        return time.monotonic() - self.t_dato < T_TIMEOUT_S


# ---------------- Actuadores ----------------
class Locomocion:
    def __init__(self, puerto):
        self.ser = serial.Serial(puerto, 115200, timeout=0)
        self.buf = ""

    def iniciar(self):
        time.sleep(2.5)
        self.ser.reset_input_buffer()
        self.ser.write(b"X\n")

    def mover(self, v, w):
        """v adelante [m/s], w antihorario [rad/s]."""
        self.ser.write(f"V 0 {v:.3f} {w:.3f}\n".encode())

    def frenar(self):
        self.ser.write(b"V 0 0 0\n")

    def parar(self):
        self.ser.write(b"X\n")

    def leer(self):
        """Devuelve los mensajes '#' del firmware (obstaculo, bloqueos) para mostrarlos."""
        self.buf += self.ser.read(4096).decode(errors="ignore")
        *lineas, self.buf = self.buf.split("\n")
        return [ln.strip() for ln in lineas if ln.startswith("# ") and "V MANUAL" not in ln]


# ---------------- Comportamientos de Braitenberg ----------------
# sI, sD = estimulo izquierdo/derecho. Devuelven (vI, vD) de las ruedas virtuales.
def miedo_2a(sI, sD):       # excitatorio directo: se aleja y acelera
    return V0 + K_EXC * sI, V0 + K_EXC * sD


def agresion_2b(sI, sD):    # excitatorio cruzado: gira hacia el estimulo y acelera
    return V0 + K_EXC * sD, V0 + K_EXC * sI


def amor_3a(sI, sD):        # inhibitorio directo: gira hacia el estimulo y se detiene frente a el
    return V0 * (1 - sI), V0 * (1 - sD)


def explorador_3b(sI, sD):  # inhibitorio cruzado: frena cerca y gira alejandose
    return V0 * (1 - sD), V0 * (1 - sI)


COMPORTAMIENTOS = {
    "MIEDO_2A": miedo_2a,
    "AGRESION_2B": agresion_2b,
    "AMOR_3A": amor_3a,
    "EXPLORADOR_3B": explorador_3b,
}


def diferencial_a_robot(vI, vD):
    """Ruedas virtuales -> (v adelante, omega antihorario)."""
    return (vI + vD) / 2.0, (vD - vI) / B_VIRTUAL


# ---------------- Maquina de estados ----------------
class Robot:
    def __init__(self, loco, perc):
        self.loco, self.perc = loco, perc
        self.estado = "ESPERA"

    def transicion(self, nuevo, motivo=""):
        if nuevo == self.estado:
            return
        print(f"\r\n>> {self.estado} -> {nuevo} {motivo}\r")
        self.estado = nuevo
        if nuevo in ("ESPERA", "PARO"):
            self.loco.frenar()

    def evento_tecla(self, k):
        if k in ESTADOS:
            self.transicion(ESTADOS[k], "(teclado)")
        elif k == " ":
            self.transicion("ESPERA", "(teclado)")

    def paso(self):
        # Guardas de seguridad: sin percepcion no hay comportamiento.
        if self.estado in COMPORTAMIENTOS and not self.perc.fresco():
            self.transicion("PARO", "(sin datos de percepcion)")
            return
        if self.estado == "PARO" and self.perc.fresco():
            self.transicion("ESPERA", "(percepcion recuperada)")
            return
        if self.estado not in COMPORTAMIENTOS:
            return

        sI = estimulo(self.perc.d["IR_I"])
        sD = estimulo(self.perc.d["IR_D"])
        vI, vD = COMPORTAMIENTOS[self.estado](sI, sD)
        if max(vI, vD) < V_PARO:
            self.loco.frenar()
            v = w = 0.0
        else:
            v, w = diferencial_a_robot(vI, vD)
            self.loco.mover(v, w)
        print(f"\r{self.estado:14s} IR_I={fmt(self.perc.d['IR_I'])} IR_D={fmt(self.perc.d['IR_D'])} "
              f"sI={sI:.2f} sD={sD:.2f}  v={v:+.3f} m/s  w={w:+.2f} rad/s   ", end="")


def fmt(d):
    return "  --  " if d is None else f"{d:6.0f}"


def tecla():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loco", default="/dev/ttyUSB0")
    ap.add_argument("--perc", default="/dev/ttyUSB1")
    a = ap.parse_args()

    loco, perc = Locomocion(a.loco), Percepcion(a.perc)
    loco.iniciar()
    perc.iniciar()
    robot = Robot(loco, perc)
    print("Listo. 1=Miedo 2=Agresion 3=Amor 4=Explorador 0/espacio=Espera q=salir\r")

    viejo = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    t_sens = t_cmd = 0.0
    try:
        while True:
            ahora = time.monotonic()
            k = tecla()
            if k == "q":
                break
            if k:
                robot.evento_tecla(k)
            if ahora - t_sens >= T_SENSOR_MS / 1000:
                perc.pedir()
                t_sens = ahora
            perc.leer()
            for msg in loco.leer():
                print(f"\r\n[loco] {msg}\r")
            if ahora - t_cmd >= T_CMD_MS / 1000:
                robot.paso()
                t_cmd = ahora
            time.sleep(0.01)
    finally:
        loco.parar()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        print("\nDetenido.")


if __name__ == "__main__":
    main()
