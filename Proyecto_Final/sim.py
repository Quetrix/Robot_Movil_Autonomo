#!/usr/bin/env python3
"""Simulador para probar la Pi sin el robot: imita a los dos ESP32 (mismas tramas y comandos)
y a la camara (conos y ArUco ya detectados), sobre el croquis de mapa.py.

    python3 robot.py --sim            # corrida completa; al final dibuja sim_trayectoria.png
    python3 sim.py                    # lo mismo con un resumen de choques y tiempo

No reemplaza las pruebas en el robot: los sensores son ideales con algo de ruido y la
odometria tiene un error de escala y de rumbo fijo para que se note la correccion en cada puerta.
"""
import math
import random
import threading
import time

import config
import geometria_robot as G
import mapa
import protocolo as P
from odometria import a_local

# Obstaculos sorpresa (cajas de 0.25 m) en la ruta, como en la prueba.
CAJAS = [(7.05, 13.4), (4.6, 11.95), (1.9, 8.9), (4.3, 5.6)]


class PuertoSim:
    """Lo minimo de serial.Serial que usa enlaces.py."""

    def __init__(self, al_escribir):
        self._buf = bytearray()
        self._cv = threading.Condition()
        self._al_escribir = al_escribir

    def entregar(self, datos):
        with self._cv:
            self._buf += datos
            self._cv.notify()

    @property
    def in_waiting(self):
        return len(self._buf)

    def read(self, n=1):
        with self._cv:
            if not self._buf:
                self._cv.wait(0.05)
            datos = bytes(self._buf[:n])
            del self._buf[:n]
            return datos

    def write(self, datos):
        for linea in datos.split(b"\n"):
            if linea:
                self._al_escribir(linea)
        return len(datos)


def _segmentos():
    segs = list(mapa.PAREDES)
    for x0, y0, x1, y1 in mapa.OBSTACULOS + [(x - .125, y - .125, x + .125, y + .125) for x, y in CAJAS]:
        segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    for p in mapa.PUERTAS:
        for cx, cy in p.conos():
            h = 0.05
            segs += [((cx - h, cy - h), (cx + h, cy - h)), ((cx + h, cy - h), (cx + h, cy + h)),
                     ((cx + h, cy + h), (cx - h, cy + h)), ((cx - h, cy + h), (cx - h, cy - h))]
    return segs


def _rayo(o, ang, segs, maximo=8.0):
    dx, dy = math.cos(ang), math.sin(ang)
    mejor = maximo
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - o[0]) * ey - (y1 - o[1]) * ex) / den
        u = ((x1 - o[0]) * dy - (y1 - o[1]) * dx) / den
        if 0 < t < mejor and 0 <= u <= 1:
            mejor = t
    return mejor


class Mundo:
    def __init__(self, registro=None, error_escala=0.03, deriva_dps=0.3):
        self.x, self.y, self.th = mapa.INICIO[0], mapa.INICIO[1], math.radians(mapa.INICIO[2])
        self.v = [0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0, 0.0]
        self.t_cmd = 0.0
        self.armado = False
        self.conteos = [0.0] * 4
        self.error_escala, self.deriva = error_escala, math.radians(deriva_dps)
        self.servo, self.servo_dir, self.barrido = 0.0, 1, (0.0, 150.0)
        self.segs = _segmentos()
        self.choques = 0
        self.trayectoria = []
        self.seq = {"L": 0, "T": 0, "S": 0}
        self.puerto_loco = PuertoSim(self._cmd_loco)
        self.puerto_perc = PuertoSim(self._cmd_perc)
        self._t0 = time.monotonic()

    def ms(self):
        return int((time.monotonic() - self._t0) * 1000)

    def _cmd_loco(self, linea):
        partes = linea.split(b"*")[0].decode().split(",")
        if partes[0] == "V" and self.armado:
            self.cmd = [int(partes[1]) / 1000, int(partes[2]) / 1000, math.radians(int(partes[3]))]
            self.t_cmd = time.monotonic()
        elif partes[0] == "X":
            self.cmd = [0.0, 0.0, 0.0]
        elif partes[0] == "?":
            self.puerto_loco.entregar(P.armar("I", "LOCO", "sim"))

    def _cmd_perc(self, linea):
        texto = linea.split(b"*")[0].decode()
        partes = texto.split(",")
        if partes[0] == "B":
            self.barrido = (float(partes[1]), float(partes[2]))
        elif partes[0] == "F":
            self.barrido = (0.0, 150.0)
        elif partes[0] == "?":
            self.puerto_perc.entregar(P.armar("I", "PERC", "sim"))

    def iniciar(self, armar_en_s=1.0):
        threading.Thread(target=self._bucle, args=(armar_en_s,), daemon=True).start()

    def _bucle(self, armar_en_s):
        dt, k = 0.01, 0
        while True:
            t = time.monotonic()
            if not self.armado and t - self._t0 > armar_en_s:
                self.armado = True
            if t - self.t_cmd > 0.5:
                self.cmd = [0.0, 0.0, 0.0]
            for i in range(3):        # respuesta de primer orden (~0.1 s)
                self.v[i] += (self.cmd[i] - self.v[i]) * min(1.0, dt / 0.1)
            vx, vy, wz = self.v
            c, s = math.cos(self.th), math.sin(self.th)
            nx, ny = self.x + (c * vx - s * vy) * dt, self.y + (s * vx + c * vy) * dt
            if _rayo((self.x, self.y), math.atan2(ny - self.y, nx - self.x), self.segs, 1.0) < 0.10 + math.hypot(nx - self.x, ny - self.y):
                self.choques += 1     # no atraviesa la pared: se queda
            else:
                self.x, self.y = nx, ny
            self.th += wz * dt
            # encoders: cinematica inversa con error de escala y deriva de rumbo de la odometria
            r, l = config.RADIO_RUEDA, config.L_GEOM
            wodo = wz + self.deriva * (1 if abs(vx) + abs(vy) > 0.02 else 0)
            w = [(vx - vy - l * wodo) / r, (vx + vy + l * wodo) / r, (vx + vy - l * wodo) / r, (vx - vy + l * wodo) / r]
            for i in range(4):
                self.conteos[i] += w[i] * dt / (2 * math.pi) * config.CPR[i] * (1 + self.error_escala)
            # servo del TF
            amp, vel = self.barrido
            if amp > 0:
                self.servo += self.servo_dir * vel * dt
                if abs(self.servo) >= amp:
                    self.servo, self.servo_dir = math.copysign(amp, self.servo), -self.servo_dir
            else:
                self.servo = 0.0
            self._tramas(k)
            k += 1
            if k % 10 == 0:
                self.trayectoria.append((self.x, self.y))
            time.sleep(max(0.0, dt - (time.monotonic() - t)))

    def _pose_sensor(self, x_mm, y_mm, rumbo_deg):
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + (c * x_mm - s * y_mm) / 1000, self.y + (s * x_mm + c * y_mm) / 1000,
                self.th + math.radians(rumbo_deg))

    def _tramas(self, k):
        ms = self.ms()
        x, y, a = self._pose_sensor(*G.tf_pose(self.servo))
        d = _rayo((x, y), a, self.segs) * 1000 * random.gauss(1, 0.01)
        flags = (P.T_BARRIENDO if self.barrido[0] > 0 else 0) | (P.T_INVALIDA if d > 8000 else 0)
        self.puerto_perc.entregar(P.armar("T", self.seq["T"], ms, round(self.servo * 100), round(min(d, 8000)), 900, flags))
        self.seq["T"] += 1
        if k % 2 == 0:
            flags = 1 if self.armado else 0
            self.puerto_loco.entregar(P.armar("L", self.seq["L"], ms, *[round(c) for c in self.conteos],
                                              0, 0, 0, 0, 0, 0, 0, 0, flags))
            self.seq["L"] += 1
        if k % 4 == 0:
            ds = []
            for lado in ("izq", "der"):
                sx, sy, sa = self._pose_sensor(*G.sharp_pose(lado))
                dd = _rayo((sx, sy), sa, self.segs, 1.0) * 1000
                ds.append(round(dd) if 100 <= dd <= 800 else 0)
            self.puerto_perc.entregar(P.armar("S", self.seq["S"], ms, ds[0], ds[1], 0, 0, 0))
            self.seq["S"] += 1


class VisionSim:
    """Conos y ArUco 'detectados' a partir de la pose real, con ruido."""

    def __init__(self, mundo, fov_deg=60, alcance=4.0):
        self.m = mundo
        self.fov, self.alcance = math.radians(fov_deg), alcance
        self.buscar_aruco_siempre = False
        self.ultimo_cuadro = None
        self.resultado = {"t": 0.0, "conos": [], "puerta": None, "aruco": None}
        threading.Thread(target=self._bucle, daemon=True).start()

    def _bucle(self):
        from vision import buscar_puerta
        while True:
            m = self.m
            cam = (m.x + math.cos(m.th) * G.CAMARA_X_MM / 1000, m.y + math.sin(m.th) * G.CAMARA_X_MM / 1000, m.th)
            conos = []
            for p in mapa.PUERTAS:
                for c in p.conos():
                    lx, ly = a_local(cam, c)
                    d, ang = math.hypot(lx, ly), math.atan2(ly, lx)
                    if 0.3 < d < self.alcance and abs(ang) < self.fov / 2:
                        d *= random.gauss(1, 0.04)
                        ang += random.gauss(0, math.radians(1))
                        xr, yr = G.CAMARA_X_MM / 1000 + d * math.cos(ang), d * math.sin(ang)
                        conos.append({"xr": xr, "yr": yr, "dist": math.hypot(xr, yr), "rumbo": math.atan2(yr, xr)})
            aruco = None
            lx, ly = a_local(cam, (mapa.FIN[0], 0.0))
            if 0.2 < math.hypot(lx, ly) < 2.5 and abs(math.atan2(ly, lx)) < self.fov / 2:
                n = a_local((0, 0, m.th), (0.0, 1.0))       # normal del marcador (+y mundo) en marco robot
                aruco = {"dic": "sim", "id": 7, "x": lx + G.CAMARA_X_MM / 1000, "y": ly, "z": 0.1,
                         "normal": [n[0], n[1], 0.0], "arriba": [0, 0, 1]}
            self.resultado = {"t": time.monotonic(), "conos": conos, "puerta": buscar_puerta(conos), "aruco": aruco}
            time.sleep(1 / 15)


if __name__ == "__main__":
    import sys
    import robot
    sys.argv = ["robot.py", "--sim", "--max-s", "400"] + sys.argv[1:]
    t0 = time.monotonic()
    nav = robot.main()
    m = nav.perc._puerto._al_escribir.__self__
    print("Estado final %s, objetivo %d/%d, %.0f s, choques %d, error final %.2f m" % (
        nav.estado, nav.i, len(nav.objetivos), time.monotonic() - t0, m.choques,
        math.hypot(m.x - mapa.FIN[0], m.y - config.D_ESTACIONAR)))
    print("Trayectoria:", mapa.dibujar("sim_trayectoria.png", m.trayectoria, CAJAS))
