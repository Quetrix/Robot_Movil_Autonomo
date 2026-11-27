"""Falda de deteccion: matriz de angulos y distancias a partir del TF-Luna y los Sharp.

Cada lectura se guarda como punto en el marco odom (con la pose del robot en el instante de
llegada) y vence a los FALDA_VIGENCIA_S. Asi una pasada del barrido sigue siendo util mientras
el robot se mueve: al consultar, los puntos se pasan al marco actual del robot.
"""
import collections
import math
import threading
import time

import config
import geometria_robot as G
from odometria import a_global, a_local


class Falda:
    def __init__(self, pose_odom):
        self._pose = pose_odom                     # funcion -> (x, y, th) en odom
        self._lock = threading.Lock()
        self._puntos = collections.deque()         # (t, x_odom, y_odom, fuente)
        self._sharp = ([], [])                     # ultimas lecturas para la mediana
        self.sharp_mm = [None, None]               # mediana actual (izq, der); None = libre

    def _guardar(self, x_r, y_r, t, fuente):
        x, y = a_global(self._pose(), (x_r, y_r))
        with self._lock:
            self._puntos.append((t, x, y, fuente))

    def agregar_tf(self, angulo_deg, dist_m, t):
        if dist_m > config.FALDA_ALCANCE_TF_M:
            return
        x, y = G.tf_punto(angulo_deg, dist_m * 1000)
        self._guardar(x / 1000, y / 1000, t, "tf")

    def agregar_sharp(self, izq_m, der_m, t):
        for i, (d, lado) in enumerate(((izq_m, "izq"), (der_m, "der"))):
            ventana = self._sharp[i]
            ventana.append(d)
            del ventana[:-config.SHARP_MEDIANA]
            validos = sorted(v for v in ventana if v > 0)
            # mediana con los ceros (sin obstaculo) incluidos: un pico aislado no crea un punto
            if len(validos) * 2 > len(ventana):
                m = validos[len(validos) // 2]
                self.sharp_mm[i] = m * 1000
                x, y = G.punto_detectado(G.sharp_pose(lado), m * 1000)
                self._guardar(x / 1000, y / 1000, t, lado)
            else:
                self.sharp_mm[i] = None

    def puntos(self, ahora=None):
        """Puntos vigentes en el marco actual del robot: [(x, y, fuente)]."""
        ahora = time.monotonic() if ahora is None else ahora
        limite = ahora - config.FALDA_VIGENCIA_S
        pose = self._pose()
        with self._lock:
            while self._puntos and self._puntos[0][0] < limite:
                self._puntos.popleft()
            copia = list(self._puntos)
        return [(*a_local(pose, (x, y)), f) for _, x, y, f in copia]

    def matriz(self, ahora=None, sector_deg=config.FALDA_SECTOR_DEG):
        """[(angulo_deg, distancia_m)] por sector, desde el centro del robot. Es la 'matriz de
        angulos y distancias' del enunciado; los sectores sin puntos no aparecen."""
        mejor = {}
        for x, y, _ in self.puntos(ahora):
            a = math.degrees(math.atan2(y, x))
            k = int(round(a / sector_deg)) * sector_deg
            d = math.hypot(x, y)
            if d < mejor.get(k, 1e9):
                mejor[k] = d
        return sorted(mejor.items())

    def pasillo(self, rumbo, ancho=config.ROBOT_ANCHO_M + 0.06, ignorar=(), ahora=None):
        """Distancia libre desde el borde del robot en la direccion 'rumbo' (marco robot),
        considerando solo puntos dentro de un pasillo del ancho dado. ignorar: [(x, y, radio)]."""
        c, s = math.cos(rumbo), math.sin(rumbo)
        borde = abs(c) * config.ROBOT_LARGO_M / 2 + abs(s) * config.ROBOT_ANCHO_M / 2
        libre = math.inf
        for x, y, _ in self.puntos(ahora):
            if any(math.hypot(x - ix, y - iy) < r for ix, iy, r in ignorar):
                continue
            a = c * x + s * y                    # avance a lo largo del pasillo
            b = -s * x + c * y                   # desvio lateral
            if a > 0 and abs(b) <= ancho / 2:
                libre = min(libre, a - borde)
        return libre
