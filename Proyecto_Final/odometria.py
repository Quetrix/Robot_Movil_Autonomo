"""Odometria Mecanum con los conteos acumulados de los encoders.

Marco odom: donde arranco el robot. Marco mundo: el del croquis (mapa.py). La correccion
mundo<-odom se ajusta al cruzar cada puerta (el enunciado lo sugiere: "reiniciar la creencia
de posicion en base a las dimensiones del croquis").
"""
import math
import threading

import config


def envolver(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def componer(a, b):
    """Pose a (x, y, th) seguida de la pose relativa b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], envolver(a[2] + b[2]))


def inversa(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def a_local(pose, punto):
    """Punto del marco global al marco de la pose."""
    dx, dy = punto[0] - pose[0], punto[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def a_global(pose, punto):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * punto[0] - s * punto[1], pose[1] + s * punto[0] + c * punto[1])


class Odometria:
    def __init__(self, radio=config.RADIO_RUEDA, l_geom=config.L_GEOM, cpr=config.CPR):
        self.radio, self.l_geom, self.cpr = radio, l_geom, cpr
        self._lock = threading.Lock()
        self._previo = None
        self._t_previo = None
        self.pose = (0.0, 0.0, 0.0)          # marco odom
        self.vel = (0.0, 0.0, 0.0)           # marco robot
        self.recorrido = 0.0                 # distancia total [m]
        self.correccion = (0.0, 0.0, 0.0)    # mundo <- odom

    def actualizar(self, conteos, t_esp):
        with self._lock:
            if self._previo is None:
                self._previo, self._t_previo = list(conteos), t_esp
                return
            dphi = [2 * math.pi * (c - p) / n for c, p, n in zip(conteos, self._previo, self.cpr)]
            dt = t_esp - self._t_previo
            self._previo, self._t_previo = list(conteos), t_esp
            r, l = self.radio, self.l_geom
            dx = r / 4 * (dphi[0] + dphi[1] + dphi[2] + dphi[3])
            dy = r / 4 * (-dphi[0] + dphi[1] + dphi[2] - dphi[3])
            dth = r / (4 * l) * (-dphi[0] + dphi[1] - dphi[2] + dphi[3])
            x, y, th = self.pose
            media = th + dth / 2                 # integracion con el rumbo medio del paso
            c, s = math.cos(media), math.sin(media)
            self.pose = (x + c * dx - s * dy, y + s * dx + c * dy, envolver(th + dth))
            self.recorrido += math.hypot(dx, dy)
            if dt > 0:
                self.vel = (dx / dt, dy / dt, dth / dt)

    def pose_odom(self):
        with self._lock:
            return self.pose

    def pose_mundo(self):
        with self._lock:
            return componer(self.correccion, self.pose)

    def fijar_mundo(self, pose_mundo):
        """Declara que la pose actual del robot en el mundo es pose_mundo."""
        with self._lock:
            self.correccion = componer(pose_mundo, inversa(self.pose))

    def odom_a_mundo(self, p):
        return a_global(self.correccion, p)

    def mundo_a_odom(self, p):
        return a_local(self.correccion, p)
