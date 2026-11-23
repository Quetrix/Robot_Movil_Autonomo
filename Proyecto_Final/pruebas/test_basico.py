"""Pruebas rapidas sin robot: python3 -m unittest discover pruebas   (desde Proyecto_Final)."""
import math
import re
import sys
import unittest
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

import cv2
import numpy as np

import config
import geometria_robot as G
import mapa
import protocolo as P
import vision
from falda import Falda
from odometria import Odometria

FIRMWARE = {
    "loco": RAIZ.parent / "Avance_1_Locomocion/locomocion/locomocion.ino",
    "perc": RAIZ.parent / "Avance_2_Percepcion/percepcion/percepcion.ino",
}


class Protocolo(unittest.TestCase):
    def test_ida_y_vuelta(self):
        t = P.armar("T", 1, 2, -3000, 1234, 900, 4)
        self.assertEqual(P.separar(t.strip()), ("T", ["1", "2", "-3000", "1234", "900", "4"]))

    def test_checksum_malo(self):
        t = bytearray(P.armar("V", 100, 0, 0))
        t[2] = ord("9")
        self.assertIsNone(P.separar(bytes(t)))

    def test_texto_y_basura(self):
        self.assertEqual(P.separar(b"# hola"), ("#", "hola"))
        self.assertIsNone(P.separar(b"\xff\xfeets Jun  8 2016 00:22:57"))

    def test_campos_iguales_al_firmware(self):
        """El numero de campos de cada trama coincide con los snprintf del firmware."""
        for nombre, ruta in FIRMWARE.items():
            texto = ruta.read_text()
            for tipo, n in P.CAMPOS.items():
                if n is None:
                    continue
                m = re.search(r'"%s,(%%[^"]*)"' % tipo, texto)
                if m:
                    self.assertEqual(m.group(1).count("%"), n, "%s trama %s" % (nombre, tipo))


class Odometria_(unittest.TestCase):
    def rpm_firmware(self, vx, vy, wz):
        r, l = config.RADIO_RUEDA, config.L_GEOM
        w = [(vx - vy - l * wz) / r, (vx + vy + l * wz) / r, (vx + vy - l * wz) / r, (vx - vy + l * wz) / r]
        return [x * 60 / (2 * math.pi) for x in w]

    def test_inversa_de_la_cinematica_del_firmware(self):
        for vx, vy, wz in ((0.2, 0, 0), (0, 0.15, 0), (0, 0, 1.0), (0.1, -0.1, 0.5)):
            o = Odometria()
            c = [0.0] * 4
            o.actualizar([0] * 4, 0.0)
            rpm = self.rpm_firmware(vx, vy, wz)
            for k in range(1, 51):                       # 1 s a 50 Hz
                c = [ci + r / 60 * n * 0.02 for ci, r, n in zip(c, rpm, config.CPR)]
                o.actualizar([round(x) for x in c], k * 0.02)
            x, y, th = o.pose_odom()
            self.assertAlmostEqual(th, wz, delta=0.01)
            if wz == 0:
                self.assertAlmostEqual(x, vx, delta=0.003)
                self.assertAlmostEqual(y, vy, delta=0.003)


class Camara(unittest.TestCase):
    def test_piso_ida_y_vuelta(self):
        m = vision.ModeloCamara(640, 480, 60, inclinacion_deg=15, altura_m=0.26)
        for p in ((1.0, 0.0), (2.0, 0.5), (0.7, -0.3)):
            # proyectar el punto del piso a pixel y volver
            v = np.array([p[0] - m.x0, p[1] - m.y0, -m.h])
            c, s = math.cos(m.t), math.sin(m.t)
            z = v[0] * c - v[2] * s                      # adelante de la camara
            yc = -v[0] * s - v[2] * c                    # abajo
            xc = -v[1]
            u, vv = m.cx + m.f * xc / z, m.cy + m.f * yc / z
            q = m.al_piso(u, vv)
            self.assertAlmostEqual(q[0], p[0], places=6)
            self.assertAlmostEqual(q[1], p[1], places=6)

    def test_inclinacion_recomendada(self):
        inc, franja = G.inclinacion_optima(0.26)
        self.assertTrue(10 < inc < 20 and franja < 36)


class Conos(unittest.TestCase):
    """Escena sintetica de Pruebas/Deteccion_Conos/paso0_autoprueba.py."""

    @staticmethod
    def cono(img, cx, base_y, alto, color=(0, 110, 255)):
        ancho, punta = int(alto * 0.6), int(alto * 0.12)
        cuerpo = np.array([[cx - punta // 2, base_y - alto], [cx + punta // 2, base_y - alto],
                           [cx + ancho // 2, base_y], [cx - ancho // 2, base_y]], np.int32)
        cv2.fillConvexPoly(img, cuerpo, color)
        y0, y1 = base_y - int(alto * 0.62), base_y - int(alto * 0.45)
        franja = np.zeros(img.shape[:2], np.uint8)
        cv2.fillConvexPoly(franja, cuerpo, 255)
        franja[:y0] = 0
        franja[y1:] = 0
        img[franja > 0] = (235, 235, 235)

    def test_dos_conos_sin_falsos(self):
        img = np.full((480, 640, 3), 128, np.uint8)
        self.cono(img, 175, 400, 200)
        self.cono(img, 450, 280, 60)
        cv2.rectangle(img, (525, 320), (600, 427), (0, 110, 255), -1)   # caja naranja
        cv2.circle(img, (320, 400), 40, (0, 110, 255), -1)              # pelota naranja
        img = cv2.add(img, np.random.default_rng(0).integers(0, 25, img.shape, dtype=np.uint8))
        cfg = dict(vision.CONFIG_DEFECTO)
        conos, _ = vision.detectar_y_ubicar(img, cfg)
        centros = sorted(round(c["x"] + c["w"] / 2) for c in conos)
        self.assertEqual(len(conos), 2, centros)
        self.assertLess(abs(centros[0] - 175), 10)
        self.assertLess(abs(centros[1] - 450), 10)
        self.assertGreater(conos[0]["rumbo"], 0)          # el de la izquierda tiene rumbo positivo

    def test_puerta(self):
        conos = [{"xr": 1.0, "yr": 0.25}, {"xr": 1.0, "yr": -0.25}, {"xr": 2.5, "yr": 1.0}]
        cx, cy, rumbo, d, _ = vision.buscar_puerta(conos)
        self.assertAlmostEqual(cx, 1.0)
        self.assertAlmostEqual(cy, 0.0)
        self.assertAlmostEqual(rumbo, 0.0)


class Aruco(unittest.TestCase):
    def test_detecta_y_ubica(self):
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        gen = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker
        marca = gen(dic, 7, 120)
        img = np.full((480, 640), 255, np.uint8)
        img[180:300, 260:380] = marca
        m = vision.ModeloCamara(640, 480, 60, inclinacion_deg=0, altura_m=0.26)
        a = vision.Aruco(m, lado=0.10)
        r = a.detectar(img)
        self.assertIsNotNone(r)
        self.assertEqual(r["id"], 7)
        f = m.f
        self.assertAlmostEqual(r["x"] - m.x0, 0.10 * f / 120, delta=0.02)   # distancia por tamano aparente
        self.assertLess(r["normal"][0], -0.9)                             # apunta hacia el robot (-X)


class Falda_(unittest.TestCase):
    def test_tf_al_frente(self):
        f = Falda(lambda: (0.0, 0.0, 0.0))
        f.agregar_tf(0.0, 1.0, t=1e9)
        x, y, fuente = f.puntos(ahora=1e9)[0]
        self.assertAlmostEqual(x, (G.EJE_SERVO_X_MM + G.TF_ORIGEN_CABEZAL_X_MM) / 1000 + 1.0)
        self.assertAlmostEqual(f.pasillo(0.0, ahora=1e9), x - config.ROBOT_LARGO_M / 2)
        self.assertEqual(f.pasillo(math.pi / 2, ahora=1e9), math.inf)


class Mapa(unittest.TestCase):
    def test_puertas(self):
        for p in mapa.PUERTAS:
            a, b = p.conos()
            self.assertAlmostEqual(math.dist(a, b), 0.5)
        self.assertEqual(mapa.objetivos()[-1]["tipo"], "fin")


if __name__ == "__main__":
    unittest.main()
