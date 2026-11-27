"""Grabacion de cada corrida en registros/AAAAMMDD_HHMMSS/.

  crudo_loco.txt, crudo_perc.txt   todas las lineas del USB (RX y TX) con su tiempo de la Pi.
                                   Con esto se puede reconstruir todo lo demas despues.
  nav.csv                          estado de la navegacion a la frecuencia del lazo.
  eventos.txt                      cambios de estado, avisos y errores.
  fotos/                           una foto cada FOTO_CADA_S y al ver un cono/ArUco nuevo.

La escritura usa buffer y se vacia una vez por segundo; si la Pi se apaga de golpe se pierde
como maximo el ultimo segundo.
"""
import csv
import os
import threading
import time

import config


class Registro:
    def __init__(self, carpeta_base=config.CARPETA_REGISTROS, activo=True):
        self.activo = activo
        self.t0 = time.monotonic()
        self._lock = threading.Lock()
        self._archivos = {}
        self._csv = None
        self._ultimo_flush = 0.0
        self.carpeta = None
        if activo:
            self.carpeta = os.path.join(carpeta_base, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(os.path.join(self.carpeta, "fotos"), exist_ok=True)

    def t(self):
        return time.monotonic() - self.t0

    def _archivo(self, nombre):
        f = self._archivos.get(nombre)
        if f is None:
            f = open(os.path.join(self.carpeta, nombre), "a", buffering=1 << 16, encoding="utf-8")
            self._archivos[nombre] = f
        return f

    def crudo(self, enlace, sentido, linea):
        """sentido: 'RX' o 'TX'; linea: bytes sin el salto final."""
        if not self.activo:
            return
        texto = linea.decode("ascii", errors="replace").rstrip("\r\n")
        with self._lock:
            self._archivo("crudo_%s.txt" % enlace).write("%.4f\t%s\t%s\n" % (self.t(), sentido, texto))

    def evento(self, texto):
        linea = "%9.3f  %s" % (self.t(), texto)
        print(linea, flush=True)
        if self.activo:
            with self._lock:
                self._archivo("eventos.txt").write(linea + "\n")

    def nav(self, fila: dict):
        if not self.activo:
            return
        with self._lock:
            if self._csv is None:
                self._csv = csv.DictWriter(self._archivo("nav.csv"), fieldnames=["t"] + list(fila))
                self._csv.writeheader()
            self._csv.writerow({"t": "%.3f" % self.t(), **fila})

    def foto(self, imagen, etiqueta):
        if not self.activo or imagen is None:
            return
        import cv2
        ruta = os.path.join(self.carpeta, "fotos", "%08.2f_%s.jpg" % (self.t(), etiqueta))
        cv2.imwrite(ruta, imagen, [cv2.IMWRITE_JPEG_QUALITY, 70])

    def vaciar(self, forzar=False):
        if not self.activo or (not forzar and time.monotonic() - self._ultimo_flush < 1.0):
            return
        self._ultimo_flush = time.monotonic()
        with self._lock:
            for f in self._archivos.values():
                f.flush()

    def cerrar(self):
        self.vaciar(forzar=True)
        with self._lock:
            for f in self._archivos.values():
                f.close()
            self._archivos.clear()
