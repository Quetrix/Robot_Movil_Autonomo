"""Paso 2: ajustar el rango de naranja a TU cono y TU iluminacion.

    python paso2_calibrar_color.py --camara 1
    python paso2_calibrar_color.py --imagen fotos/foto.jpg     # calibrar sobre una foto

1. Pon el cono frente a la camara y haz CLIC sobre la parte naranja: el rango se ajusta solo.
2. Afina con las barras hasta que en "mascara" el cono se vea blanco y el fondo negro.
3. Aleja y acerca el cono, y prueba con sombra, para confirmar que sigue blanco.
4. Tecla g = guardar en config_conos.json | q = salir sin guardar
"""
import argparse

import cv2
import numpy as np

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision import abrir_camara, cargar_config, detectar_conos, dibujar, guardar_config

VENTANA, VENTANA_MASCARA = "Paso 2 - clic en el cono", "mascara"
BARRAS = [("H min", 179), ("H max", 179), ("S min", 255), ("S max", 255), ("V min", 255), ("V max", 255)]
ANCHO_VISTA = 800


def leer_barras(cfg):
    v = [cv2.getTrackbarPos(n, VENTANA_MASCARA) for n, _ in BARRAS]
    cfg["hsv_min"], cfg["hsv_max"] = [v[0], v[2], v[4]], [v[1], v[3], v[5]]


def poner_barras(cfg):
    valores = [cfg["hsv_min"][0], cfg["hsv_max"][0], cfg["hsv_min"][1], cfg["hsv_max"][1], cfg["hsv_min"][2], cfg["hsv_max"][2]]
    for (nombre, _), valor in zip(BARRAS, valores):
        cv2.setTrackbarPos(nombre, VENTANA_MASCARA, int(valor))


def main():
    cfg = cargar_config()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camara", type=int, default=cfg["camara"])
    p.add_argument("--imagen", help="calibrar sobre una foto en vez de la camara")
    a = p.parse_args()

    cap = fija = None
    if a.imagen:
        fija = cv2.imread(a.imagen)
        if fija is None:
            raise SystemExit("No pude leer la imagen " + a.imagen)
    else:
        cap = abrir_camara(a.camara, cfg["ancho"], cfg["alto"], cfg["fps"], cfg["autofoco"])
        if cap is None:
            raise SystemExit("No hay camara en el indice %d." % a.camara)
        cfg["camara"] = a.camara

    cv2.namedWindow(VENTANA)
    cv2.namedWindow(VENTANA_MASCARA)
    for nombre, maximo in BARRAS:
        cv2.createTrackbar(nombre, VENTANA_MASCARA, 0, maximo, lambda _: None)
    poner_barras(cfg)

    estado = {"cuadro": None, "escala": 1.0}

    def clic(evento, x, y, *_):
        if evento != cv2.EVENT_LBUTTONDOWN or estado["cuadro"] is None:
            return
        px, py = int(x / estado["escala"]), int(y / estado["escala"])
        parche = estado["cuadro"][max(0, py - 7):py + 8, max(0, px - 7):px + 8]
        h, s, v = np.median(cv2.cvtColor(parche, cv2.COLOR_BGR2HSV).reshape(-1, 3), axis=0)
        print("Clic: H=%d S=%d V=%d" % (h, s, v))
        # Si el tono queda pegado al rojo (H<8) el minimo cruza a 172-179; la deteccion lo soporta.
        cfg["hsv_min"] = [int((h - 8) % 180), int(max(60, s - 90)), int(max(50, v - 110))]
        cfg["hsv_max"] = [int(min(179, h + 8)), 255, 255]
        poner_barras(cfg)

    cv2.setMouseCallback(VENTANA, clic)

    while True:
        if cap is not None:
            ok, cuadro = cap.read()
            if not ok:
                raise SystemExit("La camara dejo de entregar imagen.")
        else:
            cuadro = fija.copy()
        estado["cuadro"] = cuadro.copy()
        leer_barras(cfg)
        conos, mascara = detectar_conos(cuadro, cfg)
        dibujar(cuadro, conos)

        escala = estado["escala"] = min(1.0, ANCHO_VISTA / cuadro.shape[1])
        cv2.imshow(VENTANA, cv2.resize(cuadro, None, fx=escala, fy=escala))
        cv2.imshow(VENTANA_MASCARA, cv2.resize(mascara, None, fx=escala, fy=escala))

        tecla = cv2.waitKey(15) & 0xFF
        if tecla == ord("g"):
            guardar_config(cfg)
            print("Guardado: hsv_min=%s hsv_max=%s" % (cfg["hsv_min"], cfg["hsv_max"]))
        if tecla == ord("q"):
            break
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
