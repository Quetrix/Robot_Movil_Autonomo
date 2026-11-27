"""Croquis del escenario (Figura 2 del enunciado) como datos para la navegacion.

Marco mundo: origen en la esquina inferior izquierda del laboratorio F3-03 (abajo a la
izquierda en la figura), X hacia la derecha, Y hacia arriba de la figura, metros.

IMPORTANTE: la figura NO esta a escala (la escala en pixeles por metro varia entre 41 y 63
segun la cota). Todo lo que tiene cota se toma de la cota; lo que no la tiene (posicion de
los conos dentro de cada tramo, ancho de las mesas, punto de inicio) se estimo con la escala
local y esta marcado como ESTIMADO. Por eso la navegacion usa el mapa solo para llegar cerca
de cada puerta y despues se guia con los conos que ve la camara.

python3 mapa.py dibuja el mapa en mapa.png para revisarlo.
"""
import math
from dataclasses import dataclass


@dataclass
class Puerta:
    nombre: str
    x: float
    y: float
    rumbo_deg: float          # direccion de cruce (mundo)
    nota: str = ""

    @property
    def rumbo(self):
        return math.radians(self.rumbo_deg)

    def punto(self, d):
        """Punto a distancia d del centro a lo largo del cruce (d < 0: antes de la puerta)."""
        return (self.x + d * math.cos(self.rumbo), self.y + d * math.sin(self.rumbo))

    def conos(self, separacion=0.5):
        nx, ny = -math.sin(self.rumbo), math.cos(self.rumbo)
        h = separacion / 2
        return [(self.x + h * nx, self.y + h * ny), (self.x - h * nx, self.y - h * ny)]


# ---------------------------------------------------------------- cotas del croquis
LAB_ALTO = 3.7 + 3.9 + 3.0            # 10.6: pared inferior del pasillo horizontal
PASILLO_Y0, PASILLO_Y1 = LAB_ALTO, LAB_ALTO + 1.85
PASILLO_X1 = 0.86 + 1.3 + 0.8 + 2.7   # 5.66: inicio de la sala de salida
SALA_X0, SALA_X1 = PASILLO_X1, PASILLO_X1 + 3.0
SALA_Y0, SALA_Y1 = PASILLO_Y1, PASILLO_Y1 + 3.0

# Inicio: rectangulo rojo pegado a la pared superior de la sala de salida (ESTIMADO).
INICIO = (7.15, 15.10, -90.0)         # x, y, rumbo_deg: mirando hacia el pasillo (ASUMIDO)
# Fin: rectangulo rojo a 0.7 m (borde derecho) de las mesas inferiores, contra la pared.
FIN = (4.20, 0.15)
FIN_RUMBO_DEG = -90.0                 # ASUMIDO: el marcador define la orientacion real

PUERTAS = [
    Puerta("P1", 7.33, 12.45, -90, "salida de la sala de inicio; x ESTIMADA"),
    Puerta("P2", 2.96, 11.99, 180, "entre el obstaculo fijo y la pared superior; y ESTIMADA"),
    Puerta("P3", 0.61, 10.60, -90, "entrada al laboratorio; cono derecho en x=0.86 (cota)"),
    Puerta("P4", 3.36, 7.60, -37, "diagonal en el centro del laboratorio; todo ESTIMADO"),
    Puerta("P5", 4.69, 3.70, -90, "sobre la linea de la cota 3.7; x ESTIMADA"),
]

# Ruta: puertas y puntos de paso (para no cortar esquinas). ('paso', x, y) o ('puerta', indice).
RUTA = [
    ("puerta", 0),
    ("paso", 7.00, 11.55),            # bajar al pasillo antes de girar al oeste
    ("puerta", 1),
    ("puerta", 2),
    ("puerta", 3),
    ("puerta", 4),
]

# Obstaculos fijos y mesas como rectangulos (x0, y0, x1, y1). Solo para el simulador y el dibujo.
OBSTACULOS = [
    (5.66, 13.59, 6.07, 14.27),       # obstaculo fijo en la sala de inicio (ESTIMADO)
    (2.16, 10.60, 2.96, 11.60),       # obstaculo fijo del pasillo (0.8 x 1, cotas)
    (0.00, 0.00, 3.80, 3.70),         # mesas inferiores izquierdas (cotas)
    (0.00, 3.70, 1.00, 7.60),         # mesas de la pared izquierda (cotas)
    (1.70, 9.85, PASILLO_X1, LAB_ALTO),     # mesas bajo el pasillo (ESTIMADO)
    (5.06, 4.60, PASILLO_X1, 9.50),   # mesas de la derecha, tramo alto (ESTIMADO)
    (5.40, 0.00, PASILLO_X1, 4.60),   # mesas de la derecha, tramo bajo (ESTIMADO)
    (0.00, 10.60, 0.24, 9.85),        # resto de pared/mesa a la izquierda de la entrada
]

# Paredes como segmentos ((x0, y0), (x1, y1)).
PAREDES = [
    ((0, 0), (PASILLO_X1, 0)), ((0, 0), (0, PASILLO_Y1)), ((PASILLO_X1, 0), (PASILLO_X1, PASILLO_Y0)),
    ((0, PASILLO_Y1), (SALA_X0, PASILLO_Y1)), ((SALA_X0, PASILLO_Y1), (SALA_X0, SALA_Y1)),
    ((SALA_X0, SALA_Y1), (SALA_X1, SALA_Y1)), ((SALA_X1, SALA_Y1), (SALA_X1, PASILLO_Y0)),
    ((0, PASILLO_Y0), (0.24, PASILLO_Y0)), ((1.74, PASILLO_Y0), (SALA_X1, PASILLO_Y0)),  # puerta 0.24-1.74 (cota 1.5)
]


def objetivos():
    """Lista ordenada de objetivos para la navegacion: dicts con tipo, posicion y puerta."""
    out = []
    for item in RUTA:
        if item[0] == "paso":
            out.append({"tipo": "paso", "xy": (item[1], item[2])})
        else:
            out.append({"tipo": "puerta", "puerta": PUERTAS[item[1]]})
    out.append({"tipo": "fin", "xy": FIN, "rumbo": math.radians(FIN_RUMBO_DEG)})
    return out


def dibujar(ruta="mapa.png", trayectoria=None, cajas=()):
    import cv2
    import numpy as np
    esc, m = 60, 30
    ancho, alto = int(SALA_X1 * esc) + 2 * m, int(SALA_Y1 * esc) + 2 * m
    img = np.full((alto, ancho, 3), 255, np.uint8)
    px = lambda x, y: (int(m + x * esc), int(alto - m - y * esc))
    for x0, y0, x1, y1 in OBSTACULOS:
        cv2.rectangle(img, px(x0, y0), px(x1, y1), (200, 200, 200), -1)
    for a, b in PAREDES:
        cv2.line(img, px(*a), px(*b), (0, 0, 0), 2)
    for p in PUERTAS:
        for c in p.conos():
            cv2.circle(img, px(*c), 5, (0, 120, 255), -1)
        cv2.arrowedLine(img, px(*p.punto(-0.4)), px(*p.punto(0.4)), (0, 160, 0), 1, tipLength=0.3)
        cv2.putText(img, p.nombre, px(p.x + 0.15, p.y + 0.15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.circle(img, px(*INICIO[:2]), 7, (0, 0, 255), 2)
    cv2.rectangle(img, px(FIN[0] - 0.15, FIN[1] - 0.1), px(FIN[0] + 0.15, FIN[1] + 0.1), (0, 0, 255), -1)
    for x, y in cajas:
        cv2.rectangle(img, px(x - .125, y - .125), px(x + .125, y + .125), (60, 60, 160), -1)
    if trayectoria:
        for a, b in zip(trayectoria, trayectoria[1:]):
            cv2.line(img, px(*a), px(*b), (255, 0, 0), 1)
    cv2.imwrite(ruta, img)
    return ruta


if __name__ == "__main__":
    for o in objetivos():
        print(o)
    print("dibujo:", dibujar())
