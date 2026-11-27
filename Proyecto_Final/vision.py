"""Camara (Arducam IMX219 USB), conos naranjas y marcador ArUco.

La deteccion de conos es la de Pruebas/Deteccion_Conos/conos.py (color HSV + forma, union de
las dos manchas que deja la franja reflectiva), ahora con posicion en el marco robot:
  - por el plano del piso (base del cono + altura e inclinacion de la camara), si la base se ve;
  - si no, por la altura aparente del cono (CONO_ALTO_M).
El rango HSV se calibra con herramientas_conos/paso2_calibrar_color.py y queda en config_conos.json.
"""
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import config
import geometria_robot as G

RUTA_CONFIG = Path(__file__).with_name("config_conos.json")
CONFIG_DEFECTO = {
    "camara": config.CAMARA_INDICE,
    "ancho": config.CAMARA_ANCHO,
    "alto": config.CAMARA_ALTO,
    "fps": config.CAMARA_PROCESAR_HZ,
    "autofoco": True,
    "fov_horizontal_grados": config.CAMARA_FOV_H_DEG,
    "alto_cono_m": config.CONO_ALTO_M,
    "hsv_min": [2, 120, 90],
    "hsv_max": [22, 255, 255],
    "area_minima_frac": 0.0004,
}


def cargar_config():
    cfg = dict(CONFIG_DEFECTO)
    if RUTA_CONFIG.exists():
        cfg.update(json.loads(RUTA_CONFIG.read_text(encoding="utf-8")))
    return cfg


def guardar_config(cfg):
    RUTA_CONFIG.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


# ---------------------------------------------------------------- camara
def abrir_camara(indice, ancho, alto, fps=15, autofoco=True):
    """Abre la camara en MJPG (en YUYV la Arducam baja a ~10 fps). None si no hay camara."""
    if sys.platform == "win32":
        backend = cv2.CAP_DSHOW
    elif sys.platform.startswith("linux"):
        backend = cv2.CAP_V4L2
    else:
        backend = cv2.CAP_ANY
    cap = cv2.VideoCapture(indice, backend)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofoco else 0)
    return cap


def describir_camara(cap):
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00 ")
    return "%dx%d %s %.0f fps" % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                   codec or "?", cap.get(cv2.CAP_PROP_FPS))


# ---------------------------------------------------------------- conos en la imagen
def mascara_naranja(bgr, cfg):
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    h0, s0, v0 = (int(v) for v in cfg["hsv_min"])
    h1, s1, v1 = (int(v) for v in cfg["hsv_max"])
    if h0 <= h1:
        m = cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
    else:  # el rango cruza el rojo (ej. H de 170 a 15)
        m = cv2.inRange(hsv, (h0, s0, v0), (179, s1, v1)) | cv2.inRange(hsv, (0, s0, v0), (h1, s1, v1))
    lado = max(3, round(bgr.shape[1] / 256)) | 1
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (lado, lado))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)


def _unir_franjas(contornos):
    """Agrupa manchas apiladas verticalmente: la franja blanca reflejante parte al cono en dos."""
    cajas = [cv2.boundingRect(c) for c in contornos]
    padre = list(range(len(cajas)))

    def raiz(i):
        while padre[i] != i:
            padre[i] = padre[padre[i]]
            i = padre[i]
        return i

    for i in range(len(cajas)):
        for j in range(i + 1, len(cajas)):
            a, b = sorted((cajas[i], cajas[j]), key=lambda r: r[1])
            ax, ay, aw, ah = a
            bx, by, bw, bh = b
            traslape = min(ax + aw, bx + bw) - max(ax, bx)
            hueco = by - (ay + ah)
            if traslape >= 0.6 * min(aw, bw) and hueco <= 0.8 * max(ah, bh) and aw <= 1.15 * bw:
                padre[raiz(i)] = raiz(j)
    grupos = {}
    for i, c in enumerate(contornos):
        grupos.setdefault(raiz(i), []).append(c)
    return [np.vstack(g) for g in grupos.values()]


def detectar_conos(bgr, cfg):
    """(conos, mascara). Cada cono: x, y, w, h (caja en pixeles), cortado (bool)."""
    alto_img, ancho_img = bgr.shape[:2]
    mascara = mascara_naranja(bgr, cfg)
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_min = cfg["area_minima_frac"] * ancho_img * alto_img
    piezas = [c for c in contornos if cv2.contourArea(c) >= area_min / 4]
    conos = []
    for puntos in _unir_franjas(piezas):
        envolvente = cv2.convexHull(puntos)
        x, y, w, h = cv2.boundingRect(envolvente)
        if h < 8 or cv2.contourArea(envolvente) < area_min or not 0.7 <= h / w <= 3.5:
            continue
        silueta = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(silueta, envolvente - (x, y), 255)
        anchos = np.count_nonzero(silueta, axis=1)
        cuarto = max(1, h // 4)
        if anchos[:cuarto].mean() > 0.6 * anchos[-cuarto:].mean():   # punta angosta, base ancha
            continue
        relleno = cv2.countNonZero(mascara[y:y + h, x:x + w] & silueta) / max(1, cv2.countNonZero(silueta))
        if relleno < 0.45:
            continue
        conos.append({"x": x, "y": y, "w": w, "h": h, "cortado": y <= 1 or y + h >= alto_img - 1})
    conos.sort(key=lambda c: -c["h"])
    return conos, mascara


# ---------------------------------------------------------------- pixeles -> marco robot
class ModeloCamara:
    """Pinhole sin distorsion a partir del FOV horizontal; pose de la camara de geometria_robot."""

    def __init__(self, ancho, alto, fov_h_deg, inclinacion_deg=G.CAMARA_INCLINACION_DEG,
                 altura_m=None):
        self.ancho, self.alto = ancho, alto
        self.f = (ancho / 2) / math.tan(math.radians(fov_h_deg) / 2)
        self.cx, self.cy = ancho / 2, alto / 2
        self.t = math.radians(inclinacion_deg)
        self.h = G.altura_camara_m() if altura_m is None else altura_m
        self.x0, self.y0 = G.CAMARA_X_MM / 1000, G.CAMARA_Y_MM / 1000

    def rayo(self, u, v):
        """Direccion (marco robot, sin normalizar) del pixel (u, v)."""
        xn, yn = (u - self.cx) / self.f, (v - self.cy) / self.f
        c, s = math.cos(self.t), math.sin(self.t)
        return (c - yn * s, -xn, -s - yn * c)

    def rumbo(self, u, v):
        r = self.rayo(u, v)
        return math.atan2(r[1], r[0])

    def al_piso(self, u, v):
        """Punto del piso (x, y) visto en el pixel, o None si el rayo no baja."""
        r = self.rayo(u, v)
        if r[2] >= -1e-3:
            return None
        k = self.h / -r[2]
        return self.x0 + k * r[0], self.y0 + k * r[1]

    def matriz(self):
        return np.array([[self.f, 0, self.cx], [0, self.f, self.cy], [0, 0, 1]], float)

    def a_robot(self, p_cam):
        """Vector del marco de la camara de OpenCV (x der, y abajo, z adelante) al marco robot."""
        c, s = math.cos(self.t), math.sin(self.t)
        x, y, z = p_cam
        return (self.x0 + z * c - y * s, self.y0 - x, self.h - z * s - y * c)


def ubicar_conos(conos, modelo, alto_cono):
    """Agrega xr, yr (marco robot, m), distancia y rumbo a cada cono."""
    for c in conos:
        u = c["x"] + c["w"] / 2
        base = None if c["cortado"] else modelo.al_piso(u, c["y"] + c["h"])
        if base is not None:
            ang = math.atan2(base[1] - modelo.y0, base[0] - modelo.x0)
            base = (base[0] + 0.05 * math.cos(ang), base[1] + 0.05 * math.sin(ang))  # centro del cono
            c["metodo"] = "piso"
        else:
            d = alto_cono * modelo.f / c["h"]
            ang = modelo.rumbo(u, c["y"] + c["h"] / 2)
            base = (modelo.x0 + d * math.cos(ang), modelo.y0 + d * math.sin(ang))
            c["metodo"] = "altura"
        c["xr"], c["yr"] = base
        c["dist"] = math.hypot(*base)
        c["rumbo"] = math.atan2(base[1], base[0])
    return conos


def buscar_puerta(conos):
    """Par de conos separado ~0.5 m mas cercano. Devuelve (cx, cy, rumbo_cruce) en marco robot."""
    mejor = None
    for i in range(len(conos)):
        for j in range(i + 1, len(conos)):
            a, b = conos[i], conos[j]
            sep = math.hypot(a["xr"] - b["xr"], a["yr"] - b["yr"])
            if abs(sep - config.CONO_SEPARACION_M) > config.CONO_SEPARACION_TOL_M:
                continue
            cx, cy = (a["xr"] + b["xr"]) / 2, (a["yr"] + b["yr"]) / 2
            nx, ny = -(b["yr"] - a["yr"]), b["xr"] - a["xr"]
            if nx * cx + ny * cy < 0:           # la normal apunta hacia adelante (lejos del robot)
                nx, ny = -nx, -ny
            d = math.hypot(cx, cy)
            if mejor is None or d < mejor[3]:
                mejor = (cx, cy, math.atan2(ny, nx), d, ((a["xr"], a["yr"]), (b["xr"], b["yr"])))
    return mejor


def detectar_y_ubicar(bgr, cfg):
    """Para las herramientas: deteccion + posicion con el modelo de camara de la configuracion."""
    conos, mascara = detectar_conos(bgr, cfg)
    modelo = ModeloCamara(bgr.shape[1], bgr.shape[0], cfg["fov_horizontal_grados"])
    return ubicar_conos(conos, modelo, cfg["alto_cono_m"]), mascara


def texto_conos(conos):
    if not conos:
        return "sin conos"
    return " | ".join("cono %d: %.2f m, %+.1f grados (%s)" % (i + 1, c["dist"], math.degrees(c["rumbo"]), c["metodo"])
                      for i, c in enumerate(conos))


def dibujar(bgr, conos):
    for c in conos:
        x, y, w, h = c["x"], c["y"], c["w"], c["h"]
        cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if "dist" in c:
            texto = "%.2f m %+.0f" % (c["dist"], math.degrees(c["rumbo"]))
            cv2.putText(bgr, texto, (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(bgr, texto, (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return bgr


# ---------------------------------------------------------------- ArUco
def _detector(nombre):
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, nombre))
    if hasattr(cv2.aruco, "ArucoDetector"):                    # OpenCV >= 4.7
        det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
        return det.detectMarkers
    params = cv2.aruco.DetectorParameters_create()             # OpenCV 4.6 (apt de Bookworm)
    return lambda gris: cv2.aruco.detectMarkers(gris, dic, parameters=params)


class Aruco:
    """Prueba varios diccionarios hasta que uno detecta y desde ahi usa solo ese."""

    def __init__(self, modelo, lado=config.ARUCO_LADO_M):
        self.modelo = modelo
        self.lado = lado
        self.detectores = {n: _detector(n) for n in config.ARUCO_DICCIONARIOS if hasattr(cv2.aruco, n)}
        self.fijo = None
        h = lado / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    def detectar(self, gris):
        nombres = [self.fijo] if self.fijo else list(self.detectores)
        for n in nombres:
            esquinas, ids, _ = self.detectores[n](gris)
            if ids is not None and len(ids):
                self.fijo = n
                return self._pose(n, int(np.asarray(ids).ravel()[0]), np.asarray(esquinas[0]).reshape(4, 2))
        return None

    def _pose(self, dic, ident, esquinas):
        ok, rvec, tvec = cv2.solvePnP(self.obj, esquinas.astype(np.float32), self.modelo.matriz(), None,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        rot, _ = cv2.Rodrigues(rvec)
        centro = self.modelo.a_robot(tvec.ravel())
        x0, y0, z0 = self.modelo.a_robot((0, 0, 0))
        normal = [a - b for a, b in zip(self.modelo.a_robot(rot[:, 2]), (x0, y0, z0))]   # hacia la camara
        arriba = [a - b for a, b in zip(self.modelo.a_robot(-rot[:, 1]), (x0, y0, z0))]
        return {"dic": dic, "id": ident, "x": centro[0], "y": centro[1], "z": centro[2],
                "normal": normal, "arriba": arriba, "esquinas": esquinas}


# ---------------------------------------------------------------- hilo de vision
class Vision:
    """Lee la camara en su propio hilo y deja el ultimo resultado en self.resultado."""

    def __init__(self, registro=None, indice=None):
        self.cfg = cargar_config()
        self.registro = registro
        self.indice = self.cfg["camara"] if indice is None else indice
        self.modelo = ModeloCamara(self.cfg["ancho"], self.cfg["alto"], self.cfg["fov_horizontal_grados"])
        self.aruco = Aruco(self.modelo)
        self.buscar_aruco_siempre = False
        self.resultado = {"t": 0.0, "conos": [], "puerta": None, "aruco": None}
        self.ultimo_cuadro = None
        self.fps = 0.0
        self._vivo = threading.Event()
        self._cap = None

    def iniciar(self):
        self._cap = abrir_camara(self.indice, self.cfg["ancho"], self.cfg["alto"],
                                 config.CAMARA_PROCESAR_HZ, self.cfg["autofoco"])
        if self._cap is None:
            raise RuntimeError("No hay camara en %s" % self.indice)
        if self.registro:
            self.registro.evento("Camara: " + describir_camara(self._cap))
        self._vivo.set()
        threading.Thread(target=self._bucle, name="vision", daemon=True).start()
        return self

    def detener(self):
        self._vivo.clear()

    def procesar(self, cuadro, t, n=0):
        conos, _ = detectar_conos(cuadro, self.cfg)
        ubicar_conos(conos, self.modelo, self.cfg["alto_cono_m"])
        puerta = buscar_puerta(conos)
        aruco = None
        if self.buscar_aruco_siempre or n % config.ARUCO_CADA_N_CUADROS == 0:
            aruco = self.aruco.detectar(cv2.cvtColor(cuadro, cv2.COLOR_BGR2GRAY))
        self.resultado = {"t": t, "conos": conos, "puerta": puerta, "aruco": aruco}
        return self.resultado

    def _bucle(self):
        n, t_fps, cuadros, fallos = 0, time.monotonic(), 0, 0
        while self._vivo.is_set():
            ok, cuadro = self._cap.read()
            t = time.monotonic()
            if not ok:
                fallos += 1
                if fallos in (5, 50) and self.registro:
                    self.registro.evento("Camara sin imagen (%d lecturas fallidas)" % fallos)
                time.sleep(0.05)
                continue
            fallos = 0
            self.ultimo_cuadro = cuadro
            self.procesar(cuadro, t, n)
            n += 1
            cuadros += 1
            if t - t_fps >= 2:
                self.fps, cuadros, t_fps = cuadros / (t - t_fps), 0, t
