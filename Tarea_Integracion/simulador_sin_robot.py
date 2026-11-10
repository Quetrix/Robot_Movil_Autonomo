#!/usr/bin/env python3
"""
Simulador para probar braitenberg.py sin el robot (PC o Raspberry Pi).

Imita, con la misma interfaz, a:
  - el ESP32 de locomocion (firmware locomocion_esp32_ir_frontal) en un puerto serial virtual (pty):
    comandos V / H / X / ?, timeout de H de 300 ms, recorte del avance con el IR frontal, mensajes
    "# IR FRENTE: ..." y CSV con la columna irF;
  - el cabezal de percepcion por HTTP (GET /data, POST /action) y, opcionalmente, por serial (DETAIL);
  - un mundo 2D con obstaculos: el robot se mueve con la velocidad que recibe y los Sharp (+-60 deg),
    el TF-Luna (0 deg) y el KY-032 se calculan por trazado de rayos.

Uso (dos terminales):
  python3 simulador_sin_robot.py --escenario izq
      -> imprime el puerto de locomocion (/dev/pts/N) y la URL del cabezal
  python3 braitenberg.py --loco /dev/pts/N --perc http://127.0.0.1:8080

Escenarios: izq, der, frente, arena. --traza archivo.csv guarda x, y, theta a 20 Hz.
Solo Linux/macOS (usa pty).
"""
import argparse
import json
import math
import os
import pty
import random
import select
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

R_RUEDA, L_GEOM, RPM_MAX = 0.024, 0.15, 120.0
RADIO_ROBOT = 0.11          # para detectar choques (chasis 120 x 180 mm + ruedas)
IR_ALCANCE = 0.18           # KY-032: detecta si el obstaculo esta a menos de esto del centro [m]
ACEL = 1.6                  # m/s^2, aprox. 650 RPM/s del frenado del firmware

ESCENARIOS = {
    # circulos (x, y, r) y segmentos ((x1, y1), (x2, y2)); el robot arranca en (0, 0) mirando a +y
    "izq": {"circulos": [(-0.30, 0.45, 0.08)], "muros": []},
    "der": {"circulos": [(0.30, 0.45, 0.08)], "muros": []},
    "frente": {"circulos": [(0.0, 0.70, 0.10)], "muros": []},
    "caja_izq": {"circulos": [(-0.40, 0.55, 0.15)], "muros": []},
    "arena": {
        "circulos": [(-0.35, 0.60, 0.08), (0.40, 1.10, 0.08)],
        "muros": [((-1.0, -0.5), (1.0, -0.5)), ((1.0, -0.5), (1.0, 1.7)),
                  ((1.0, 1.7), (-1.0, 1.7)), ((-1.0, 1.7), (-1.0, -0.5))],
    },
}


def rayo(px, py, ang, mundo, alcance=9.0):
    dx, dy = math.cos(ang), math.sin(ang)
    mejor = alcance
    for cx, cy, r in mundo["circulos"]:
        fx, fy = px - cx, py - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if 0 < t < mejor:
                mejor = t
    for (x1, y1), (x2, y2) in mundo["muros"]:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if 0 < t < mejor and 0 <= u <= 1:
            mejor = t
    return mejor


def holgura(px, py, mundo):
    h = 9.0
    for cx, cy, r in mundo["circulos"]:
        h = min(h, math.hypot(px - cx, py - cy) - r)
    for (x1, y1), (x2, y2) in mundo["muros"]:
        ex, ey = x2 - x1, y2 - y1
        u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
        h = min(h, math.hypot(px - x1 - u * ex, py - y1 - u * ey))
    return h


def limitar_ruedas(vx, vy, w_ccw):
    """Misma cinematica y escalado a 120 RPM que el firmware; devuelve la velocidad resultante."""
    wcw = -w_ccw
    ruedas = [(vy + vx + L_GEOM * wcw), (vy - vx - L_GEOM * wcw),
              (vy - vx + L_GEOM * wcw), (vy + vx - L_GEOM * wcw)]
    rpm_max = max(abs(r) for r in ruedas) / R_RUEDA * 60 / (2 * math.pi)
    k = RPM_MAX / rpm_max if rpm_max > RPM_MAX else 1.0
    return vx * k, vy * k, w_ccw * k


class Mundo:
    def __init__(self, escenario, ruido_mm):
        self.m = ESCENARIOS[escenario]
        self.x, self.y, self.th = 0.0, 0.0, math.pi / 2
        self.cmd = [0.0, 0.0, 0.0]        # vx, vy, w pedidos
        self.vel = [0.0, 0.0, 0.0]        # vx, vy, w actuales
        self.ruido = ruido_mm
        self.choques = 0
        self.min_holgura = 9.0
        self.lock = threading.Lock()

    def distancias(self):
        with self.lock:
            x, y, th = self.x, self.y, self.th
        return (rayo(x, y, th + math.radians(60), self.m),
                rayo(x, y, th - math.radians(60), self.m),
                rayo(x, y, th, self.m))

    def avanzar(self, dt):
        with self.lock:
            for i in range(3):
                paso = ACEL * dt if i < 2 else ACEL / L_GEOM * dt
                e = self.cmd[i] - self.vel[i]
                self.vel[i] += max(-paso, min(paso, e))
            vx, vy, w = self.vel
            self.th += w * dt
            self.x += (vy * math.cos(self.th) + vx * math.sin(self.th)) * dt
            self.y += (vy * math.sin(self.th) - vx * math.cos(self.th)) * dt
            h = holgura(self.x, self.y, self.m) - RADIO_ROBOT
            if h < 0 and self.min_holgura >= 0:
                self.choques += 1
            self.min_holgura = min(self.min_holgura, h)


class LocoFalso:
    """ESP32 de locomocion falso sobre un pty."""

    def __init__(self, mundo, con_ir=True):
        self.mundo = mundo
        self.con_ir = con_ir
        self.maestro, esclavo = pty.openpty()
        tty.setraw(esclavo)
        self.esclavo = esclavo                     # se mantiene abierto para evitar EIO
        self.ruta = os.ttyname(esclavo)
        self.hold = False
        self.t_hold = 0.0
        self.ir = False
        self.buf = b""
        self.activo = False
        self.t_csv = 0.0

    def decir(self, txt):
        try:
            os.write(self.maestro, (txt + "\r\n").encode())
        except OSError:
            pass

    def aplicar(self, vx, vy, w):
        if self.ir and vy > 0:
            vy = 0.0
        self.mundo.cmd = list(limitar_ruedas(vx, vy, w))
        self.activo = any(abs(c) > 1e-4 for c in self.mundo.cmd)

    def linea(self, ln):
        p = ln.split()
        if not p:
            return
        c = p[0].upper()
        if c == "X":
            self.hold = False
            self.mundo.cmd = [0.0, 0.0, 0.0]
            self.mundo.vel = [0.0, 0.0, 0.0]
            self.activo = False
            self.decir("# ROBOT DETENIDO")
        elif c == "?":
            self.decir("# Use: W S A D [ms] | R | X")
            self.decir("#      V vx vy omega [ms] | H vx vy omega")
            if self.con_ir:
                self.decir("# IR frontal (GPIO34) activo: se anula vy > 0")
        elif c in ("V", "H") and len(p) >= 4:
            vx, vy, w = (float(v) for v in p[1:4])
            if self.ir and vy > 0 and self.con_ir:
                vy = 0.0
            if abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(w) < 1e-4:
                if self.activo:
                    self.decir("# FRENADO PI")
                self.hold = False
                self.aplicar(0, 0, 0)
                return
            if c == "H":
                if not self.hold:
                    self.decir(f"# H MANTENER vx={vx:.3f} vy={vy:.3f} omega={w:.3f}")
                self.hold, self.t_hold = True, time.monotonic()
            else:
                self.hold = False
            self.aplicar(vx, vy, w)
        else:
            self.decir("# COMANDO INVALIDO")

    def servicio(self):
        if select.select([self.maestro], [], [], 0)[0]:
            try:
                self.buf += os.read(self.maestro, 1024)
            except OSError:
                return
            while b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                self.linea(ln.decode(errors="ignore").strip())
        ahora = time.monotonic()
        if self.hold and ahora - self.t_hold > 0.3:
            self.hold = False
            self.decir("# H SOLTADO")
            self.aplicar(0, 0, 0)
        if self.con_ir:
            ir = self.mundo.distancias()[2] < IR_ALCANCE
            if ir != self.ir:
                self.ir = ir
                self.decir("# IR FRENTE: OBSTACULO" if ir else "# IR FRENTE: LIBRE")
                if ir and self.mundo.cmd[1] > 0:
                    self.mundo.cmd[1] = 0.0
                    self.decir("# IR FRENTE: AVANCE ANULADO")
        if self.activo and ahora - self.t_csv >= 0.2:
            self.t_csv = ahora
            campos = [str(int(ahora * 1000))] + ["0.0"] * 12 + ["0.0"]
            if self.con_ir:
                campos.append("1" if self.ir else "0")
            self.decir(",".join(campos + ["0", "0"]))


class Cabezal:
    """Cabezal falso: capturas cada 180 ms, igual que el firmware en modo hold."""

    def __init__(self, mundo):
        self.mundo = mundo
        self.seq = 0
        self.t_cap = 0.0
        self.attached = False
        self.dist = [None, None, None]
        self.estado = ["sin_datos"] * 3
        self.volts = [None, None]
        self.lock = threading.Lock()

    def actualizar(self):
        ahora = time.monotonic()
        if ahora - self.t_cap < 0.18:
            return
        self.t_cap = ahora
        di, dd, dt = (d * 1000 for d in self.mundo.distancias())
        dist, est, volts = [], [], []
        for d in (di, dd):
            d += random.gauss(0, self.mundo.ruido)
            if 100 <= d <= 800:
                dist.append(round(d, 1)); est.append("ok"); volts.append(round(27.0 / (d / 10 + 0.42) * 0.9, 4))
            else:
                dist.append(None); est.append("IR_fuera_de_rango"); volts.append(2.6 if d < 100 else 0.3)
        dt += random.gauss(0, self.mundo.ruido / 2)
        if 200 <= dt <= 8000:
            dist.append(round(dt, 1)); est.append("ok")
        else:
            dist.append(None); est.append("TF_menos_de_20cm_no_fiable" if 0 < dt < 200 else "TF_rango_o_calidad_invalida")
        with self.lock:
            self.dist, self.estado, self.volts = dist, est, volts
            self.seq += 1

    def json(self):
        with self.lock:
            m = [{"valid": d is not None, "pose": d is not None, "preliminary": i < 2,
                  "mm": d, "age_ms": 50} for i, d in enumerate(self.dist)]
            return json.dumps({
                "version": "simulador", "ms": int(time.monotonic() * 1000), "seq": self.seq,
                "head_command_deg": 0.0 if self.attached else None, "attached": self.attached,
                "mode": "hold", "state": "captura_completa", "samples": m,
                "sensor_status": self.estado, "ir_volts": self.volts, "tf_native_mm": self.dist[2],
            })

    def detail(self):
        with self.lock:
            filas = []
            for k, d, e in zip(("IR_I", "IR_D", "TF"), self.dist, self.estado):
                txt = "--" if d is None else (("~" if k != "TF" else "") + f"{d:.1f}")
                filas.append(f"{k};0.0;0.0;{txt};{txt.replace('~', '')};--;--;50;{e}")
            v = ["--" if x is None else f"{x:.4f}" for x in self.volts]
            return ("\n# Captura %d t=%dms cabezal_cmd=%sdeg modo=hold\n" % (
                self.seq, int(time.monotonic() * 1000), "0.0" if self.attached else "--")
                + "sensor;head_cmd_deg;rayo_est_deg;d_sensor_mm;r_eje_mm;x_est_mm;y_est_mm;edad_ms;estado\n"
                + "\n".join(filas) + "\nFINAL;--;--;--;--;--;--;--;sin_TF_valido_o_pose\n"
                + "FUENTES PARA LA DIRECCION TF: sensor;r_eje_mm;edad_ms;peso_efectivo;uso\n"
                + "IR_I;--;--;0.0000;x\nIR_D;--;--;0.0000;x\nTF;--;--;0.0000;x\n"
                + f"IR_volts={v[0]},{v[1]} ADC_mV=--,-- TF_crudo_mm=--\n")


def servidor_http(cabezal, puerto):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _resp(self, code, cuerpo, tipo):
            b = cuerpo.encode()
            self.send_response(code)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.startswith("/data"):
                self._resp(200, cabezal.json(), "application/json")
            else:
                self._resp(404, "no", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            cmd = parse_qs(self.rfile.read(n).decode()).get("cmd", [""])[0].upper()
            if cmd == "CENTER":
                cabezal.attached = True
                self._resp(200, "Centro solicitado (simulador).", "text/plain")
            else:
                self._resp(400, "Comando no valido.", "text/plain")

    srv = ThreadingHTTPServer(("127.0.0.1", puerto), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


class CabezalSerial:
    def __init__(self, cabezal):
        self.c = cabezal
        self.maestro, esclavo = pty.openpty()
        tty.setraw(esclavo)
        self.esclavo = esclavo
        self.ruta = os.ttyname(esclavo)
        self.buf = b""
        self.t_matriz = 0.0

    def servicio(self):
        if select.select([self.maestro], [], [], 0)[0]:
            try:
                self.buf += os.read(self.maestro, 1024)
            except OSError:
                return
            while b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                cmd = ln.decode(errors="ignore").strip().upper()
                if cmd == "CENTER":
                    self.c.attached = True
                elif cmd == "DETAIL":
                    os.write(self.maestro, self.c.detail().encode())
        if time.monotonic() - self.t_matriz > 0.5:           # ruido: la matriz periodica
            self.t_matriz = time.monotonic()
            os.write(self.maestro, b"\n# MATRIZ FINAL captura=1 t=1ms vigencia=12000ms\n"
                                   b"angulo_estimado_deg;distancia_ponderada_mm\n0.00;500.0\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--escenario", choices=sorted(ESCENARIOS), default="arena")
    ap.add_argument("--puerto-http", type=int, default=8080)
    ap.add_argument("--traza", help="CSV con t,x,y,theta_deg,holgura a 20 Hz")
    ap.add_argument("--ruido-mm", type=float, default=5.0)
    ap.add_argument("--sin-ir", action="store_true", help="imitar el firmware sin_IR")
    ap.add_argument("--duracion", type=float, default=0, help="segundos (0 = hasta Ctrl+C)")
    a = ap.parse_args()

    mundo = Mundo(a.escenario, a.ruido_mm)
    loco = LocoFalso(mundo, con_ir=not a.sin_ir)
    cab = Cabezal(mundo)
    servidor_http(cab, a.puerto_http)
    cab_ser = CabezalSerial(cab)
    print(f"LOCO {loco.ruta}", flush=True)
    print(f"PERC_HTTP http://127.0.0.1:{a.puerto_http}", flush=True)
    print(f"PERC_SERIAL {cab_ser.ruta}", flush=True)
    traza = open(a.traza, "w") if a.traza else None
    if traza:
        traza.write("t,x,y,theta_deg,holgura,vy_cmd,w_cmd\n")
    t0 = t_prev = t_tr = time.monotonic()
    try:
        while not a.duracion or time.monotonic() - t0 < a.duracion:
            ahora = time.monotonic()
            mundo.avanzar(ahora - t_prev)
            t_prev = ahora
            loco.servicio()
            cab.actualizar()
            cab_ser.servicio()
            if traza and ahora - t_tr >= 0.05:
                t_tr = ahora
                h = holgura(mundo.x, mundo.y, mundo.m) - RADIO_ROBOT
                traza.write(f"{ahora - t0:.2f},{mundo.x:.3f},{mundo.y:.3f},{math.degrees(mundo.th):.1f},"
                            f"{h:.3f},{mundo.cmd[1]:.3f},{mundo.cmd[2]:.3f}\n")
                traza.flush()
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    print(f"FIN x={mundo.x:.2f} y={mundo.y:.2f} theta={math.degrees(mundo.th):.0f} deg "
          f"holgura_min={mundo.min_holgura:.3f} m choques={mundo.choques}", flush=True)


if __name__ == "__main__":
    main()
