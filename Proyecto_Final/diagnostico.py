#!/usr/bin/env python3
"""Diagnostico y pruebas manuales desde la Raspberry Pi (por SSH).

    python3 diagnostico.py puertos              # que ESP32 hay en cada USB
    python3 diagnostico.py salud                # alimentacion, temperatura y servicios de la Pi
    python3 diagnostico.py monitor [--barrer]   # panel en vivo de ambos enlaces (y lo graba)
    python3 diagnostico.py teleop               # mover el robot con el teclado (lo graba)
    python3 diagnostico.py servo B 60 150       # comando directo a percepcion: B / F / R / Z
    python3 diagnostico.py camara [--fotos 5]   # fps reales y conos vistos, guarda fotos

En monitor y teleop todo queda en registros/<fecha_hora>/ como en robot.py.
"""
import argparse
import math
import os
import select
import subprocess
import sys
import time

import config
from enlaces import Locomocion, Percepcion, buscar_puertos, candidatos, identificar
from falda import Falda
from odometria import Odometria
from registro import Registro


PUERTOS = {}   # --loco / --perc de la linea de comandos


def abrir(reg, cual=("LOCO", "PERC")):
    rutas = {"LOCO": PUERTOS.get("LOCO") or config.PUERTO_LOCO, "PERC": PUERTOS.get("PERC") or config.PUERTO_PERC}
    if any(rutas[k] is None for k in cual):
        enc = buscar_puertos()
        rutas = {k: rutas[k] or enc.get(k) for k in rutas}
    odom = Odometria()
    falda = Falda(odom.pose_odom)
    loco = Locomocion(rutas["LOCO"], reg, odom).iniciar() if "LOCO" in cual and rutas["LOCO"] else None
    perc = Percepcion(rutas["PERC"], reg, falda).iniciar() if "PERC" in cual and rutas["PERC"] else None
    if not loco and "LOCO" in cual:
        print("!! no encontre el ESP32 de locomocion")
    if not perc and "PERC" in cual:
        print("!! no encontre el ESP32 de percepcion")
    return odom, falda, loco, perc


def cmd_puertos(_):
    for ruta in candidatos() or ["(ningun puerto serial)"]:
        quien = identificar(ruta) if os.path.exists(ruta) else None
        print("%-70s %s" % (ruta, quien or "sin respuesta (otro firmware, ocupado o sin permiso)"))
    print("\nPara fijarlos, copiar la ruta /dev/serial/by-id/... en PUERTO_LOCO / PUERTO_PERC de config.py")


def salud():
    """(texto, problemas) del estado de la Pi."""
    out, problemas = [], []
    try:
        t = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
        out.append("CPU %.0f C" % t)
        if t > 80:
            problemas.append("temperatura alta")
    except OSError:
        pass
    try:
        r = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
        v = int(r.strip().split("=")[1], 16)
        out.append("throttled=0x%x" % v)
        if v & 0x1:
            problemas.append("BAJO VOLTAJE AHORA")
        elif v & 0x10000:
            problemas.append("hubo bajo voltaje desde el arranque")
        if v & 0x4:
            problemas.append("CPU limitada")
    except Exception:
        pass
    try:
        out.append("carga %.2f" % os.getloadavg()[0])
    except OSError:
        pass
    return "  ".join(out), problemas


def cmd_salud(_):
    texto, problemas = salud()
    print(texto)
    for p in problemas:
        print("!!", p)
    try:
        txt = open("/boot/firmware/config.txt").read()
        if "usb_max_current_enable=1" not in txt:
            print("!! /boot/firmware/config.txt sin usb_max_current_enable=1: con fuente de 3 A la Pi 5 "
                  "limita el USB a 600 mA en total (camara + 2 ESP32 + sensores). Ver README.")
    except OSError:
        pass
    for servicio in ("ModemManager", "brltty"):
        r = subprocess.run(["systemctl", "is-active", servicio], capture_output=True, text=True)
        if r.stdout.strip() == "active":
            print("!! %s activo: puede abrir los /dev/ttyUSB y mandar basura a los ESP32. "
                  "sudo systemctl disable --now %s" % (servicio, servicio))
    r = subprocess.run(["id", "-nG"], capture_output=True, text=True)
    if "dialout" not in r.stdout.split():
        print("!! el usuario no esta en el grupo dialout: sudo usermod -aG dialout $USER (y volver a entrar)")


def tasas(enlace, previo, dt):
    s = enlace.stats
    tasa = (s["ok"] - previo.get("ok", 0)) / dt
    previo.update(s)
    return tasa


def cmd_monitor(args):
    reg = Registro(activo=not args.sin_registro)
    odom, falda, loco, perc = abrir(reg)
    if perc and args.barrer:
        perc.barrer()
    previos = ({}, {})
    t_previo = time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            if perc:
                perc.keepalive()      # sin esto el cabezal vuelve al centro a los 3 s
            ahora = time.monotonic()
            dt, t_previo = ahora - t_previo, ahora
            lineas = [time.strftime("%H:%M:%S") + "  " + salud()[0]]
            if loco:
                rpm = ["%5.0f" % v for v in ruedas_rpm(odom)]
                lineas.append("LOCO %-4s %5.1f tramas/s  malas %d perdidas %d  armado %d IRf %d flags 0x%02x" % (
                    "ok" if loco.conectado() else "--", tasas(loco, previos[0], dt), loco.stats["malas"],
                    loco.stats["perdidas"], loco.armado, loco.ir_frente, loco.flags))
                lineas.append("     ref rpm %s | med rpm %s | pwm %s" % (
                    " ".join("%5d" % v for v in loco.ref_rpm), " ".join(rpm), " ".join("%4d" % v for v in loco.pwm)))
                if loco.ultimo_d:
                    d = loco.ultimo_d
                    lineas.append("     ESP32: lazo max %d us  rx_malas %d  tx_perdidas %d  watchdogs %d  stalls %d" % tuple(d[2:7]))
                p = odom.pose_odom()
                lineas.append("     odometria x %.3f y %.3f th %.1f  recorrido %.2f m" % (p[0], p[1], math.degrees(p[2]), odom.recorrido))
            if perc:
                lineas.append("PERC %-4s %5.1f tramas/s  malas %d perdidas %d  IR trasero %d" % (
                    "ok" if perc.conectado() else "--", tasas(perc, previos[1], dt), perc.stats["malas"],
                    perc.stats["perdidas"], perc.ir_trasero))
                if perc.ultimo_d:
                    d = perc.ultimo_d
                    lineas.append("     ESP32: lazo max %d us  rx_malas %d  tx_perdidas %d  i2c_err %d  TF validas/s %d  modo %d" % tuple(d[2:8]))
                if perc.ultimo_t:
                    t = perc.ultimo_t
                    lineas.append("     TF %+6.1f deg %5d mm amp %5d flags %d | Sharp izq %s der %s mm" % (
                        t[2] / 100, t[3], t[4], t[5], fmt(falda.sharp_mm[0]), fmt(falda.sharp_mm[1])))
                m = falda.matriz()
                lineas.append("     falda: " + " ".join("%+d:%.2f" % (a, d) for a, d in m[::3]) if m else "     falda vacia")
            print("\n".join(lineas) + "\n", flush=True)
            reg.vaciar()
    except KeyboardInterrupt:
        pass
    finally:
        if perc:
            perc.fijar(0)
        reg.cerrar()


def fmt(v):
    return "----" if v is None else "%4.0f" % v


_rpm_previo = {}


def ruedas_rpm(odom):
    """RPM medidas a partir de los conteos que llegan (promedio entre llamadas)."""
    c, t = odom._previo, odom._t_previo
    if c is None:
        return [0, 0, 0, 0]
    p = _rpm_previo.get("v")
    _rpm_previo["v"] = (list(c), t)
    if not p or t <= p[1]:
        return [0, 0, 0, 0]
    return [(a - b) / n * 60 / (t - p[1]) for a, b, n in zip(c, p[0], config.CPR)]


def cmd_teleop(args):
    import termios
    import tty
    reg = Registro(activo=not args.sin_registro)
    odom, falda, loco, perc = abrir(reg, ("LOCO",) if args.sin_percepcion else ("LOCO", "PERC"))
    if not loco:
        return
    if perc:
        perc.barrer()
    print(__doc_teleop__)
    v = [0.0, 0.0, 0.0]
    paso_v, paso_w = 0.05, math.radians(15)
    fd = sys.stdin.fileno()
    viejo = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        t_print = 0
        while True:
            if select.select([sys.stdin], [], [], config.PERIODO_CMD_S)[0]:
                k = sys.stdin.read(1).lower()
                if k == "w": v[0] += paso_v
                elif k == "s": v[0] -= paso_v
                elif k == "a": v[1] += paso_v
                elif k == "d": v[1] -= paso_v
                elif k == "q": v[2] += paso_w
                elif k == "e": v[2] -= paso_w
                elif k in " x": v = [0.0, 0.0, 0.0]
                elif k == "r": loco.enviar("A", 0 if loco.armado else 1)
                elif k == "i": loco.enviar("Q", 0 if not (loco.flags & 64) else 1)
                elif k == "\x1b" or k == "z": break
                v[0] = max(-0.3, min(0.3, v[0]))
                v[1] = max(-0.3, min(0.3, v[1]))
                v[2] = max(-1.5, min(1.5, v[2]))
            loco.velocidad(*v)
            if perc:
                perc.keepalive()
            if time.monotonic() - t_print > 0.5:
                t_print = time.monotonic()
                p = odom.pose_odom()
                print("\rv=(%.2f %.2f %.0f)  armado %d IRf %d  pose %.2f %.2f %.0f  ref %s pwm %s     " % (
                    v[0], v[1], math.degrees(v[2]), loco.armado, loco.ir_frente, p[0], p[1], math.degrees(p[2]),
                    loco.ref_rpm, loco.pwm), end="", flush=True)
            reg.vaciar()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, viejo)
        loco.parar()
        if perc:
            perc.fijar(0)
        reg.cerrar()
        print()


__doc_teleop__ = """Teleoperacion (cada tecla suma un paso; el robot sigue hasta espacio):
  w/s adelante/atras   a/d izquierda/derecha   q/e giro antihorario/horario
  espacio o x = parar  r = armar/desarmar      i = IR frontal on/off   z o Esc = salir
El robot se mueve solo si esta armado (boton o tecla r)."""


def cmd_servo(args):
    reg = Registro(activo=False)
    _, _, _, perc = abrir(reg, ("PERC",))
    if perc:
        perc.enviar(args.comando, *args.valores)
        time.sleep(0.3)
        print("enviado", args.comando, args.valores, "| perc:", perc.stats)
        if args.comando in ("B", "F"):
            print("El ESP32 vuelve al centro si no recibe nada en 3 s (keepalive); para mantenerlo usar monitor.")


def cmd_camara(args):
    import cv2
    from vision import Vision
    reg = Registro(activo=True)
    v = Vision(reg).iniciar()
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.segundos:
        time.sleep(1)
        r = v.resultado
        print("%.1f fps  conos %d  %s  puerta %s  aruco %s" % (
            v.fps, len(r["conos"]),
            " ".join("(%.2f m %+.0f %s)" % (c["dist"], math.degrees(c["rumbo"]), c["metodo"]) for c in r["conos"]),
            "si" if r["puerta"] else "no", r["aruco"]["id"] if r["aruco"] else "no"), flush=True)
        if args.fotos and v.ultimo_cuadro is not None:
            args.fotos -= 1
            reg.foto(v.ultimo_cuadro, "camara")
    v.detener()
    reg.cerrar()
    print("Fotos en", reg.carpeta)


def agregar_puertos(p):
    p.add_argument("--loco", help="puerto del ESP32 de locomocion (p. ej. COM5); por defecto se busca")
    p.add_argument("--perc", help="puerto del ESP32 de percepcion")


def leer_puertos(args):
    PUERTOS.update({k: v for k, v in (("LOCO", args.loco), ("PERC", args.perc)) if v})


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("puertos").set_defaults(f=cmd_puertos)
    sub.add_parser("salud").set_defaults(f=cmd_salud)
    m = sub.add_parser("monitor")
    m.add_argument("--barrer", action="store_true")
    m.add_argument("--sin-registro", action="store_true")
    m.set_defaults(f=cmd_monitor)
    t = sub.add_parser("teleop")
    t.add_argument("--sin-percepcion", action="store_true")
    t.add_argument("--sin-registro", action="store_true")
    t.set_defaults(f=cmd_teleop)
    s = sub.add_parser("servo")
    s.add_argument("comando", choices=["B", "F", "R", "Z"])
    s.add_argument("valores", nargs="*", type=float)
    s.set_defaults(f=cmd_servo)
    c = sub.add_parser("camara")
    c.add_argument("--segundos", type=float, default=10)
    c.add_argument("--fotos", type=int, default=3)
    c.set_defaults(f=cmd_camara)
    agregar_puertos(p)
    args = p.parse_args()
    leer_puertos(args)
    args.f(args)


if __name__ == "__main__":
    main()
