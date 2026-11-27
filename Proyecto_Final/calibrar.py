#!/usr/bin/env python3
"""Calibraciones que faltan antes de las pruebas de navegacion (ver README, plan de pruebas).

    python3 calibrar.py ff          # robot levantado: feedforward de los motores con la bateria real
    python3 calibrar.py pi          # robot levantado: referencia vs RPM medida (verifica el ff)
    python3 calibrar.py odometria   # en el piso: recta de 2 m y dos vueltas -> RADIO_RUEDA y L_GEOM
    python3 calibrar.py lag         # canto vertical de una caja a ~0.8 m al frente -> LAG del servo
    python3 calibrar.py sharp       # objeto plano a distancias conocidas -> tablas de los Sharp
    python3 calibrar.py camara --distancia 1.5 --lateral 0.5   # un cono -> FOV e inclinacion

Cada una imprime los valores para copiar en el firmware o en config.py y guarda sus datos en
registros/<fecha_hora>/ (CSV de la calibracion + lineas crudas del USB). Funciona en la Pi o en
una laptop con el ESP32 por USB (--loco COM5 / --perc COM6). Guia: GUIA_PRUEBAS.md.
"""
import csv
import os
import argparse
import math
import statistics
import time

import config
import protocolo as P
from diagnostico import abrir, agregar_puertos, leer_puertos
from registro import Registro


def guardar_csv(reg, nombre, columnas, filas):
    ruta = os.path.join(reg.carpeta, nombre)
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(columnas)
        w.writerows(filas)
    print("Datos guardados en", ruta)
    return ruta


def esperar_enter(texto):
    input(texto + "  [Enter] ")


# ------------------------------------------------------------------ feedforward
def cmd_ff(args):
    reg = Registro()
    odom, _, loco, _ = abrir(reg, ("LOCO",))
    esperar_enter("Robot LEVANTADO, ruedas libres, bateria de motores cargada.")
    loco.enviar("A", 1)
    time.sleep(0.3)
    pwms = list(range(args.desde, 256, args.paso))
    if pwms[-1] != 255:
        pwms.append(255)
    if args.bajando:
        # De 255 hacia abajo sin detener las ruedas: mide la friccion en movimiento (la rueda
        # sigue girando con menos PWM del que necesita para arrancar) y el PWM donde se detiene.
        pwms = list(range(255, args.desde - 1, -args.paso))
    resultado = {1: {}, -1: {}}
    filas = []
    try:
        for sentido in (1, -1):
            for pwm in pwms:
                c0, t0 = None, None
                fin = time.monotonic() + 2.0
                while time.monotonic() < fin:
                    loco.enviar("P", *([sentido * pwm] * 4))
                    if c0 is None and time.monotonic() > fin - 1.0:     # mide el ultimo segundo
                        c0, t0 = list(loco.conteos), time.monotonic()
                    time.sleep(0.1)
                c1, t1 = list(loco.conteos), time.monotonic()
                rpm = [abs(b - a) / n * 60 / (t1 - t0) for a, b, n in zip(c0, c1, config.CPR)]
                resultado[sentido][pwm] = rpm
                filas.append([sentido, pwm] + ["%.2f" % r for r in rpm])
                print("%s PWM %3d -> RPM %s" % ("+" if sentido > 0 else "-", pwm, " ".join("%6.1f" % r for r in rpm)))
            loco.parar()
            time.sleep(1.5)
    finally:
        loco.parar()
        loco.enviar("A", 0)
        reg.vaciar(forzar=True)
        guardar_csv(reg, "ff_bajando.csv" if args.bajando else "ff.csv", ["sentido", "pwm", "rpm_FL", "rpm_FR", "rpm_RL", "rpm_RR"], filas)
        reg.cerrar()
    if not args.bajando:
        imprimir_ff(resultado)


def interpolar_pwm(tabla, rueda, rpm_obj):
    pts = sorted((r[rueda], p) for p, r in tabla.items())
    for (r0, p0), (r1, p1) in zip(pts, pts[1:]):
        if r0 <= rpm_obj <= r1 and r1 > r0:
            return p0 + (rpm_obj - r0) / (r1 - r0) * (p1 - p0)
    return None


def imprimir_ff(res):
    rpms = (80.0, 100.0, 120.0)
    print("\n// Pegar en locomocion.ino (reemplaza las tablas):")
    filas = []
    for i in range(4):
        v = [interpolar_pwm(res[1], i, r) for r in rpms]
        filas.append("{%s}" % ", ".join("%.1ff" % x if x is not None else "255.0f /*no alcanza*/" for x in v))
    print("const float PWM_FF_FWD[4][N_FF_FWD] = {\n  %s};" % ",\n  ".join(filas))
    pwm_rev = (195, 210, 225, 240, 255)
    filas = []
    for i in range(4):
        vals = []
        for p in pwm_rev:
            cerca = min(res[-1], key=lambda q: abs(q - p))
            vals.append("%.2ff" % res[-1][cerca][i])
        filas.append("{%s}" % ", ".join(vals))
    print("const float RPM_REV[4][N_REV] = {\n  %s};" % ",\n  ".join(filas))
    print("// Si alguna rueda no llega a 120 RPM hacia adelante, bajar RPM_MAX (firmware y config.py).")
    print("// Para tener feedforward a baja velocidad, repetir con --desde 60 y revisar donde empieza a girar.")


# ------------------------------------------------------------------ seguimiento del PI
PRUEBAS_PI = [  # (nombre, vx m/s, vy m/s, wz grados/s)
    ("adelante_0.10", 0.10, 0, 0), ("adelante_0.20", 0.20, 0, 0), ("adelante_0.28", 0.28, 0, 0),
    ("atras_0.20", -0.20, 0, 0), ("izquierda_0.15", 0, 0.15, 0), ("derecha_0.15", 0, -0.15, 0),
    ("giro_60", 0, 0, 60), ("giro_-60", 0, 0, -60),
]


def cmd_pi(args):
    reg = Registro()
    odom, _, loco, _ = abrir(reg, ("LOCO",))
    esperar_enter("Robot LEVANTADO (o en el piso con %.1f m libres por prueba)." % 0.9)
    loco.enviar("A", 1)
    filas, resumen = [], []
    try:
        for nombre, vx, vy, wz in PRUEBAS_PI:
            t0 = time.monotonic()
            previo, ultimo = None, None
            errores = [[] for _ in range(4)]
            while time.monotonic() - t0 < args.segundos:
                loco.velocidad(vx, vy, math.radians(wz))
                c, t = loco.conteos, time.monotonic()
                if c is not None and c is not ultimo:
                    if previo is not None and t > previo[1]:
                        rpm = [(a - b) / n * 60 / (t - previo[1]) for a, b, n in zip(c, previo[0], config.CPR)]
                        filas.append([nombre, "%.3f" % (t - t0)] + list(loco.ref_rpm) + ["%.1f" % r for r in rpm] + list(loco.pwm))
                        if t - t0 > args.segundos / 2:
                            for i in range(4):
                                if abs(loco.ref_rpm[i]) > 5:
                                    errores[i].append((rpm[i] - loco.ref_rpm[i]) / abs(loco.ref_rpm[i]) * 100)
                    previo, ultimo = (list(c), t), c
                time.sleep(config.PERIODO_CMD_S)
            loco.velocidad(0, 0, 0)
            time.sleep(1.2)
            e = ["%+5.1f%%" % statistics.mean(x) if x else "  -  " for x in errores]
            resumen.append((nombre, e))
            print("%-15s error medio FL %s FR %s RL %s RR %s" % (nombre, *e))
    finally:
        loco.parar()
        loco.enviar("A", 0)
        reg.vaciar(forzar=True)
        guardar_csv(reg, "pi.csv", ["prueba", "t_s", "ref_FL", "ref_FR", "ref_RL", "ref_RR",
                                    "rpm_FL", "rpm_FR", "rpm_RL", "rpm_RR", "pwm_FL", "pwm_FR", "pwm_RL", "pwm_RR"], filas)
        reg.cerrar()
    print("Criterio: |error medio| <= 5 %% en todas las ruedas. Error grande y con signo fijo = feedforward mal.")


# ------------------------------------------------------------------ odometria
def mover(loco, odom, vx, vy, wz, hasta):
    """Comanda hasta que hasta(odom) sea verdadero."""
    while not hasta(odom):
        loco.velocidad(vx, vy, wz)
        time.sleep(config.PERIODO_CMD_S)
    loco.velocidad(0, 0, 0)
    time.sleep(1.0)


def cmd_odometria(args):
    reg = Registro()
    odom, _, loco, _ = abrir(reg, ("LOCO",))
    print("Radio y L actuales: %.4f m, %.4f m (config.py)" % (config.RADIO_RUEDA, config.L_GEOM))
    loco.enviar("A", 1)
    try:
        esperar_enter("RECTA: robot en el piso con %.1f m libres al frente; marcar la posicion inicial." % args.distancia)
        x0 = odom.pose_odom()[0]
        mover(loco, odom, 0.15, 0, 0, lambda o: o.pose_odom()[0] - x0 >= args.distancia)
        d_odo = odom.pose_odom()[0] - x0
        d_real = float(input("Distancia REAL recorrida [m]: "))
        radio = config.RADIO_RUEDA * d_real / d_odo
        print("odometria %.3f m, real %.3f m -> RADIO_RUEDA = %.4f" % (d_odo, d_real, radio))
        esperar_enter("GIRO: marcar la orientacion inicial (cinta en el piso). Girara %d vueltas." % args.vueltas)
        th, previo = 0.0, odom.pose_odom()[2]

        def acumulado(o):
            nonlocal th, previo
            a = o.pose_odom()[2]
            th += math.atan2(math.sin(a - previo), math.cos(a - previo))
            previo = a
            return th >= 2 * math.pi * args.vueltas

        mover(loco, odom, 0, 0, math.radians(45), acumulado)
        extra = float(input("Grados REALES de mas (+) o de menos (-) respecto de %d vueltas: " % args.vueltas))
        real = 2 * math.pi * args.vueltas + math.radians(extra)
        # th_odo = R/(4L) * sum -> con el radio corregido, L_real = L * (th_odo * radio/R) / th_real
        l_geom = config.L_GEOM * (th * radio / config.RADIO_RUEDA) / real
        print("\nCopiar en config.py:\nRADIO_RUEDA = %.4f\nL_GEOM = %.4f" % (radio, l_geom))
        guardar_csv(reg, "odometria.csv", ["d_odo_m", "d_real_m", "th_odo_rad", "th_real_rad", "radio", "l_geom"],
                    [[d_odo, d_real, th, real, radio, l_geom]])
        print("(El firmware usa RADIO y L_GEOM para la cinematica inversa: si cambian mas de 5 %, "
              "actualizarlos tambien en locomocion.ino para que las velocidades pedidas sean reales.)")
    finally:
        loco.parar()
        loco.enviar("A", 0)
        reg.cerrar()


# ------------------------------------------------------------------ lag del servo
def borde(muestras):
    """Angulo del mayor salto de distancia entre muestras consecutivas (ordenadas por angulo)."""
    m = sorted(muestras)
    mejor = max(range(1, len(m)), key=lambda i: abs(m[i][1] - m[i - 1][1]))
    return (m[mejor][0] + m[mejor - 1][0]) / 2, abs(m[mejor][1] - m[mejor - 1][1])


def cmd_lag(args):
    reg = Registro()
    _, _, _, perc = abrir(reg, ("PERC",))
    esperar_enter("Caja con un canto vertical a ~0.8 m al frente, cerca de 0 grados, fondo lejano detras.")
    try:
        perc.enviar("Z", 0, 0)                     # angulo = consigna pura
        estaticas = []
        for a in [x * 0.5 for x in range(-30, 31)]:
            perc.fijar(a)
            time.sleep(0.25)
            t = perc.ultimo_t
            if t and not t[5] & P.T_INVALIDA:
                estaticas.append((a, t[3]))
        th_borde, salto = borde(estaticas)
        print("Borde estatico en %.2f grados (salto %d mm)" % (th_borde, salto))
        perc.barrer(args.amplitud, args.vel)
        sube, baja = [], []
        fin = time.monotonic() + args.segundos
        ultimo = None
        while time.monotonic() < fin:
            perc.keepalive()
            t = perc.ultimo_t
            if t and t is not ultimo and not t[5] & (P.T_INVALIDA | P.T_EXTREMO):
                (sube if ultimo is None or t[2] >= ultimo[2] else baja).append((t[2] / 100, t[3]))
                ultimo = t
            time.sleep(0.002)
        th_s, _ = borde(sube)
        th_b, _ = borde(baja)
        lag_ccw = abs(th_s - th_borde) / args.vel * 1000
        lag_cw = abs(th_b - th_borde) / args.vel * 1000
        print("Borde barriendo: subiendo %.2f, bajando %.2f grados" % (th_s, th_b))
        print("\nLAG antihorario %.0f ms, horario %.0f ms" % (lag_ccw, lag_cw))
        guardar_csv(reg, "lag.csv", ["fase", "angulo_deg", "dist_mm"],
                    [["estatico", a, d] for a, d in estaticas] + [["sube", a, d] for a, d in sube] +
                    [["baja", a, d] for a, d in baja])
        print("Probar ya: python3 diagnostico.py servo Z %.0f %.0f" % (lag_ccw, lag_cw))
        print("Dejar fijo en percepcion.ino: float lagCcwMs = %.0f, lagCwMs = %.0f;" % (lag_ccw, lag_cw))
        print("Aceptacion: repetir con los valores nuevos; los dos sentidos deben dar el borde a +-1-2 grados.")
    finally:
        perc.fijar(0)
        reg.cerrar()


# ------------------------------------------------------------------ Sharp
def cmd_sharp(args):
    reg = Registro()
    _, _, _, perc = abrir(reg, ("PERC",))
    tabla = {0: [], 1: []}
    filas = []
    try:
        for d in args.distancias:
            esperar_enter("Objeto plano (carton blanco) a %d mm de la cara de AMBOS Sharp." % d)
            mv = {0: [], 1: []}
            fin = time.monotonic() + 1.5
            ultimo = None
            while time.monotonic() < fin:
                s = perc.ultimo_s
                if s and s is not ultimo:
                    mv[0].append(s[4])
                    mv[1].append(s[5])
                    ultimo = s
                perc.keepalive()
                time.sleep(0.02)
            for i in (0, 1):
                tabla[i].append((statistics.median(mv[i]), d))
            filas += [[d, a, b] for a, b in zip(mv[0], mv[1])]
            print("  izq %.0f mV  der %.0f mV" % (statistics.median(mv[0]), statistics.median(mv[1])))
    finally:
        guardar_csv(reg, "sharp.csv", ["dist_mm", "izq_mv", "der_mv"], filas)
        reg.cerrar()
    print("\n// Pegar en percepcion.ino (mV crecientes):")
    print("const PuntoCal SHARP_TABLA[2][%d] = {" % len(args.distancias))
    for i in (0, 1):
        pts = sorted(tabla[i])
        print("  {%s}," % ", ".join("{%.0f, %d}" % p for p in pts))
    print("};\nconst int SHARP_N[2] = {%d, %d};" % (len(args.distancias), len(args.distancias)))


# ------------------------------------------------------------------ camara
def cmd_camara(args):
    from vision import Vision
    import geometria_robot as G
    v = Vision(Registro(activo=False)).iniciar()
    esperar_enter("Un solo cono a %.2f m al frente del centro del robot y %.2f m a la izquierda." % (args.distancia, args.lateral))
    time.sleep(2)
    us, bases = [], []
    for _ in range(20):
        r = v.resultado
        if len(r["conos"]) == 1 and not r["conos"][0]["cortado"]:
            c = r["conos"][0]
            us.append(c["x"] + c["w"] / 2)
            bases.append(c["y"] + c["h"])
        time.sleep(0.1)
    v.detener()
    if not us:
        raise SystemExit("No vi exactamente un cono completo. Revisar color (paso2) o posicion.")
    m = v.modelo
    u, vb = statistics.median(us), statistics.median(bases)
    x = args.distancia - G.CAMARA_X_MM / 1000
    ang = math.atan2(args.lateral, x)
    print("Pixel del cono: u=%.0f, base v=%.0f" % (u, vb))
    if abs(u - m.cx) > 20 and abs(ang) > math.radians(5):
        f = (m.cx - u) / math.tan(ang)
        print("FOV horizontal medido: %.1f grados -> CAMARA_FOV_H_DEG (config.py) y fov_horizontal_grados (config_conos.json)"
              % math.degrees(2 * math.atan(m.ancho / 2 / f)))
    else:
        f = m.f
    dist = math.hypot(x, args.lateral) - 0.05                       # a la base frontal del cono
    yn = (vb - m.cy) / f
    incl = math.degrees(math.atan2(m.h, dist) - math.atan(yn))
    print("Inclinacion medida: %.1f grados hacia abajo (con altura de camara %.3f m) -> CAMARA_INCLINACION_DEG"
          % (incl, m.h))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("ff")
    f.add_argument("--desde", type=int, default=60, help="PWM inicial (bajo, para ver la zona muerta)")
    f.add_argument("--paso", type=int, default=15)
    f.add_argument("--bajando", action="store_true", help="de 255 hacia abajo sin detener (zona muerta en movimiento)")
    f.set_defaults(fn=cmd_ff)
    q = sub.add_parser("pi")
    q.add_argument("--segundos", type=float, default=3.0)
    q.set_defaults(fn=cmd_pi)
    o = sub.add_parser("odometria")
    o.add_argument("--distancia", type=float, default=2.0)
    o.add_argument("--vueltas", type=int, default=2)
    o.set_defaults(fn=cmd_odometria)
    l = sub.add_parser("lag")
    l.add_argument("--amplitud", type=float, default=30)
    l.add_argument("--vel", type=float, default=config.BARRIDO_VEL_DPS)
    l.add_argument("--segundos", type=float, default=8)
    l.set_defaults(fn=cmd_lag)
    s = sub.add_parser("sharp")
    s.add_argument("--distancias", type=int, nargs="+", default=[100, 150, 200, 300, 400, 500, 600, 800])
    s.set_defaults(fn=cmd_sharp)
    c = sub.add_parser("camara")
    c.add_argument("--distancia", type=float, default=1.5)
    c.add_argument("--lateral", type=float, default=0.5)
    c.set_defaults(fn=cmd_camara)
    agregar_puertos(p)
    args = p.parse_args()
    leer_puertos(args)
    args.fn(args)


if __name__ == "__main__":
    main()
