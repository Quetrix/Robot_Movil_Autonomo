#!/usr/bin/env python3
"""Programa de la prueba final (Raspberry Pi 5).

    python3 robot.py                 # busca los dos ESP32 y la camara, espera el boton
    python3 robot.py --sin-camara    # solo odometria + falda (pruebas de pasillo)
    python3 robot.py --sim           # todo simulado en la PC, sin robot (ver sim.py)

Cada corrida queda grabada en registros/<fecha_hora>/ (ver registro.py).
Ctrl+C detiene el robot (X) y cierra los registros.
"""
import argparse
import math
import signal
import time

import config
from enlaces import Locomocion, Percepcion, buscar_puertos
from falda import Falda
from navegacion import Navegador
from odometria import Odometria
from registro import Registro


def construir(args, reg):
    odom = Odometria()
    falda = Falda(odom.pose_odom)
    if args.sim:
        import sim
        mundo = sim.Mundo(reg)
        loco = Locomocion(None, reg, odom, puerto=mundo.puerto_loco)
        perc = Percepcion(None, reg, falda, puerto=mundo.puerto_perc)
        vision = None if args.sin_camara else sim.VisionSim(mundo)
        mundo.iniciar(armar_en_s=1.0)
    else:
        rutas = {"LOCO": config.PUERTO_LOCO, "PERC": config.PUERTO_PERC}
        if None in rutas.values():
            encontrados = buscar_puertos()
            rutas = {k: v or encontrados.get(k) for k, v in rutas.items()}
        reg.evento("Puertos: %s" % rutas)
        if None in rutas.values():
            raise SystemExit("Falta un ESP32 (%s). Revisar USB o usar diagnostico.py puertos." % rutas)
        loco = Locomocion(rutas["LOCO"], reg, odom)
        perc = Percepcion(rutas["PERC"], reg, falda)
        vision = None
        if not args.sin_camara:
            from vision import Vision
            vision = Vision(reg, args.camara).iniciar()
    loco.iniciar()
    perc.iniciar()
    loco.enviar("Q", 1 if config.IR_FRENTE_HABILITADO else 0)
    return odom, falda, loco, perc, vision


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sin-camara", action="store_true")
    p.add_argument("--sim", action="store_true")
    p.add_argument("--camara", default=None, help="indice o ruta de la camara")
    p.add_argument("--sin-registro", action="store_true")
    p.add_argument("--max-s", type=float, default=0, help="terminar despues de este tiempo (pruebas)")
    args = p.parse_args()

    reg = Registro(activo=not args.sin_registro)
    if reg.carpeta:
        reg.evento("Registro en " + reg.carpeta)
    odom, falda, loco, perc, vision = construir(args, reg)
    nav = Navegador(loco, perc, vision, odom, falda, reg)

    corriendo = [True]
    signal.signal(signal.SIGINT, lambda *_: corriendo.__setitem__(0, False))
    signal.signal(signal.SIGTERM, lambda *_: corriendo.__setitem__(0, False))
    reg.evento("Listo. Presionar el boton para arrancar.")
    t_ini = t_estado = time.monotonic()
    n = 0
    try:
        while corriendo[0]:
            t0 = time.monotonic()
            nav.paso()
            n += 1
            if n % 2 == 0:
                reg.nav(nav.fila())
            if t0 - t_estado >= 1.0:
                t_estado = t0
                pm = odom.pose_mundo()
                print("%-10s obj %d  pose %.2f %.2f %4.0f  libre %4.2f  loco %s perc %s  conos %d" % (
                    nav.estado, nav.i, pm[0], pm[1], math.degrees(pm[2]), min(nav.libre, 9.99),
                    "ok" if loco.conectado() else "--", "ok" if perc.conectado() else "--",
                    len(vision.resultado["conos"]) if vision else -1), flush=True)
            reg.vaciar()
            if nav.estado == "FIN" and args.sim or (args.max_s and t0 - t_ini > args.max_s):
                break
            time.sleep(max(0.0, config.PERIODO_CMD_S - (time.monotonic() - t0)))
    finally:
        loco.parar()
        perc.enviar("F", 0)
        reg.evento("Fin. Estado %s, objetivo %d. Enlaces: loco %s perc %s" % (
            nav.estado, nav.i, loco.stats, perc.stats))
        reg.cerrar()
    return nav


if __name__ == "__main__":
    main()
