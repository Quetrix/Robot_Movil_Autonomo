#!/usr/bin/env python3
"""
Coreografia de respaldo: movimiento PREGRABADO, sin sensores, que reproduce como se moveria cada
vehiculo de Braitenberg ante un obstaculo supuesto. Corre igual cada vez (en lazo abierto; el
deslizamiento de las ruedas Mecanum puede variar un poco la trayectoria en el suelo).

No es el comportamiento autonomo: los estimulos sI(t), sD(t) son un perfil fijo en el tiempo, no
lecturas de los Sharp. El panel lo indica siempre. Si se usa en el video, hay que decirlo.

El perfil de estimulo pasa por las MISMAS funciones de braitenberg.py (miedo_2a, ... ,
diferencial_a_robot, tope V_MAX) y se envia con el mismo comando H a 10 Hz, asi que el
movimiento es el que produciria el vehiculo si el obstaculo apareciera de esa forma.

Teclas: 1 = 2a Miedo  2 = 2b Agresion  3 = 3a Amor  4 = 3b Explorador  (cada una corre una vez)
        0 o espacio = frenar   q = salir (envia X)
Uso:    python3 coreografia.py --loco /dev/serial/by-id/<locomocion> [--espejo]
        --espejo: obstaculo supuesto a la derecha en vez de a la izquierda
Montaje: dejar el obstaculo donde dice el panel para que la toma se vea coherente.
"""
import argparse
import sys
import time

import braitenberg as bt


def rampa(t, t0, t1, alto):
    """0 antes de t0, sube suave hasta 'alto' en t1 y se queda."""
    if t <= t0:
        return 0.0
    if t >= t1:
        return alto
    u = (t - t0) / (t1 - t0)
    return alto * u * u * (3 - 2 * u)


def pulso(t, t0, t1, alto):
    """Sube de t0 a la mitad y baja hasta t1."""
    m = (t0 + t1) / 2
    return rampa(t, t0, m, alto) if t < m else alto - rampa(t, m, t1, alto)


# vehiculo, duracion [s], perfil sI(t), sD(t) con obstaculo a la IZQUIERDA, montaje sugerido
PERFILES = {
    "1": ("MIEDO_2A", 6.0,
          lambda t: (pulso(t, 1.5, 4.0, 0.8), 0.0),
          "caja en diagonal adelante-IZQUIERDA a ~50 cm: se aleja a la derecha acelerando"),
    "2": ("AGRESION_2B", 5.0,
          lambda t: (rampa(t, 1.5, 3.0, 0.7), rampa(t, 3.0, 4.2, 0.7)),
          "caja en diagonal adelante-IZQUIERDA a ~60 cm: gira hacia ella y se detiene antes"),
    "3": ("AMOR_3A", 7.0,
          lambda t: (rampa(t, 1.5, 3.5, 0.6) + rampa(t, 4.0, 6.0, 0.4), rampa(t, 3.5, 6.0, 1.0)),
          "caja ancha en diagonal adelante-IZQUIERDA: gira hacia ella, desacelera y se queda"),
    "4": ("EXPLORADOR_3B", 9.0,
          lambda t: (pulso(t, 1.5, 3.5, 0.8), pulso(t, 5.0, 7.0, 0.8)),
          "caja adelante-IZQUIERDA y despues otra adelante-DERECHA: las esquiva y sigue"),
}


class Coreografia:
    def __init__(self, loco, espejo):
        self.loco, self.espejo = loco, espejo
        self.tecla = None
        self.t0 = 0.0
        self.t = dict(sI=0.0, sD=0.0, vI=0.0, vD=0.0, v=0.0, w=0.0)

    def iniciar(self, k):
        self.tecla, self.t0 = k, time.monotonic()

    def frenar(self):
        self.tecla = None
        self.loco.frenar()
        self.t = dict(sI=0.0, sD=0.0, vI=0.0, vD=0.0, v=0.0, w=0.0)

    def paso(self):
        if self.tecla is None:
            return
        estado, dur, perfil, _ = PERFILES[self.tecla]
        t = time.monotonic() - self.t0
        if t >= dur:
            self.frenar()
            return
        sI, sD = perfil(t)
        if self.espejo:
            sI, sD = sD, sI
        sI, sD = min(1.0, sI), min(1.0, sD)
        vI, vD = bt.COMPORTAMIENTOS[estado](sI, sD)
        v, w = bt.diferencial_a_robot(vI, vD)
        v = min(v, bt.V_MAX)
        if estado == "AGRESION_2B" and t > 4.2:      # el TF lo detendria al llegar
            v = 0.0
        if max(vI, vD) < bt.V_PARO:
            v = w = 0.0
        self.loco.mover(v, w)
        self.t = dict(sI=sI, sD=sD, vI=vI, vD=vD, v=v, w=w)


def panel(c, loco, eventos, t):
    x = c.t
    if c.tecla:
        estado, dur, _, montaje = PERFILES[c.tecla]
        prog = f"{time.monotonic() - c.t0:4.1f} / {dur:.0f} s"
        if c.espejo:
            montaje = montaje.replace("IZQUIERDA", "@").replace("DERECHA", "IZQUIERDA").replace("@", "DERECHA")
    else:
        estado, prog, montaje = "ESPERA", "", "-"
    sep = "-" * 74
    lineas = [
        " COREOGRAFIA PREGRABADA (sin sensores) - respaldo, no es el modo autonomo",
        sep,
        f" VEHICULO: {estado:14s} {prog}",
        f" montaje:  {montaje}",
        sep,
        f"  estimulo supuesto  sI = {x['sI']:.2f}  {bt.barra(x['sI'])}",
        f"                     sD = {x['sD']:.2f}  {bt.barra(x['sD'])}",
        f"  ruedas virtuales   vI = {x['vI']:+.3f}   vD = {x['vD']:+.3f}  m/s",
        f"  robot              v  = {x['v']:+.3f} m/s   w = {x['w']:+.2f} rad/s",
        f"  comando            {loco.ultimo_cmd}",
        f"  IR frente (firmware): {'OBSTACULO' if loco.ir_frente else 'libre'}",
        sep,
        "  1 Miedo 2a | 2 Agresion 2b | 3 Amor 3a | 4 Explorador 3b | 0/espacio Frenar | q Salir",
        sep,
    ] + ["  " + e for e in eventos.lista]
    return "\033[H" + "".join(ln + "\033[K\n" for ln in lineas) + "\033[J"


def main():
    ap = argparse.ArgumentParser(description="Coreografia pregrabada de los vehiculos de Braitenberg")
    ap.add_argument("--loco", required=True)
    ap.add_argument("--espejo", action="store_true", help="obstaculo supuesto a la derecha")
    a = ap.parse_args()

    loco = bt.Locomocion(a.loco)
    eventos = bt.Eventos(time.monotonic())
    print("Abriendo puerto (2.5 s)...")
    loco.iniciar()
    c = Coreografia(loco, a.espejo)
    teclado = bt.Teclado()
    sys.stdout.write("\033[2J")
    t_cmd = t_pant = 0.0
    try:
        while True:
            ahora = time.monotonic()
            k = teclado.leer()
            if k == "q":
                break
            if k in PERFILES:
                c.iniciar(k)
                eventos.agregar(f"inicia {PERFILES[k][0]}")
            elif k in ("0", " "):
                c.frenar()
                eventos.agregar("frenar")
            for msg in loco.leer():
                eventos.agregar(f"[loco] {msg}")
            if ahora - t_cmd >= bt.T_CMD_MS / 1000:
                c.paso()
                t_cmd = ahora
            if ahora - t_pant >= bt.T_PANTALLA_MS / 1000:
                sys.stdout.write(panel(c, loco, eventos, ahora))
                sys.stdout.flush()
                t_pant = ahora
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        loco.parar()
        time.sleep(0.1)
        teclado.restaurar()
        print("\nDetenido (se envio X).")


if __name__ == "__main__":
    main()
