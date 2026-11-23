#!/usr/bin/env python3
"""Teleoperacion del ESP32 de locomocion (firmware locomocion_esp32_sin_IR).

Dos modos:
  teclas   Mantener teclas para mover. Envia "H vx vy omega" mientras haya
           teclas presionadas; al soltar todo envia "V 0 0 0".
  consola  Escribir comandos tal cual (W, V 0 0.1 0 2000, X, ...).

Uso:
  pip install pyserial pynput
  python teleop_locomocion.py --port COM3
  python teleop_locomocion.py --port COM3 --modo consola

Teclas (modo teclas):
  W/S  adelante / atras        A/D  izquierda / derecha (lateral)
  Q/E  giro antihorario / horario
  +/-  velocidad lineal        ]/[  velocidad angular
  Espacio  parada inmediata (X)    Esc  salir

pynput lee el teclado de todo el sistema: las teclas actuan aunque la
ventana de la terminal no tenga el foco. En Linux necesita sesion grafica
(no funciona por SSH); ahi usar el modo consola.
"""

import argparse
import sys
import threading
import time

import serial

V_MAX = 0.30
W_MAX = 3.0


def lector(ser, mostrar_csv, activo):
    while activo.is_set():
        try:
            linea = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            break
        if not linea:
            continue
        if not mostrar_csv and linea[0].isdigit():
            continue
        print(f"\r[esp] {linea}")


def enviar(ser, texto):
    ser.write((texto + "\n").encode())


def modo_consola(ser):
    print("Modo consola. Escribe comandos; 'salir' para terminar.")
    print("Ejemplos: W | W 2000 | V 0 0.1 0 | V 0.1 0 0 1500 | R | X")
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd.lower() in ("salir", "exit", "q"):
            break
        if cmd:
            enviar(ser, cmd)


def modo_teclas(ser, v, w, hz):
    from pynput import keyboard

    presionadas = set()
    estado = {"v": v, "w": w, "salir": False, "moviendo": False}
    candado = threading.Lock()

    def nombre(tecla):
        if tecla == keyboard.Key.space:
            return "espacio"
        if tecla == keyboard.Key.esc:
            return "esc"
        try:
            return tecla.char.lower() if tecla.char else None
        except AttributeError:
            return None

    def al_presionar(tecla):
        k = nombre(tecla)
        if k is None:
            return
        with candado:
            if k == "esc":
                estado["salir"] = True
                return False
            if k == "espacio":
                presionadas.clear()
                enviar(ser, "X")
                estado["moviendo"] = False
            elif k in "+=":
                estado["v"] = min(V_MAX, round(estado["v"] + 0.02, 2))
                print(f"\rv = {estado['v']:.2f} m/s")
            elif k in "-_":
                estado["v"] = max(0.02, round(estado["v"] - 0.02, 2))
                print(f"\rv = {estado['v']:.2f} m/s")
            elif k == "]":
                estado["w"] = min(W_MAX, round(estado["w"] + 0.1, 2))
                print(f"\romega = {estado['w']:.2f} rad/s")
            elif k == "[":
                estado["w"] = max(0.1, round(estado["w"] - 0.1, 2))
                print(f"\romega = {estado['w']:.2f} rad/s")
            elif k in "wasdqe":
                presionadas.add(k)

    def al_soltar(tecla):
        k = nombre(tecla)
        if k is None:
            return
        with candado:
            presionadas.discard(k)

    print(__doc__.split("Teclas (modo teclas):")[1].split("pynput")[0])
    print(f"v = {v:.2f} m/s, omega = {w:.2f} rad/s")

    escucha = keyboard.Listener(on_press=al_presionar, on_release=al_soltar)
    escucha.start()
    periodo = 1.0 / hz
    try:
        while not estado["salir"]:
            with candado:
                p = set(presionadas)
                vv, ww = estado["v"], estado["w"]
            vy = vv * (("w" in p) - ("s" in p))
            vx = vv * (("d" in p) - ("a" in p))
            om = ww * (("q" in p) - ("e" in p))
            if vx or vy or om:
                enviar(ser, f"H {vx:.3f} {vy:.3f} {om:.3f}")
                estado["moviendo"] = True
            elif estado["moviendo"]:
                enviar(ser, "V 0 0 0")
                estado["moviendo"] = False
            time.sleep(periodo)
    except KeyboardInterrupt:
        pass
    finally:
        escucha.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="COM3, /dev/ttyUSB0, ...")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--modo", choices=["teclas", "consola"], default="teclas")
    ap.add_argument("--v", type=float, default=0.20, help="velocidad lineal [m/s]")
    ap.add_argument("--w", type=float, default=1.0, help="velocidad angular [rad/s]")
    ap.add_argument("--hz", type=float, default=10.0, help="frecuencia de envio de H")
    ap.add_argument("--csv", action="store_true", help="mostrar telemetria CSV")
    a = ap.parse_args()

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(1.5)
    activo = threading.Event()
    activo.set()
    threading.Thread(target=lector, args=(ser, a.csv, activo), daemon=True).start()

    try:
        if a.modo == "consola":
            modo_consola(ser)
        else:
            if a.hz < 4:
                sys.exit("--hz debe ser >= 4 (el firmware frena si no recibe H en 300 ms)")
            modo_teclas(ser, min(a.v, V_MAX), min(a.w, W_MAX), a.hz)
    finally:
        enviar(ser, "X")
        time.sleep(0.2)
        activo.clear()
        ser.close()
        print("\nPuerto cerrado.")


if __name__ == "__main__":
    main()
