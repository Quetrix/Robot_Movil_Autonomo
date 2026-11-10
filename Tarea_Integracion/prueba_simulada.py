#!/usr/bin/env python3
"""Pruebas de lazo cerrado: braitenberg.py contra simulador_sin_robot.py (Linux, ~40 s).

Uso: python3 prueba_simulada.py
Corre en paralelo varios escenarios y revisa el sentido de giro de cada vehiculo, el bloqueo
frontal, el modo serial del cabezal, el paso a PARO y el frenado por timeout de H.
"""
import csv
import os
import signal
import subprocess
import sys
import threading
import time

D = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable


def correr(nombre, escenario, teclas, dur, puerto, modo_perc="http", extra_sim=(), extra_bt=(), matar_bt_en=None):
    traza = f"/tmp/traza_{nombre}.csv"
    sim = subprocess.Popen([PY, "-u", f"{D}/simulador_sin_robot.py", "--escenario", escenario,
                            "--puerto-http", str(puerto), "--traza", traza, *extra_sim],
                           stdout=subprocess.PIPE, text=True)
    rutas = {}
    for _ in range(3):
        k, v = sim.stdout.readline().split()
        rutas[k] = v
    perc = rutas["PERC_HTTP"] if modo_perc == "http" else (rutas["PERC_SERIAL"] if modo_perc == "serial" else modo_perc)
    log = f"/tmp/log_{nombre}.csv"
    bt = subprocess.Popen([PY, "-u", f"{D}/braitenberg.py", "--loco", rutas["LOCO"], "--perc", perc,
                           "--sin-pantalla", "--log", log, *extra_bt],
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    salida = []
    threading.Thread(target=lambda: salida.extend(bt.stdout), daemon=True).start()
    t0 = time.monotonic()
    for t, k in teclas:
        time.sleep(max(0, t - (time.monotonic() - t0)))
        if k == "KILL":
            bt.send_signal(signal.SIGKILL)
            continue
        try:
            bt.stdin.write(k)
            bt.stdin.flush()
        except BrokenPipeError:
            pass
    time.sleep(max(0, dur - (time.monotonic() - t0)))
    if bt.poll() is None:
        try:
            bt.stdin.write("q"); bt.stdin.flush()
        except BrokenPipeError:
            pass
        try:
            bt.wait(3)
        except subprocess.TimeoutExpired:
            bt.kill()
    time.sleep(0.5)
    sim.send_signal(signal.SIGINT)
    fin = sim.stdout.read()
    sim.wait(3)
    filas = list(csv.DictReader(open(traza)))
    return filas, "".join(salida), fin.strip(), log


fallas = 0


def check(ok, msg):
    global fallas
    print(("PASS " if ok else "FAIL ") + msg, flush=True)
    fallas += (not ok)


def theta_en(filas, t):
    for f in filas:
        if float(f["t"]) >= t:
            return float(f["theta_deg"])
    return float(filas[-1]["theta_deg"])


casos = []
def caso(*a, **k):
    casos.append((a, k))

# (nombre, escenario, teclas, duracion, puerto)
caso("2a_izq", "izq", [(4.0, "1"), (9.0, "0")], 10, 8101)
caso("2a_der", "der", [(4.0, "1"), (9.0, "0")], 10, 8102)
caso("2b_izq", "izq", [(4.0, "2"), (11.0, "0")], 12, 8103)
caso("3a_izq", "izq", [(4.0, "3"), (12.0, "0")], 13, 8104)
caso("3a_caja", "caja_izq", [(4.0, "3"), (12.0, "0")], 13, 8112)
caso("2b_caja", "caja_izq", [(4.0, "2"), (11.0, "0")], 12, 8113)
caso("3b_izq", "izq", [(4.0, "4"), (9.0, "0")], 10, 8105)
caso("3b_frente", "frente", [(4.0, "4"), (12.0, "0")], 13, 8106)
caso("3b_arena", "arena", [(4.0, "4"), (34.0, "0")], 35, 8107)
caso("3b_izq_serial", "izq", [(5.5, "4"), (11.0, "0")], 12, 8108, modo_perc="serial")
caso("sin_perc", "izq", [(4.0, "4")], 7, 8109, modo_perc="http://127.0.0.1:1")
caso("kill", "arena", [(4.0, "4"), (6.0, "KILL")], 8, 8110)
caso("2a_izq_inv", "izq", [(4.0, "1"), (9.0, "0")], 10, 8114, extra_bt=("--invertir-sharp",))
caso("fw_sin_ir", "izq", [], 5, 8111, extra_sim=("--sin-ir",))

res = {}
hilos = []
for a, k in casos:
    def f(a=a, k=k):
        res[a[0]] = correr(*a, **k)
    h = threading.Thread(target=f)
    h.start()
    hilos.append(h)
    time.sleep(0.3)
for h in hilos:
    h.join()

for nombre in res:
    filas, salida, fin, log = res[nombre]
    print(f"--- {nombre}: {fin}")

f, s, fin, _ = res["2a_izq"]
check(theta_en(f, 9) < 80, f"2a obstaculo izq: gira a la derecha (theta {theta_en(f, 9):.0f} < 80)")
f, s, fin, _ = res["2a_der"]
check(theta_en(f, 9) > 100, f"2a obstaculo der: gira a la izquierda (theta {theta_en(f, 9):.0f} > 100)")
f, s, fin, _ = res["2b_izq"]
check(theta_en(f, 7) > 95, f"2b obstaculo izq: gira hacia el obstaculo (theta {theta_en(f, 7):.0f} > 95)")
print("   2b izq (embiste, se permite contacto):", fin.split("holgura_min=")[1])
f, s, fin, _ = res["3a_izq"]
check(theta_en(f, 9) > 95, f"3a obstaculo izq: gira hacia el (theta {theta_en(f, 9):.0f} > 95)")
print("   3a obstaculo chico:", fin.split("holgura_min=")[1])
for n in ("3a_caja", "2b_caja"):
    f, s, fin, _ = res[n]
    check(theta_en(f, 10) > 95, f"{n}: gira hacia la caja (theta {theta_en(f, 10):.0f}); {fin.split('holgura_min=')[1]}")
f, s, fin, _ = res["3b_izq"]
check(theta_en(f, 9) < 80, f"3b obstaculo izq: se aparta a la derecha (theta {theta_en(f, 9):.0f} < 80)")
check("choques=0" in fin, "3b izq sin choque")
f, s, fin, _ = res["3b_frente"]
check("choques=0" in fin, "3b obstaculo al frente: sin choque")
check(abs(theta_en(f, 12) - 90) > 20, f"3b frente: el reflejo lo hace girar (theta {theta_en(f, 12):.0f})")
f, s, fin, _ = res["3b_arena"]
print("   3b arena (zonas ciegas entre 0 y +-60 grados):", fin.split("holgura_min=")[1])
dist = sum(((float(b["x"]) - float(a["x"])) ** 2 + (float(b["y"]) - float(a["y"])) ** 2) ** 0.5 for a, b in zip(f, f[1:]))
check(dist > 2.0, f"3b arena: explora (recorrido {dist:.1f} m)")
f, s, fin, _ = res["3b_izq_serial"]
check(theta_en(f, 10) < 80, f"cabezal por serial (DETAIL): 3b se aparta (theta {theta_en(f, 10):.0f} < 80)")
f, s, fin, _ = res["sin_perc"]
check("PARO" in s, "sin percepcion: pasa a PARO")
check(max(abs(float(r["vy_cmd"])) for r in f) < 1e-3, "sin percepcion: el robot nunca se mueve")
f, s, fin, _ = res["kill"]
mov = [r for r in f if 5.0 < float(r["t"]) < 6.0 and abs(float(r["vy_cmd"])) + abs(float(r["w_cmd"])) > 1e-3]
quieto = [r for r in f if float(r["t"]) > 6.5]
check(len(mov) > 0 and all(abs(float(r["vy_cmd"])) + abs(float(r["w_cmd"])) < 1e-3 for r in quieto),
      "script muerto con SIGKILL: el robot frena solo por el timeout de H")
f, s, fin, _ = res["2a_izq_inv"]
check(theta_en(f, 9) > 100, f"--invertir-sharp intercambia los lados (2a gira al reves: theta {theta_en(f, 9):.0f})")
f, s, fin, _ = res["fw_sin_ir"]
check("no tiene el firmware ir_frontal" in s, "detecta firmware sin IR frontal")

print("\nTODO OK" if not fallas else f"\n{fallas} FALLAS")
sys.exit(1 if fallas else 0)
