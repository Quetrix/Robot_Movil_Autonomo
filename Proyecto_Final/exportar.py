#!/usr/bin/env python3
"""Convierte un registro en CSV por tipo de trama y revisa si se perdieron datos.

    python3 exportar.py                       # el registro mas reciente
    python3 exportar.py registros/20261124_101500
    python3 exportar.py --zip                 # ademas arma <carpeta>.zip para compartir

Genera en la misma carpeta:
  loco_L.csv   t_pi, seq, ms, conteos, ref RPM, PWM, flags y RPM medidas
  loco_D.csv / perc_D.csv   diagnostico de cada ESP32 (1 Hz)
  perc_T.csv   TF-Luna: angulo, distancia, amplitud, flags
  perc_S.csv   Sharp (mm y mV) e IR trasero
  resumen.txt  tramas por tipo, frecuencia real, perdidas por secuencia, lineas malas,
               mayor hueco entre tramas, y los contadores de los propios ESP32
"""
import argparse
import csv
import glob
import os
import shutil
import sys

import config
import protocolo as P

COLUMNAS = {
    ("loco", "L"): ["seq", "ms", "c_FL", "c_FR", "c_RL", "c_RR", "ref_FL", "ref_FR", "ref_RL", "ref_RR",
                    "pwm_FL", "pwm_FR", "pwm_RL", "pwm_RR", "flags"],
    ("loco", "D"): ["seq", "ms", "lazo_max_us", "rx_malas", "tx_perdidas", "watchdogs", "stalls"],
    ("perc", "T"): ["seq", "ms", "ang_cdeg", "dist_mm", "amp", "flags"],
    ("perc", "S"): ["seq", "ms", "izq_mm", "der_mm", "izq_mv", "der_mv", "ir_tras"],
    ("perc", "D"): ["seq", "ms", "lazo_max_us", "rx_malas", "tx_perdidas", "i2c_err", "tf_validas_s", "modo"],
}
ESPERADO_HZ = {("loco", "L"): 50, ("perc", "T"): 100, ("perc", "S"): 25, ("loco", "D"): 1, ("perc", "D"): 1}


def exportar(carpeta):
    resumen = ["Registro: " + carpeta, ""]
    for enlace in ("loco", "perc"):
        ruta = os.path.join(carpeta, "crudo_%s.txt" % enlace)
        if not os.path.exists(ruta):
            continue
        datos = {}           # tipo -> filas
        malas, texto, tx = 0, [], 0
        for linea in open(ruta, encoding="utf-8", errors="replace"):
            partes = linea.rstrip("\n").split("\t", 2)
            if len(partes) != 3:
                continue
            t, sentido, contenido = partes
            if sentido == "TX":
                tx += 1
                continue
            r = P.separar(contenido.encode())
            if r is None:
                malas += 1
                continue
            tipo, campos = r
            if tipo == "#":
                texto.append("%s %s" % (t, campos))
            elif (enlace, tipo) in COLUMNAS:
                datos.setdefault(tipo, []).append([float(t)] + [int(c) for c in campos])
        resumen.append("== %s: %d lineas malas, %d comandos enviados" % (enlace.upper(), malas, tx))
        for tipo, filas in sorted(datos.items()):
            cols = COLUMNAS[(enlace, tipo)]
            if tipo == "L":                      # RPM medidas entre tramas consecutivas
                cols = cols + ["rpm_FL", "rpm_FR", "rpm_RL", "rpm_RR"]
                for i, f in enumerate(filas):
                    if i == 0 or f[2] <= filas[i - 1][2]:
                        f += [0.0] * 4
                    else:
                        dt = (f[2] - filas[i - 1][2]) / 1000
                        f += [round((f[3 + k] - filas[i - 1][3 + k]) / config.CPR[k] * 60 / dt, 1) for k in range(4)]
            with open(os.path.join(carpeta, "%s_%s.csv" % (enlace, tipo)), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_pi"] + cols)
                w.writerows(filas)
            seqs = [f[1] for f in filas]
            perdidas = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]) if b > a + 1)
            reinicios = sum(1 for a, b in zip(seqs, seqs[1:]) if b < a)
            dur = filas[-1][0] - filas[0][0] if len(filas) > 1 else 0
            huecos = max((b[0] - a[0] for a, b in zip(filas, filas[1:])), default=0)
            hz = (len(filas) - 1) / dur if dur else 0
            esperado = ESPERADO_HZ.get((enlace, tipo), 0)
            resumen.append("  %s: %d tramas en %.1f s = %.1f Hz (esperado %d)  perdidas por secuencia: %d  "
                           "reinicios del ESP32: %d  mayor hueco: %.0f ms" % (
                               tipo, len(filas), dur, hz, esperado, perdidas, reinicios, huecos * 1000))
            if tipo == "D" and filas:
                u = dict(zip(COLUMNAS[(enlace, "D")], filas[-1][1:]))
                resumen.append("  contadores del ESP32 al final: " + ", ".join(
                    "%s=%s" % (k, v) for k, v in u.items() if k not in ("seq", "ms")))
        if texto:
            resumen.append("  mensajes: " + " | ".join(texto[-8:]))
        resumen.append("")
    resumen.append("Todo bien si: perdidas = 0, lineas malas = 0 (o solo al arrancar), reinicios = 0, "
                   "tx_perdidas = 0, frecuencias cerca de lo esperado y mayor hueco < 100 ms en L/T.")
    texto = "\n".join(resumen)
    open(os.path.join(carpeta, "resumen.txt"), "w", encoding="utf-8").write(texto)
    print(texto)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("carpeta", nargs="?")
    p.add_argument("--zip", action="store_true")
    a = p.parse_args()
    carpeta = a.carpeta or max(glob.glob(os.path.join(config.CARPETA_REGISTROS, "*")), default=None)
    if not carpeta or not os.path.isdir(carpeta):
        sys.exit("No hay registros en %s/" % config.CARPETA_REGISTROS)
    exportar(carpeta.rstrip("/\\"))
    if a.zip:
        print("Comprimido:", shutil.make_archive(carpeta.rstrip("/\\"), "zip", carpeta))


if __name__ == "__main__":
    main()
