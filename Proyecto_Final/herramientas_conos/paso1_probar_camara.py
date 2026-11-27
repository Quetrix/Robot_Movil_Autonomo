"""Paso 1: comprobar que la Arducam se ve y a que velocidad entrega imagen.

    python paso1_probar_camara.py --listar        # que indices tienen camara
    python paso1_probar_camara.py --camara 1      # video en vivo

Teclas en la ventana:  q salir | s guardar foto en fotos/ | f autoenfoque si/no
"""
import argparse
import time
from pathlib import Path

import cv2

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision import abrir_camara, cargar_config, describir_camara


def listar(cfg):
    print("Buscando camaras (cada una se enciende un instante)...")
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)  # calla avisos de indices vacios
    for i in range(5):
        cap = abrir_camara(i, cfg["ancho"], cfg["alto"], cfg["fps"])
        if cap is None:
            continue
        ok, _ = cap.read()
        print("  indice %d: %s" % (i, describir_camara(cap) if ok else "abre pero no entrega imagen"))
        cap.release()
    print("La webcam de la laptop suele ser 0 y la Arducam 1. Desconecta la Arducam y\n"
          "vuelve a listar: el indice que desaparezca es el de la Arducam.")


def main():
    cfg = cargar_config()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listar", action="store_true")
    p.add_argument("--camara", type=int, default=cfg["camara"])
    p.add_argument("--ancho", type=int, default=cfg["ancho"])
    p.add_argument("--alto", type=int, default=cfg["alto"])
    a = p.parse_args()

    if a.listar:
        return listar(cfg)

    cap = abrir_camara(a.camara, a.ancho, a.alto, cfg["fps"], cfg["autofoco"])
    if cap is None:
        raise SystemExit("No hay camara en el indice %d. Prueba con --listar." % a.camara)
    print("Camara %d abierta:" % a.camara, describir_camara(cap))

    autofoco = cfg["autofoco"]
    fps, cuadros, t0 = 0.0, 0, time.time()
    while True:
        ok, cuadro = cap.read()
        if not ok:
            raise SystemExit("La camara dejo de entregar imagen (revisa el cable USB).")
        cuadros += 1
        if time.time() - t0 >= 1:
            fps, cuadros, t0 = cuadros / (time.time() - t0), 0, time.time()
        limpio = cuadro.copy()
        texto = "%dx%d  %.0f fps  autofoco %s" % (cuadro.shape[1], cuadro.shape[0], fps, "SI" if autofoco else "NO")
        cv2.putText(cuadro, texto, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(cuadro, texto, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Paso 1 - camara", cuadro)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            break
        if tecla == ord("s"):
            carpeta = Path(__file__).with_name("fotos")
            carpeta.mkdir(exist_ok=True)
            ruta = carpeta / time.strftime("foto_%Y%m%d_%H%M%S.jpg")
            cv2.imwrite(str(ruta), limpio)
            print("Foto guardada:", ruta)
        if tecla == ord("f"):
            autofoco = not autofoco
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofoco else 0)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
