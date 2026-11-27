"""Paso 3: deteccion de conos en vivo (o sobre una foto).

    python paso3_detectar_conos.py --camara 1
    python paso3_detectar_conos.py --imagen fotos/foto.jpg
    python paso3_detectar_conos.py --camara 0 --sin-ventana     # Raspberry Pi sin monitor

En consola imprime distancia y rumbo (positivo = izquierda, marco robot) de cada cono.
Teclas:  q salir | m ver/ocultar la mascara de color
"""
import argparse
import time
from pathlib import Path

import cv2

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision import abrir_camara, cargar_config, describir_camara, detectar_y_ubicar, dibujar, texto_conos


def main():
    cfg = cargar_config()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camara", type=int, default=cfg["camara"])
    p.add_argument("--imagen", help="procesar una foto en vez de la camara")
    p.add_argument("--sin-ventana", action="store_true", help="solo imprimir, sin abrir ventanas")
    a = p.parse_args()

    if a.imagen:
        img = cv2.imread(a.imagen)
        if img is None:
            raise SystemExit("No pude leer la imagen " + a.imagen)
        conos, _ = detectar_y_ubicar(img, cfg)
        print(texto_conos(conos))
        salida = Path(a.imagen).with_name(Path(a.imagen).stem + "_detectado.jpg")
        cv2.imwrite(str(salida), dibujar(img, conos))
        print("Resultado guardado en", salida)
        return

    cap = abrir_camara(a.camara, cfg["ancho"], cfg["alto"], cfg["fps"], cfg["autofoco"])
    if cap is None:
        raise SystemExit("No hay camara en el indice %d. Usa paso1_probar_camara.py --listar." % a.camara)
    print("Camara %d abierta:" % a.camara, describir_camara(cap))

    ver_mascara = False
    ultimo_texto, ultimo_print = "", 0.0
    fps, cuadros, t0 = 0.0, 0, time.time()
    while True:
        ok, cuadro = cap.read()
        if not ok:
            raise SystemExit("La camara dejo de entregar imagen (revisa el cable USB).")
        conos, mascara = detectar_y_ubicar(cuadro, cfg)

        cuadros += 1
        if time.time() - t0 >= 1:
            fps, cuadros, t0 = cuadros / (time.time() - t0), 0, time.time()

        # Imprime cuando cambia el resultado, y como maximo 5 veces por segundo.
        texto = texto_conos(conos)
        if time.time() - ultimo_print >= 0.2 and (conos or texto != ultimo_texto):
            print("%4.0f fps | %s" % (fps, texto))
            ultimo_texto, ultimo_print = texto, time.time()

        if a.sin_ventana:
            continue
        dibujar(cuadro, conos)
        cv2.putText(cuadro, "%.0f fps" % fps, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Paso 3 - conos", cuadro)
        if ver_mascara:
            cv2.imshow("mascara", mascara)
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            break
        if tecla == ord("m"):
            ver_mascara = not ver_mascara
            if not ver_mascara:
                cv2.destroyWindow("mascara")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
