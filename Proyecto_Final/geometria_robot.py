"""Geometria del robot para la Raspberry Pi (entrega final).

Fuente: Documentos/Geometria_robot.md. Es la unica copia que usa el software: el firmware
de percepcion final entrega angulo del cabezal y distancia, y la Pi hace la geometria.

Marco robot (R): origen en el centro del chasis, X adelante, Y izquierda, angulos
antihorarios, mm. Marco P: origen en el eje del servo, mismos ejes (x_R = x_P + 62).
"""
import math

# Eje del servo respecto al centro del chasis.
EJE_SERVO_X_MM = 62.0
EJE_SERVO_Y_MM = 0.0
SERVO_LIMITE_DEG = 60.0          # barrido del TF-Luna: -60..+60

# TF-Luna (gira con el servo)
TF_ORIGEN_CABEZAL_X_MM = 7.0     # [ESTIMADO] lente delante del eje
TF_ALTURA_SOBRE_L4_MM = 28.0      # medido: lente a 156 mm del piso (25 nov)

# Sharp GP2Y0A21 (fijos en L4)
SHARP_YAW_DEG = 45.0             # [AJUSTAR] leido en la escala grabada, 0..90 hacia afuera
SHARP_PIVOTE_X_MM = 22.0         # en R; +Y = izquierdo, -Y = derecho
SHARP_PIVOTE_Y_MM = 36.0
SHARP_CARA_DESDE_PIVOTE_MM = 20.0  # [ESTIMADO]
SHARP_ALTURA_SOBRE_L4_MM = 12.05

# Camara (Arducam IMX219 USB) sobre el poste
CAMARA_X_MM = 36.0
CAMARA_Y_MM = 0.0
CAMARA_ALTURA_SOBRE_L4_MM = 94.0  # medido: lente a 222 mm del piso con la camara recta (25 nov)
CAMARA_INCLINACION_DEG = 12.0    # hacia abajo; optimo para lente a 222 mm (inclinacion_optima)

H_L4_SOBRE_PISO_MM = 128.0       # medido 25 nov: cara superior de L4
H_L4_ESTIMADA_MM = 180.0         # solo para calcular mientras no se mida


def percepcion_a_robot(x_p, y_p):
    """Punto del marco del eje del servo (salida del ESP32) al marco robot."""
    return x_p + EJE_SERVO_X_MM, y_p + EJE_SERVO_Y_MM


def sharp_pose(lado):
    """(x, y, rumbo_deg) de la cara del Sharp en R. lado: 'izq' o 'der'."""
    s = 1.0 if lado == "izq" else -1.0
    rumbo = s * SHARP_YAW_DEG
    a = math.radians(rumbo)
    x = SHARP_PIVOTE_X_MM + SHARP_CARA_DESDE_PIVOTE_MM * math.cos(a)
    y = s * SHARP_PIVOTE_Y_MM + SHARP_CARA_DESDE_PIVOTE_MM * math.sin(a)
    return x, y, rumbo


def tf_pose(angulo_cabezal_deg):
    """(x, y, rumbo_deg) de la lente del TF-Luna en R para un angulo del servo."""
    a = math.radians(angulo_cabezal_deg)
    return (EJE_SERVO_X_MM + TF_ORIGEN_CABEZAL_X_MM * math.cos(a),
            EJE_SERVO_Y_MM + TF_ORIGEN_CABEZAL_X_MM * math.sin(a),
            angulo_cabezal_deg)


def punto_detectado(pose, distancia_mm):
    """Punto del obstaculo en R a partir de la pose del sensor y su lectura."""
    x, y, rumbo = pose
    a = math.radians(rumbo)
    return x + distancia_mm * math.cos(a), y + distancia_mm * math.sin(a)


def altura_camara_m():
    """Altura de la lente sobre el piso (con la estimacion si H_L4 no se ha medido)."""
    h = H_L4_SOBRE_PISO_MM if H_L4_SOBRE_PISO_MM is not None else H_L4_ESTIMADA_MM
    return (h + CAMARA_ALTURA_SOBRE_L4_MM) / 1000.0


def inclinacion_optima(h_cam_m, d_min_m=0.5, d_max_m=3.0, h_cono_m=0.18):
    """Inclinacion [grados hacia abajo] que centra en la imagen la franja util: desde la base de
    un cono a d_min hasta la punta de un cono a d_max. Requiere FOV vertical mayor que la franja."""
    abajo = math.degrees(math.atan2(h_cam_m, d_min_m))
    arriba = math.degrees(math.atan2(h_cam_m - h_cono_m, d_max_m))
    return (abajo + arriba) / 2, abajo - arriba


def tf_punto(angulo_cabezal_deg, distancia_mm):
    """Punto detectado por el TF-Luna en R [mm]."""
    return punto_detectado(tf_pose(angulo_cabezal_deg), distancia_mm)


if __name__ == "__main__":
    for lado in ("izq", "der"):
        print(lado, [round(v, 1) for v in sharp_pose(lado)])
    print("TF 0 deg", tf_pose(0))
    h = altura_camara_m()
    inc, franja = inclinacion_optima(h)
    print("camara a %.3f m: inclinacion optima %.1f grados (franja de %.1f grados)" % (h, inc, franja))
