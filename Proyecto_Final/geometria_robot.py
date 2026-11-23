"""Geometria del robot para la Raspberry Pi (entrega final).

Fuente: Documentos/Geometria_robot.md. Mantener estos valores iguales a los de
Avance_2_Percepcion/Percepcion_MG996_180/Config.h.

Marco robot (R): origen en el centro del chasis, X adelante, Y izquierda, angulos
antihorarios, mm. El firmware de percepcion entrega puntos en el marco del eje del
servo (P); pasar a R con percepcion_a_robot().
"""
import math

# Eje del servo respecto al centro del chasis.
EJE_SERVO_X_MM = 62.0
EJE_SERVO_Y_MM = 0.0
SERVO_LIMITE_DEG = 60.0          # barrido del TF-Luna: -60..+60

# TF-Luna (gira con el servo)
TF_ORIGEN_CABEZAL_X_MM = 7.0     # [ESTIMADO] lente delante del eje
TF_ALTURA_SOBRE_L4_MM = 30.75

# Sharp GP2Y0A21 (fijos en L4)
SHARP_YAW_DEG = 45.0             # [AJUSTAR] leido en la escala grabada, 0..90 hacia afuera
SHARP_PIVOTE_X_MM = 22.0         # en R; +Y = izquierdo, -Y = derecho
SHARP_PIVOTE_Y_MM = 36.0
SHARP_CARA_DESDE_PIVOTE_MM = 20.0  # [ESTIMADO]
SHARP_ALTURA_SOBRE_L4_MM = 12.05

# Camara (Arducam IMX219 USB) sobre el poste
CAMARA_X_MM = 36.0
CAMARA_Y_MM = 0.0
CAMARA_ALTURA_SOBRE_L4_MM = 80.0
CAMARA_INCLINACION_DEG = None    # [MEDIR] hacia abajo

H_L4_SOBRE_PISO_MM = None        # [MEDIR] cara superior de la placa L4


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


if __name__ == "__main__":
    for lado in ("izq", "der"):
        print(lado, [round(v, 1) for v in sharp_pose(lado)])
    print("TF 0 deg", tf_pose(0))
