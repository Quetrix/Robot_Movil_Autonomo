"""Parametros de la Raspberry Pi para la entrega final.

Unidades SI (m, s, rad) salvo que el nombre diga otra cosa. Marco robot: X adelante,
Y izquierda, giro antihorario positivo. Lo marcado [CALIBRAR] / [MEDIR] sale de una
prueba en el robot; ver README.md, seccion "Plan de pruebas".
La geometria de sensores esta en geometria_robot.py.
"""
import math

# ---------------------------------------------------------------- enlaces USB
BAUDIOS = 115200
# Puertos fijos (opcional). Si quedan en None se identifican solos con "?" -> I,LOCO / I,PERC.
PUERTO_LOCO = None
PUERTO_PERC = None
PERIODO_CMD_S = 0.05          # V a 20 Hz; el ESP32 frena solo si pasan 0.5 s sin V
PERIODO_KEEPALIVE_PERC_S = 1.0
TIMEOUT_DATOS_S = 0.5         # sin tramas L/T en este tiempo -> enlace caido

# ---------------------------------------------------------------- locomocion
# Valores del firmware; la odometria usa los calibrados de abajo.
CPR = (903.50, 939.93, 903.21, 905.21)
RADIO_RUEDA = 0.029           # rueda de 58 mm medida; [CALIBRAR] calibrar.py odometria (recta de 2 m)
L_GEOM = 0.15                 # a + b [m] [CALIBRAR] calibrar.py odometria (giro)
RPM_MAX = 120.0
IR_FRENTE_HABILITADO = True   # False si el KY-032 frontal no esta conectado (GPIO34 flotante)

# ---------------------------------------------------------------- percepcion
BARRIDO_AMPLITUD_DEG = 60
BARRIDO_VEL_DPS = 150         # ~1 s por pasada a 100 Hz -> 1.5 grados entre muestras
FALDA_VIGENCIA_S = 1.2        # un poco mas que una pasada del barrido
FALDA_ALCANCE_TF_M = 2.5      # mas lejos no interesa para esquivar (y evita ver el piso)
FALDA_SECTOR_DEG = 5          # resolucion de la matriz [angulo, distancia]
SHARP_MEDIANA = 3             # muestras para la mediana de cada Sharp

# ---------------------------------------------------------------- camara
CAMARA_INDICE = 0             # /dev/video0 en la Pi; se puede pasar una ruta
CAMARA_ANCHO, CAMARA_ALTO = 640, 480   # 4:3 = campo vertical completo del IMX219
CAMARA_FPS = 30
CAMARA_PROCESAR_HZ = 15       # cuadros analizados por segundo (ahorra CPU)
CAMARA_FOV_H_DEG = 60.0       # [MEDIR] calibrar.py camara (objeto de ancho conocido)
CONO_ALTO_M = 0.18            # enunciado: ~18 cm [MEDIR] el cono real
CONO_SEPARACION_M = 0.50      # puertas de 50 cm
CONO_SEPARACION_TOL_M = 0.22
ARUCO_LADO_M = 0.10           # [MEDIR] no se conoce el marcador
ARUCO_DICCIONARIOS = ("DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250", "DICT_ARUCO_ORIGINAL")
ARUCO_CADA_N_CUADROS = 3      # fuera de la fase final se busca ArUco solo 1 de cada N

# ---------------------------------------------------------------- navegacion
ROBOT_ANCHO_M = 0.20          # con ruedas [MEDIR]
ROBOT_LARGO_M = 0.20
V_CRUCERO = 0.20              # ~80 RPM de rueda
V_CRUCE = 0.15
V_APROX = 0.08
W_MAX = math.radians(60)
K_POS = 1.2                   # [1/s] ganancia de posicion
K_RUMBO = 2.0                 # [1/s] ganancia de rumbo
D_APROX_PUERTA = 0.60         # punto de alineacion antes del centro de la puerta
D_SALIDA_PUERTA = 0.30        # se recorre despues del centro antes de dar la puerta por cruzada
TOL_POS = 0.05
TOL_RUMBO = math.radians(6)
D_FRENO_OBST = 0.40           # obstaculo en el pasillo de avance mas cerca que esto -> esquivar
D_LIBRE_OBST = 0.60           # se retoma la ruta cuando el pasillo de avance esta libre hasta aqui
V_LATERAL_EVITAR = 0.15
RADIO_IGNORAR_CONO = 0.15     # puntos de la falda cerca de un cono de la puerta actual
BUSCAR_GIRO_DEG = 50          # barrido de busqueda alrededor del rumbo esperado
D_ESTACIONAR = 0.25           # [AJUSTAR] distancia final al marcador
TOL_FINAL_POS = 0.03
TOL_FINAL_RUMBO = math.radians(3)
T_SIN_AVANCE_S = 30           # el enunciado penaliza > 60 s detenido

# ---------------------------------------------------------------- registro
CARPETA_REGISTROS = "registros"
FOTO_CADA_S = 2.0             # foto de diagnostico (0 = nunca); tambien al ver cono/ArUco nuevo
