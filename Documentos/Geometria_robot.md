# Geometría del robot (montaje final, nov 2026)

Fuente única de las posiciones de sensores para el firmware y la Raspberry Pi. Las cifras salen de las placas de acrílico (`MT9005_L4_top` v4.2) y del modelo de Fusion `MT9005_Cabeza_v2_Sensores_Fijos`. Las marcadas **[MEDIR]** o **[ESTIMADO]** se tienen que medir en el robot armado antes de dar por buena la precisión.

Módulo de Python con estos mismos números: [`Proyecto_Final/geometria_robot.py`](../Proyecto_Final/geometria_robot.py). Es la única copia que usa el software: el firmware de percepción final solo entrega el ángulo del cabezal y la distancia, y la Pi hace la geometría.

## Marcos de referencia

- **Marco robot (R):** origen en el centro del chasís (centro de las placas de 120 × 180 mm), X hacia adelante, Y a la izquierda, Z hacia arriba, ángulos antihorarios. Es el mismo del comando `V,vx,vy,wz` del firmware de locomoción final (el de la Tarea 8 usaba vx = derecha, vy = adelante).
- **Marco de percepción (P):** origen en el eje del servo, mismos ejes que R. El firmware de percepción entrega el ángulo del cabezal (antihorario +, 0 = frente) y la distancia del TF-Luna; `falda.py` los pasa a R.
- Paso de P a R: **x_R = x_P + 62 mm**, y_R = y_P. El servo está centrado en Y.
- Conversión desde el DXF de las placas (X a lo ancho, Y a lo largo, frente = +Y): x_R = Y_placa, y_R = −X_placa.

## Sensores

| Sensor | Posición en R (x, y) [mm] | Altura del haz sobre la cara superior de L4 | Dirección | Notas |
|---|---|---|---|---|
| Eje del servo (TF-Luna) | (62, 0) | — | barrido ±60° (límite del firmware) | |
| TF-Luna, lente | eje + 7 mm en la dirección del cabezal | 30.75 mm | ángulo del cabezal | offset de 7 mm **[ESTIMADO]**, calibrar |
| Sharp izquierdo, pivote | (22, +36) | 12.05 mm | +yaw, hacia afuera | yaw de 0 a 90°, leído en la escala grabada; por defecto 45° **[AJUSTAR]** |
| Sharp derecho, pivote | (22, −36) | 12.05 mm | −yaw, hacia afuera | igual que el izquierdo |
| Cara de cada Sharp | pivote + 20 mm · (cos yaw, ±sin yaw) | 12.05 mm | | 20 mm **[ESTIMADO]**: medir del pivote a la ventana |
| Cámara (lente) | (36, 0) | ~80 mm | al frente, **15° hacia abajo** (recomendado), medir la real | Arducam IMX219 USB sobre el poste, 640 × 480 (4:3) |
| IR discretos KY-032 | uno al frente y otro atrás, en la base L1 | — | | posición sin definir |

Con el yaw por defecto de 45°, la cara del Sharp izquierdo queda en R ≈ (36.1, 50.1) y la del derecho en (36.1, −50.1). En el marco P son (−25.9, ±50.1).

El TF-Luna nunca ve el robot: no hay nada sobre L4 dentro de su barrido de ±60°. En la cámara, el borde superior delantero del cabezal queda 51° por debajo de la lente, así que no aparece en la imagen mientras la inclinación más la mitad del FOV vertical sea menor que ~48°.

## Alturas sobre el piso

La altura de L4 sobre el piso depende de los separadores y todavía no está medida **[MEDIR]**. Cuando se mida H_L4 (cara superior de L4), las alturas sobre el piso son: TF-Luna H_L4 + 30.75 mm, Sharp H_L4 + 12.05 mm y cámara H_L4 + ~80 mm. Los obstáculos de la prueba se ajustan a la altura de cada equipo, así que ese es el número que hay que reportar.

## Pendientes de medición

1. Yaw real de cada Sharp en la escala → `SHARP_YAW_DEG` (geometria_robot.py).
2. Distancia del pivote a la cara del Sharp → `SHARP_CARA_DESDE_PIVOTE_MM`.
3. Offset de rango del TF-Luna (lente delante del eje) → `TF_ORIGEN_CABEZAL_X_MM`.
4. H_L4 sobre el piso (`H_L4_SOBRE_PISO_MM`; con ella `python3 geometria_robot.py` da la inclinación óptima), e inclinación y FOV reales de la cámara (`calibrar.py camara`).
5. Anclas del servo (−60/0/+60) y *lag* por sentido en `percepcion.ino` (`calibrar.py lag`).
