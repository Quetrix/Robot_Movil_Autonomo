# Geometría del robot (montaje final, nov 2026)

Fuente única de las posiciones de sensores para el firmware y la Raspberry Pi. Las cifras salen de las placas de acrílico (`MT9005_L4_top` v4.2) y del modelo de Fusion `MT9005_Cabeza_v2_Sensores_Fijos`. Las marcadas **[MEDIR]** o **[ESTIMADO]** se tienen que medir en el robot armado antes de dar por buena la precisión.

Módulo de Python con estos mismos números: [`Proyecto_Final/geometria_robot.py`](../Proyecto_Final/geometria_robot.py). En el firmware de percepción están en `Avance_2_Percepcion/Percepcion_MG996_180/Config.h`.

## Marcos de referencia

- **Marco robot (R):** origen en el centro del chasís (centro de las placas de 120 × 180 mm), X hacia adelante, Y a la izquierda, Z hacia arriba, ángulos antihorarios. Es el mismo que usa locomoción en las tramas `X,Y,T`.
- **Marco de percepción (P):** origen en el eje del servo, mismos ejes que R. El firmware de percepción entrega `x, y` y la matriz `[ángulo; distancia]` en este marco.
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
| Cámara (lente) | (36, 0) | ~80 mm | al frente, inclinación **[MEDIR]** | Arducam IMX219 USB sobre el poste |
| IR discretos KY-032 | uno al frente y otro atrás, en la base L1 | — | | posición sin definir |

Con el yaw por defecto de 45°, la cara del Sharp izquierdo queda en R ≈ (36.1, 50.1) y la del derecho en (36.1, −50.1). En el marco P son (−25.9, ±50.1).

El TF-Luna nunca ve el robot: no hay nada sobre L4 dentro de su barrido de ±60°. En la cámara, el borde superior delantero del cabezal queda 51° por debajo de la lente, así que no aparece en la imagen mientras la inclinación más la mitad del FOV vertical sea menor que ~48°.

## Alturas sobre el piso

La altura de L4 sobre el piso depende de los separadores y todavía no está medida **[MEDIR]**. Cuando se mida H_L4 (cara superior de L4), las alturas sobre el piso son: TF-Luna H_L4 + 30.75 mm, Sharp H_L4 + 12.05 mm y cámara H_L4 + ~80 mm. Los obstáculos de la prueba se ajustan a la altura de cada equipo, así que ese es el número que hay que reportar.

## Pendientes de medición

1. Yaw real de cada Sharp en la escala → `SHARP_YAW_DEG` (Config.h) y `SHARP_YAW_DEG` (geometria_robot.py).
2. Distancia del pivote a la cara del Sharp → `SHARP_FACE_FROM_PIVOT_MM`.
3. Offset de rango del TF-Luna (lente delante del eje) → `TF_ORIGIN_X_MM`.
4. H_L4 sobre el piso, e inclinación y FOV de la cámara.
5. Al terminar, `GEOMETRY_MEASURED = true` en Config.h.
