# Robot_Movil_Autonomo

Proyecto final de MT9005 — Simulación de Procesos Industriales Robotizados (Ingeniería Mecatrónica, II semestre 2026): **robot móvil autónomo para navegación por odometría e hitos visuales**.

El robot holonómico con ruedas Mecanum debe ir desde el aula de origen hasta el laboratorio F3-03 pasando entre pares de conos naranjas (puertas de 50 cm), esquivar obstáculos no mapeados y estacionarse sobre un marcador ArUco con la orientación indicada. Una vez arrancado, no se permite ninguna intervención.

## Arquitectura

```
 Raspberry Pi 5 ── NAVEGACIÓN: cámara, conos y ArUco, comportamientos, odometría
     │  vx, vy, θ̇  (serial, maestro)          ▲  matriz [ángulo; distancia]
     ▼                                         │
 ESP32 ── LOCOMOCIÓN                        ESP32 ── PERCEPCIÓN
 4 motores CD + encoders, PI por rueda,     servo MG996R + 2 Sharp GP2Y0A21
 botón e IR discretos de paro               + TF-Luna, barrido y fusión
```

## Estructura del repositorio

| Carpeta | Contenido | Estado |
|---|---|---|
| [`Avance_1_Locomocion/`](Avance_1_Locomocion/) | Firmware de locomoción (ESP32): versión actual `locomocion_esp32_ir_frontal`, variantes anteriores, binario de respaldo e historial | Presentado el 6 oct |
| [`Avance_2_Percepcion/`](Avance_2_Percepcion/) | Firmware de percepción (ESP32) | Presentado el 27 oct |
| [`Tarea_Integracion/`](Tarea_Integracion/) | Tarea 8: comportamientos de Braitenberg en la Raspberry Pi con ambos módulos; guía de armado, conexión y puesta en marcha | Entrega 10 nov |
| `Documentos/Enunciados/` | Enunciado del proyecto, guías de los módulos y lista de materiales | |
| `Documentos/Datasheets/` | ESP32-WROOM-32, expansion board, Sharp GP2Y0A21, KY-032, TF-Luna (datasheet y manual), HC-SR04 | |

Cada carpeta de módulo tiene un README con el funcionamiento, el **pinout completo**, todos los parámetros y los pendientes para la entrega final.

## Hardware

Chasís de aluminio de 120 × 180 mm, 4 ruedas Mecanum, 4 motores CD con encoder magnético, 2 drivers de 2 canales, batería LiPo con PowerBoost a 5 V, 2 ESP32-WROOM-32 DevKit con expansion board, Raspberry Pi 5 (8 GB), botón, 2 IR discretos KY-032, 2 Sharp GP2Y0A21YK0F, TF-Luna y servo MG996R. Todavía no hay IMU, y el ESP32 de locomoción no tiene pines libres para conectarla.

## Fechas

| Entrega | Fecha |
|---|---|
| Presentación y demostración | miércoles 2 dic 2026, 8:30, laboratorio F3-03 |
| Informe IEEE (≤ 6 páginas) y video | antes del viernes 4 dic 2026, 7:30, TEC Digital |
