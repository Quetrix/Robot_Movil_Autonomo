# Robot_Movil_Autonomo

Proyecto final de MT9005 — Simulación de Procesos Industriales Robotizados (Ingeniería Mecatrónica, II semestre 2026): **robot móvil autónomo para navegación por odometría e hitos visuales**.

El robot holonómico con ruedas Mecanum debe ir desde el aula de origen hasta el laboratorio F3-03 pasando entre pares de conos naranjas (puertas de 50 cm), esquivar obstáculos no mapeados y estacionarse sobre un marcador ArUco con la orientación indicada. Una vez arrancado, no se permite ninguna intervención.

## Arquitectura

```
 Raspberry Pi 5 ── NAVEGACIÓN: cámara (conos, ArUco), odometría, falda, comportamientos, registro
     │  USB: V vx vy wz (mm/s, °/s)            ▲  USB: muestras TF-Luna y Sharp con ángulo y tiempo
     ▼  ▲ conteos de encoders (50 Hz)          │
 ESP32 ── LOCOMOCIÓN                        ESP32 ── PERCEPCIÓN
 4 motores CD + encoders, PI por rueda,     servo MG996R (barrido del TF-Luna),
 botón de arranque/paro, KY-032 frontal     2 Sharp fijos, KY-032 trasero
```

## Estructura del repositorio

| Carpeta | Contenido | Estado |
|---|---|---|
| [`Avance_1_Locomocion/`](Avance_1_Locomocion/) | Firmware de locomoción (ESP32) para la entrega final: `locomocion/`. Versiones anteriores, binario del Avance 1 e historial en `_respaldo/` | Presentado el 6 oct |
| [`Avance_2_Percepcion/`](Avance_2_Percepcion/) | Firmware de percepción (ESP32) para la entrega final: `percepcion/`. Versión del Avance 2 en `_respaldo/` | Presentado el 27 oct |
| [`Tarea_Integracion/`](Tarea_Integracion/) | Tarea 8 (Braitenberg). Usa los firmwares de `_respaldo/`: no es compatible con los finales | Entregada 10 nov |
| [`Proyecto_Final/`](Proyecto_Final/) | Programa de la Raspberry Pi para la prueba: navegación, visión, odometría, diagnóstico, calibraciones, registro y simulador. **Empezar por su README** | En pruebas |
| `Documentos/Fase_final_analisis.md` | Estado, decisiones y revisión crítica de la fase final | |
| `Documentos/Geometria_robot.md` | Posición y orientación de todos los sensores y marcos de referencia (montaje final) | |
| `Documentos/Enunciados/` | Enunciado del proyecto, guías de los módulos y lista de materiales | |
| `Documentos/Datasheets/` | ESP32-WROOM-32, expansion board, Sharp GP2Y0A21, KY-032, TF-Luna (datasheet y manual), HC-SR04 | |

El pinout final de los dos ESP32, el protocolo y el plan de pruebas están en `Proyecto_Final/README.md`.

## Hardware

Chasís de aluminio de 120 × 180 mm, 4 ruedas Mecanum, 4 motores CD con encoder magnético, 2 drivers de 2 canales, 2 × 18650 para los motores, LiPo con PowerBoost para el servo, power bank de 5 V / 3 A para la Pi, 2 ESP32-WROOM-32 DevKit con expansion board, Raspberry Pi 5 (8 GB), botón, 2 IR discretos KY-032, 2 Sharp GP2Y0A21YK0F, TF-Luna y servo MG996R. No se usa IMU: el rumbo se corrige con los conos de cada puerta.

## Fechas

| Entrega | Fecha |
|---|---|
| Presentación y demostración | miércoles 2 dic 2026, 8:30, laboratorio F3-03 |
| Informe IEEE (≤ 6 páginas) y video | antes del viernes 4 dic 2026, 7:30, TEC Digital |
