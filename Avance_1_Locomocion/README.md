# Locomoción (ESP32)

`locomocion/locomocion.ino` es el firmware de la entrega final: esclavo de la Raspberry Pi por USB, sin WiFi. Protocolo, pinout, alimentación y pruebas en [`../Proyecto_Final/README.md`](../Proyecto_Final/README.md).

Se compila en Arduino IDE con el core **esp32 3.x** y la placa "ESP32 Dev Module", sin librerías externas. Probado con el core 3.3.2: sin advertencias, 24 % de la flash.

Para probarlo sin la Pi: abrir el monitor serial a **230400** baud, escribir `A,1` (armar) y luego `V,100,0,0` (adelante a 100 mm/s), `V,0,100,0` (izquierda) o `V,0,0,45` (giro antihorario). Si no se repite el `V`, el robot frena solo a los 0.5 s. `X` para.

Respecto a la versión de la Tarea 8 (`_respaldo/locomocion_esp32_ir_frontal`) se conservaron pinout, signos, CPR, geometría, ganancias PI, feedforward, rampas y detección de stall. Cambió:

- **Protocolo y convención.** `V,vx,vy,wz` con X adelante, Y izquierda, giro antihorario, en mm/s y °/s, como pide el enunciado. Antes era vx = derecha, vy = adelante, en m/s.
- **Telemetría.** Trama `L` a 50 Hz con los conteos acumulados de los encoders para la odometría de la Pi, y diagnóstico `D` a 1 Hz. Todas las tramas llevan checksum y secuencia, y se envían sin bloquear el lazo de control.
- **Botón.** Arma y desarma (arranque y paro de emergencia). Ya no lanza la secuencia del Avance 1.
- **Watchdog.** Sin comando durante 500 ms, el robot frena con rampa.
- **Se quitó.** La secuencia del Avance 1, el giro 360, la Tarea 5, los comandos W/S/A/D/R/H y los textos de ayuda.
- **Consumo y lectura de encoders.** El ESP32 corre a 80 MHz, y los encoders se leen por registro en la interrupción.

`_respaldo/` guarda las variantes anteriores, el binario exacto del Avance 1 (`firmware_respaldo/`), el historial y el README del Avance 1 (`README_avance1.md`, con el detalle del cableado de drivers y conectores).
