# Percepción (ESP32)

`percepcion/percepcion.ino` es el firmware de la entrega final, para el montaje v2: TF-Luna sobre el servo MG996R, Sharp fijos en L4 y KY-032 trasero opcional. Es esclavo de la Raspberry Pi por USB y no usa WiFi. Protocolo, pinout y pruebas en [`../Proyecto_Final/README.md`](../Proyecto_Final/README.md). Geometría en [`../Documentos/Geometria_robot.md`](../Documentos/Geometria_robot.md).

Funcionamiento:

- **Barrido.** Continuo, de ±60° a 150°/s con rampas en los extremos, unos 0.8 s por pasada.
- **TF-Luna.** Trabaja en modo continuo a 100 Hz. Cada medición se lee cuando sube su pin 6 (*data ready*) y sale con el ángulo estimado: la consigna del servo de *lag* ms antes, con un valor por sentido.
- **Sharp.** Se leen a 25 Hz, con la mediana de 5 lecturas del ADC. Se convierten con la curva genérica d = 29.988·V^−1.173 cm hasta tener las tablas medidas (`calibrar.py sharp`).
- **Arranque y enlace.** No hay movimiento hasta que la Pi manda `B` o `F`. Si pasan 3 s sin ninguna línea de la Pi, el cabezal vuelve al centro.

Para probarlo sin la Pi: monitor serial a **115200** baud, `B,60,150` para barrer, `F,0` para dejarlo al centro y `R` para quitar la señal del servo. El monitor muestra las tramas `T`, `S` y `D`. Escriban cualquier línea cada menos de 3 s, porque si no el cabezal vuelve al centro.

Pendientes de calibración (marcados `[CALIBRAR]` en el código):

- `SERVO_US_MENOS60/CENTRO/MAS60`: medir con transportador.
- `lagCcwMs` y `lagCwMs`: `calibrar.py lag`.
- `SHARP_TABLA`: `calibrar.py sharp`.

Sobre el registro ENABLE (0x25) del TF-Luna: el manual A03 dice 0 = encendido, pero la versión anterior escribía 1 y funcionaba. El firmware no lo toca. Si no llegan datos, prueba 1 y después 0, y avisa por serial.

`_respaldo/` guarda la versión del Avance 2 (v1.2, con panel web, memoria angular y fusión) y su README.
