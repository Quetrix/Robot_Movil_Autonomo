# Tarea 8 — Comportamientos de Braitenberg (integración locomoción + percepción)

`braitenberg.py` corre en la Raspberry Pi 5. Lee los dos Sharp del módulo de percepción, ejecuta una máquina de estados con 4 vehículos de Braitenberg y manda velocidades al módulo de locomoción. Los dos ESP32 usan, sin cambios, el firmware de `Avance_1_Locomocion/locomocion_esp32` y `Avance_2_Percepcion/Percepcion_MG996_180`.

## Conexión y ejecución

1. Conectar los dos ESP32 por USB a la Pi y ver sus nombres: `ls /dev/serial/by-id/`.
2. `sudo apt install -y python3-serial`
3. `python3 braitenberg.py --loco /dev/serial/by-id/<locomocion> --perc /dev/serial/by-id/<percepcion>`
4. Teclas: `1` Miedo, `2` Agresión, `3` Amor, `4` Explorador, `0`/espacio Espera, `q` salir (manda `X`).

**No presionar el botón físico:** en el firmware del Avance 1 arranca la secuencia de demostración y bloquea el comando `V`. Para detener, usar `0` o `q`; los KY-032 siguen actuando como paro.

## Arquitectura

```
ESP32 percepción ──DETAIL (IR_I, IR_D, TF)──▶  Pi: Percepcion → Robot (máquina de estados) → comportamiento
                                                                                    │ V 0 v ω
                                                              ESP32 locomoción ◀────┘  (PI por rueda, paro KY-032)
```

- **Sensores.** El cabezal queda fijo al frente (`CENTER`). El Sharp izquierdo (+60°) y el derecho (−60°) son los dos sensores del vehículo. La distancia se normaliza a un estímulo `s = clip((600 − d)/(600 − 100), 0, 1)`: 1 = muy cerca, 0 = lejos o sin lectura.
- **Máquina de estados.** `ESPERA`, `MIEDO_2A`, `AGRESION_2B`, `AMOR_3A`, `EXPLORADOR_3B` y `PARO`. Las transiciones vienen del teclado y de una guarda de seguridad: si la percepción no responde durante 1 s, el robot pasa a `PARO` y vuelve a `ESPERA` cuando se recupera. En el proyecto final, las transiciones por teclado se cambian por eventos (cono detectado, obstáculo, ArUco).
- **Robot diferencial virtual.** Cada comportamiento calcula la velocidad de una rueda izquierda y una derecha virtuales, `vI` y `vD`. Esas velocidades se convierten en `v = (vI + vD)/2` y `ω = (vD − vI)/B` (con B = 0.20 m) y se envían como `V 0 v ω`: sin desplazamiento lateral, `ω` antihorario.

## Comportamientos (V0 = 0.20 m/s, K = 0.15 m/s)

| Vehículo | Conexión | Ecuaciones | Resultado |
|---|---|---|---|
| 2a Miedo | directa, excitatoria | vI = V0 + K·sI, vD = V0 + K·sD | se aleja del obstáculo acelerando |
| 2b Agresión | cruzada, excitatoria | vI = V0 + K·sD, vD = V0 + K·sI | gira hacia el obstáculo y lo embiste (lo detiene el KY-032) |
| 3a Amor | directa, inhibitoria | vI = V0·(1 − sI), vD = V0·(1 − sD) | gira hacia el obstáculo y se detiene frente a él |
| 3b Explorador | cruzada, inhibitoria | vI = V0·(1 − sD), vD = V0·(1 − sI) | frena al acercarse, gira y sigue buscando |

Si las dos ruedas virtuales quedan por debajo de 0.04 m/s, se envía `V 0 0 0` (frenado controlado del PI).

**Pregunta 4.** Para evitar obstáculos, el **3b Explorador**: reduce la velocidad al acercarse y gira para alejarse, así que nunca llega al obstáculo. El 2a también evade, pero acelera justo cerca del obstáculo, lo que es arriesgado en un pasillo. Para explorar, también el **3b**: en espacio libre avanza a V0, se acerca con cuidado a lo que encuentra y luego se aparta para seguir recorriendo, en vez de quedarse detenido como el 3a.

## Ajustes en pruebas

- `V0`: el feedforward está calibrado desde 80 RPM (≈ 0.20 m/s). Por debajo de unos 60 RPM las ruedas pueden no arrancar. Si el robot gira poco, subir `K_EXC` o bajar `B_VIRTUAL`.
- `D_MAX_MM`: distancia a la que el robot empieza a reaccionar.
- El Sharp no mide por debajo de 10 cm (lectura inválida → estímulo 0). A esa distancia actúa el KY-032.
