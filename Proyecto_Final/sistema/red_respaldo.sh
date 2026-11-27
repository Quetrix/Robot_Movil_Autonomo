#!/bin/bash
# Hotspot del robot solo como respaldo.
# Al arrancar la Pi, NetworkManager intenta las redes WiFi guardadas (por prioridad).
# Este script espera hasta ESPERA_S segundos a que wlan0 quede conectada como cliente.
# Si lo logra, no hace nada (la Pi queda con internet). Si no, levanta el hotspot robot-ap.
# Instalacion y uso: ESTADO.md, seccion "Red de la Pi".
ESPERA_S=${ESPERA_S:-60}
AP=${AP:-robot-ap}

for ((i = 0; i < ESPERA_S; i += 2)); do
    estado=$(nmcli -g GENERAL.STATE device show wlan0 2>/dev/null)
    con=$(nmcli -g GENERAL.CONNECTION device show wlan0 2>/dev/null)
    if [[ "$estado" == 100* && -n "$con" && "$con" != "$AP" ]]; then
        logger -t red-respaldo "wlan0 conectada a '$con'; sin hotspot"
        exit 0
    fi
    sleep 2
done

logger -t red-respaldo "ninguna red guardada en ${ESPERA_S} s; levantando $AP"
nmcli con up "$AP"
