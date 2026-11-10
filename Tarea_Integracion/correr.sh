#!/bin/bash
# Arranca braitenberg.py buscando solo el puerto del ESP32 de locomocion.
# Uso (en la Pi, por SSH):  bash ~/Robot_Movil_Autonomo/Tarea_Integracion/correr.sh
# Opciones extra se pasan tal cual, p. ej.:  bash correr.sh --invertir-sharp
# Guarda el registro en Tarea_Integracion/corrida_HHMMSS.csv
cd "$(dirname "$0")" || exit 1
PUERTOS=(/dev/serial/by-id/*)
if [ ! -e "${PUERTOS[0]}" ]; then
  echo "No hay ningun ESP32 conectado por USB (/dev/serial/by-id esta vacio). Revisa el cable."
  exit 1
fi
if [ ${#PUERTOS[@]} -gt 1 ] && [ -z "$LOCO" ]; then
  echo "Hay mas de un ESP32 conectado:"
  printf '  %s\n' "${PUERTOS[@]}"
  echo "Indica cual es el de locomocion, por ejemplo:"
  echo "  LOCO=${PUERTOS[0]} bash correr.sh --perc ${PUERTOS[1]}"
  exit 1
fi
LOCO=${LOCO:-${PUERTOS[0]}}
echo "Locomocion: $LOCO"
exec python3 braitenberg.py --loco "$LOCO" --log "corrida_$(date +%H%M%S).csv" "$@"
