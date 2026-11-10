#!/bin/bash
# Diagnostico rapido de la Raspberry Pi para la Tarea 8.
# Uso (en la Pi, por SSH):  bash ~/Robot_Movil_Autonomo/Tarea_Integracion/revisar.sh
ok()  { echo "  [OK]    $*"; }
mal() { echo "  [FALLA] $*"; }

echo "Usuario: $(whoami)   Equipo: $(hostname)   Carpeta actual: $(pwd)"

if groups | grep -qw dialout; then ok "usuario en el grupo dialout"
else mal "falta dialout: sudo usermod -aG dialout $(whoami)  y luego exit y volver a entrar por SSH"; fi

if python3 -c "import serial" 2>/dev/null; then ok "python3-serial instalado"
else mal "falta pyserial: sudo apt install -y python3-serial"; fi

if [ -f ~/Robot_Movil_Autonomo/Tarea_Integracion/braitenberg.py ]; then ok "codigo en ~/Robot_Movil_Autonomo"
else mal "no esta el codigo en ~/Robot_Movil_Autonomo (paso de git clone)"; fi

if ls /dev/serial/by-id/* >/dev/null 2>&1; then
  ok "ESP32 conectado(s) por USB:"
  for p in /dev/serial/by-id/*; do echo "          $p"; done
else mal "no hay ningun ESP32 en /dev/serial/by-id (revisar cable USB)"; fi

if nmcli -t -f NAME con show 2>/dev/null | grep -qx cabezal; then ok "perfil WiFi 'cabezal' creado"
else mal "falta el perfil WiFi 'cabezal' (paso nmcli con add)"; fi

RED=$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | grep '^yes:' | head -1 | cut -d: -f2-)
echo "  WiFi actual: ${RED:-ninguna}"
if [ "$RED" = "Percepcion-ESP32" ]; then
  if ip -4 addr show wlan0 | grep -q "192.168.4.200"; then ok "IP 192.168.4.200 en wlan0"
  else mal "la IP no es 192.168.4.200"; fi
  if curl -s -m 2 http://192.168.4.1/data | grep -q '"samples"'; then ok "el cabezal responde en http://192.168.4.1/data"
  else mal "el cabezal no responde (encendido? cerca?)"; fi
else
  echo "  (no estas en la red del cabezal: se omite la prueba del cabezal)"
fi
