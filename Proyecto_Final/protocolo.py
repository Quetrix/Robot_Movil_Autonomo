"""Tramas ASCII entre la Raspberry Pi y los ESP32.

Formato: TIPO,campo,campo,...*HH\\n   HH = XOR de todos los bytes antes de '*', en hex.
Las lineas que empiezan con '#' son mensajes para humanos (sin checksum).
Tipos que llegan y su significado: ver la tabla del README y el encabezado de cada .ino.
"""

# Campos numericos esperados por tipo (sin contar el tipo). None = cualquier cantidad.
CAMPOS = {
    "L": 15,   # seq, ms, c0..c3, r0..r3, p0..p3, flags
    "T": 6,    # seq, ms, ang_cdeg, dist_mm, amp, flags
    "S": 7,    # seq, ms, izq_mm, der_mm, izq_mv, der_mv, ir_tras
    "D": None,
    "I": None,
}

# Bits de flags de la trama L
L_ARMADO, L_IR_FRENTE, L_WATCHDOG, L_STALL, L_FRENANDO, L_LAZO_ABIERTO, L_IR_OFF = (1 << i for i in range(7))
# Bits de flags de la trama T
T_INVALIDA, T_EXTREMO, T_BARRIENDO, T_SIN_ANGULO = 1, 2, 4, 8


def xor(datos: bytes) -> int:
    x = 0
    for b in datos:
        x ^= b
    return x


def armar(*campos) -> bytes:
    """armar('V', 100, 0, 0) -> b'V,100,0,0*HH\\n'"""
    cuerpo = ",".join(str(c) for c in campos).encode()
    return cuerpo + b"*%02X\n" % xor(cuerpo)


def separar(linea: bytes):
    """Devuelve (tipo, campos) con los campos como texto, ('#', texto) o None si la linea es invalida."""
    linea = linea.strip()
    if not linea:
        return None
    if linea[:1] == b"#":
        return "#", linea[1:].decode(errors="replace").strip()
    cuerpo, ast, hh = linea.rpartition(b"*")
    if not ast or len(hh) != 2:
        return None
    try:
        if int(hh, 16) != xor(cuerpo):
            return None
        partes = cuerpo.decode("ascii").split(",")
    except (ValueError, UnicodeDecodeError):
        return None
    tipo, campos = partes[0], partes[1:]
    esperado = CAMPOS.get(tipo, -1)
    if esperado == -1 or (esperado is not None and len(campos) != esperado):
        return None
    return tipo, campos
