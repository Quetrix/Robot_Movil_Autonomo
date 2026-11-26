"""Enlaces USB con los dos ESP32: hilo lector por puerto, verificacion de tramas y contadores.

Fuentes de perdida de datos que se cubren aqui:
  - nombres /dev/ttyUSB* que cambian de orden: los puertos se identifican con '?' (I,LOCO / I,PERC)
    o se fijan por /dev/serial/by-id;
  - reinicio del ESP32 al abrir el puerto: DTR y RTS quedan en bajo antes de abrir;
  - lineas cortadas o corruptas: checksum XOR; se cuentan y se descartan;
  - tramas perdidas: cada tipo trae un numero de secuencia; los huecos se cuentan;
  - bloqueo del programa principal: cada puerto se lee en su propio hilo, con marca de tiempo
    de la Pi en el momento de llegada;
  - desconexion USB: se reintenta abrir el mismo puerto cada segundo.
"""
import glob
import os
import threading
import time

import config
import protocolo as P

try:
    import serial
except ImportError:  # el simulador no necesita pyserial
    serial = None


def abrir_puerto(ruta, timeout=0.05):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = ruta, config.BAUDIOS, timeout
    s.dtr = False     # sin esto, abrir el puerto reinicia el ESP32 (EN conectado a DTR/RTS)
    s.rts = False
    s.open()
    return s


def candidatos():
    """Puertos seriales posibles. En Linux se usan /dev/serial/by-path: dependen del puerto USB
    fisico y no cambian. /dev/serial/by-id NO sirve con dos ESP32 iguales: los CP2102 traen todos
    el mismo numero de serie (0001) y solo queda un enlace para los dos. En Windows/macOS: COMx."""
    rutas, vistos = [], set()
    for r in sorted(glob.glob("/dev/serial/by-path/*"), key=lambda r: ("usbv2" in r, r)):
        real = os.path.realpath(r)        # udev crea dos nombres (usb- y usbv2-) por dispositivo
        if real not in vistos:
            vistos.add(real)
            rutas.append(r)
    if len(rutas) < len(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        rutas = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not rutas and serial is not None:
        from serial.tools import list_ports
        rutas = [p.device for p in list_ports.comports() if p.vid is not None]
    return rutas


def identificar(ruta, espera_s=3.0):
    """Devuelve 'LOCO', 'PERC' o None.

    En Linux abrir el puerto puede reiniciar el ESP32 (las lineas DTR/RTS cambian al abrir, y la
    placa las usa para el reset automatico). Por eso se espera hasta 3 s preguntando '?' cada
    0.3 s: el ESP32 tarda ~0.5 s en arrancar y tambien se identifica solo al encender."""
    try:
        s = abrir_puerto(ruta, timeout=0.1)
    except Exception:
        return None
    try:
        fin = time.monotonic() + espera_s
        t_pregunta = 0.0
        resto = b""
        while time.monotonic() < fin:
            if time.monotonic() - t_pregunta > 0.3:
                s.write(b"?\n")
                t_pregunta = time.monotonic()
            resto += s.read(max(1, s.in_waiting))
            *lineas, resto = resto.split(b"\n")
            for linea in lineas:
                r = P.separar(linea)
                if r and r[0] == "I" and r[1]:
                    return r[1][0]
        return None
    finally:
        s.close()


def buscar_puertos():
    encontrados = {}
    for ruta in candidatos():
        quien = identificar(ruta)
        if quien and quien not in encontrados:
            encontrados[quien] = ruta
    return encontrados


class Enlace:
    nombre = "?"

    def __init__(self, ruta, registro, puerto=None):
        self.ruta = ruta
        self.registro = registro
        self._puerto = puerto            # inyectado por el simulador
        self._lock_tx = threading.Lock()
        self._vivo = threading.Event()
        self.version = "?"
        self.ultimo_d = None
        self.t_ultima = 0.0              # tiempo de llegada de la ultima trama valida
        self.stats = {"ok": 0, "malas": 0, "perdidas": 0, "texto": 0, "reconexiones": 0, "tx_error": 0}
        self._seq = {}

    # ------------------------------------------------------------ ciclo de vida
    def iniciar(self):
        if self._puerto is None:
            self._puerto = abrir_puerto(self.ruta)
        self._vivo.set()
        threading.Thread(target=self._leer, name="rx-" + self.nombre, daemon=True).start()
        self.enviar("?")
        return self

    def detener(self):
        self._vivo.clear()

    def conectado(self):
        return time.monotonic() - self.t_ultima < config.TIMEOUT_DATOS_S

    # ------------------------------------------------------------ envio
    def enviar(self, *campos):
        trama = P.armar(*campos)
        with self._lock_tx:
            try:
                self._puerto.write(trama)
            except Exception:
                self.stats["tx_error"] += 1
                return False
        self.registro.crudo(self.nombre, "TX", trama)
        return True

    # ------------------------------------------------------------ recepcion
    def _leer(self):
        resto = b""
        while self._vivo.is_set():
            try:
                # read(1) espera hasta el timeout; luego se toma todo lo que haya (sin latencia extra)
                datos = self._puerto.read(1)
                if datos:
                    datos += self._puerto.read(self._puerto.in_waiting)
            except Exception:
                self._reconectar()
                resto = b""
                continue
            if not datos:
                continue
            ahora = time.monotonic()
            lineas = (resto + datos).split(b"\n")
            resto = lineas.pop()
            if len(resto) > 512:          # basura sin salto de linea (p. ej. arranque del ESP32)
                resto = b""
                self.stats["malas"] += 1
            for linea in lineas:
                self._procesar(linea, ahora)

    def _procesar(self, linea, ahora):
        if not linea.strip():
            return
        self.registro.crudo(self.nombre, "RX", linea)
        r = P.separar(linea)
        if r is None:
            self.stats["malas"] += 1
            return
        tipo, campos = r
        if tipo == "#":
            self.stats["texto"] += 1
            self.registro.evento("[%s] %s" % (self.nombre, campos))
            return
        self.stats["ok"] += 1
        if tipo == "I":
            self.version = ",".join(campos[1:])
            return
        try:
            valores = [int(c) for c in campos]
        except ValueError:
            self.stats["malas"] += 1
            return
        seq = valores[0]
        anterior = self._seq.get(tipo)
        if anterior is not None and seq > anterior + 1:
            self.stats["perdidas"] += seq - anterior - 1
        self._seq[tipo] = seq
        if tipo == "D":
            self.ultimo_d = valores
            return
        self.t_ultima = ahora
        self.trama(tipo, valores, ahora)

    def trama(self, tipo, v, t_rx):
        raise NotImplementedError

    def _reconectar(self):
        if self.ruta is None:            # simulador
            time.sleep(0.1)
            return
        try:
            self._puerto.close()
        except Exception:
            pass
        self.registro.evento("[%s] USB desconectado; reintentando %s" % (self.nombre, self.ruta))
        while self._vivo.is_set():
            time.sleep(1.0)
            if not os.path.exists(self.ruta):
                continue
            try:
                self._puerto = abrir_puerto(self.ruta)
                self.stats["reconexiones"] += 1
                self.registro.evento("[%s] reconectado" % self.nombre)
                return
            except Exception:
                pass


class Locomocion(Enlace):
    """Trama L a 50 Hz. Alimenta la odometria con los conteos acumulados (sin perdida aunque
    falten tramas: el siguiente conteo trae todo lo recorrido)."""
    nombre = "loco"

    def __init__(self, ruta, registro, odometria, puerto=None):
        super().__init__(ruta, registro, puerto)
        self.odometria = odometria
        self.conteos = None
        self.ref_rpm = [0] * 4
        self.pwm = [0] * 4
        self.flags = 0
        self._t_cmd = 0.0

    def trama(self, tipo, v, t_rx):
        if tipo != "L":
            return
        self.conteos = v[2:6]
        self.ref_rpm = v[6:10]
        self.pwm = v[10:14]
        self.flags = v[14]
        self.odometria.actualizar(self.conteos, v[1] / 1000.0)

    @property
    def armado(self):
        return bool(self.flags & P.L_ARMADO)

    @property
    def ir_frente(self):
        return bool(self.flags & P.L_IR_FRENTE) and not self.flags & P.L_IR_OFF

    def velocidad(self, vx, vy, wz):
        """vx, vy en m/s y wz en rad/s (marco robot). Se envian enteros mm/s y grados/s."""
        self._t_cmd = time.monotonic()
        return self.enviar("V", round(vx * 1000), round(vy * 1000), round(wz * 57.29578))

    def parar(self):
        return self.enviar("X")


class Percepcion(Enlace):
    """Tramas T (TF-Luna, 100 Hz) y S (Sharp + IR trasero, 25 Hz) hacia la falda."""
    nombre = "perc"

    def __init__(self, ruta, registro, falda, puerto=None):
        super().__init__(ruta, registro, puerto)
        self.falda = falda
        self.ultimo_t = None
        self.ultimo_s = None
        self.ir_trasero = False
        self._t_keepalive = 0.0

    def trama(self, tipo, v, t_rx):
        if tipo == "T":
            self.ultimo_t = v
            if not v[5] & (P.T_INVALIDA | P.T_SIN_ANGULO):
                self.falda.agregar_tf(v[2] / 100.0, v[3] / 1000.0, t_rx)
        elif tipo == "S":
            self.ultimo_s = v
            self.ir_trasero = v[6] == 1
            self.falda.agregar_sharp(v[2] / 1000.0, v[3] / 1000.0, t_rx)

    def barrer(self, amplitud=config.BARRIDO_AMPLITUD_DEG, vel=config.BARRIDO_VEL_DPS):
        return self.enviar("B", amplitud, vel)

    def fijar(self, angulo):
        return self.enviar("F", angulo)

    def keepalive(self):
        if time.monotonic() - self._t_keepalive >= config.PERIODO_KEEPALIVE_PERC_S:
            self._t_keepalive = time.monotonic()
            self.enviar("K")
