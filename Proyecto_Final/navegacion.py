"""Navegacion por comportamientos (maquina de estados) para la prueba final.

  ESPERA ──boton──▶ RUTA ──puerta vista──▶ ALINEAR ──en el punto de aproximacion──▶ CRUZAR
                      │  ▲                                                          │
            sin verla │  └──────────── siguiente objetivo (pose corregida con el croquis) ◀┘
                      ▼
                   BUSCAR (gira +-BUSCAR_GIRO_DEG; si no la ve, cruza con la posicion del mapa)
  despues de la ultima puerta: RUTA hacia el fin ──ArUco visto──▶ ESTACIONAR ──▶ FIN
  EVITAR se superpone a RUTA y ALINEAR cuando el pasillo de avance esta ocupado.
  Si se suelta el armado (boton), el robot se detiene y retoma donde iba al volver a armar.

Todo lo visto (puerta, marcador) se guarda en el marco odom: cuando la camara deja de verlo
(p. ej. los conos salen del campo al acercarse) se sigue usando con la odometria.
"""
import math
import time

import config
import mapa
from odometria import a_global, a_local, envolver


def saturar(v, lim):
    return max(-lim, min(lim, v))


class Navegador:
    def __init__(self, loco, perc, vision, odom, falda, registro, objetivos=None):
        self.loco, self.perc, self.vision = loco, perc, vision
        self.odom, self.falda, self.reg = odom, falda, registro
        self.objetivos = objetivos or mapa.objetivos()
        self.i = 0
        self.estado = "ESPERA"
        self.previo = None               # estado al que vuelve EVITAR
        self.t_estado = time.monotonic()
        self.puerta = None               # dict en odom: c (x, y), rumbo, conos, t
        self.marcador = None
        self.buscar_fase = 0
        self.evitar_lado = 1
        self.libre = math.inf
        self.cmd = (0.0, 0.0, 0.0)
        self._armado_antes = False
        self._t_vision = 0.0
        self._ultimo_avance = (time.monotonic(), 0.0)
        self._t_foto = 0.0
        self._meta = None

    # ------------------------------------------------------------ utilidades
    def cambiar(self, estado, motivo=""):
        if estado != self.estado:
            self.reg.evento("NAV %s -> %s %s" % (self.estado, estado, motivo))
        self.estado, self.t_estado = estado, time.monotonic()

    def objetivo(self):
        return self.objetivos[self.i] if self.i < len(self.objetivos) else None

    def siguiente(self):
        self.i += 1
        self.puerta = None
        self.buscar_fase = 0
        self.cambiar("RUTA", "objetivo %d" % self.i)

    def ir_a(self, xy_odom, rumbo_odom, v_max, ignorar=(), ancho=config.ROBOT_ANCHO_M + 0.06):
        """Control holonomico hacia un punto (odom) con rumbo deseado. Devuelve la distancia."""
        pose = self.odom.pose_odom()
        self._meta = xy_odom
        ex, ey = a_local(pose, xy_odom)
        d = math.hypot(ex, ey)
        v = min(v_max, config.K_POS * d)
        vx, vy = (v * ex / d, v * ey / d) if d > 1e-3 else (0.0, 0.0)
        wz = saturar(config.K_RUMBO * envolver(rumbo_odom - pose[2]), config.W_MAX)
        if d > 1e-3:
            self.libre = self.falda.pasillo(math.atan2(vy, vx), ancho=ancho, ignorar=ignorar)
            escala = max(0.0, min(1.0, (self.libre - 0.1) / 0.5))   # frena al acercarse
            vx, vy = vx * escala, vy * escala
        self.cmd = (vx, vy, wz)
        return d

    def rumbo_hacia(self, xy_odom):
        pose = self.odom.pose_odom()
        return math.atan2(xy_odom[1] - pose[1], xy_odom[0] - pose[0])

    def bloqueado(self):
        return self.libre < config.D_FRENO_OBST

    # ------------------------------------------------------------ percepcion visual
    def actualizar_vision(self):
        if self.vision is None:
            return
        r = self.vision.resultado
        if r["t"] <= self._t_vision:
            return
        self._t_vision = r["t"]
        pose = self.odom.pose_odom()
        obj = self.objetivo()
        if obj and obj["tipo"] == "puerta" and self.estado in ("RUTA", "BUSCAR", "ALINEAR", "EVITAR"):
            self._actualizar_puerta(r, pose, obj["puerta"])
        m = r["aruco"]
        if m and obj and obj["tipo"] == "fin":
            nh = math.hypot(m["normal"][0], m["normal"][1])
            if nh > 0.5:      # marcador en pared: quedar enfrente, a D_ESTACIONAR
                n = (m["normal"][0] / nh, m["normal"][1] / nh)
                meta = (m["x"] + config.D_ESTACIONAR * n[0], m["y"] + config.D_ESTACIONAR * n[1])
                rumbo = math.atan2(-n[1], -n[0])
            else:             # marcador en el piso: llegar con su 'arriba' hacia adelante
                rumbo = math.atan2(m["arriba"][1], m["arriba"][0])
                meta = (m["x"] - config.D_ESTACIONAR * math.cos(rumbo), m["y"] - config.D_ESTACIONAR * math.sin(rumbo))
            nuevo = {"meta": a_global(pose, meta), "rumbo": envolver(pose[2] + rumbo), "t": r["t"], "id": m["id"]}
            if self.marcador is None:
                self.reg.evento("ArUco %s id %d visto" % (m["dic"], m["id"]))
                self._foto("aruco")
                self.marcador = nuevo
            else:
                a = 0.3
                mk = self.marcador
                mk["meta"] = tuple(p + a * (q - p) for p, q in zip(mk["meta"], nuevo["meta"]))
                mk["rumbo"] = envolver(mk["rumbo"] + a * envolver(nuevo["rumbo"] - mk["rumbo"]))
                mk["t"] = r["t"]

    def _actualizar_puerta(self, r, pose, p):
        """Memoria de la puerta en odom. La orientacion medida con dos conos lejanos es muy ruidosa
        (4 % de error en distancia a 3 m = +-20 grados), asi que lejos manda el rumbo del croquis y
        cerca se acepta la medicion, limitada a +-30 grados del croquis."""
        rumbo_mapa = self._rumbo_odom(p.rumbo)
        if r["puerta"]:
            cx, cy, rumbo, dist, conos = r["puerta"]
            c = a_global(pose, (cx, cy))
            medido = envolver(pose[2] + rumbo)
            peso = 0.0 if dist > 1.5 else 0.7
            rumbo_obs = envolver(rumbo_mapa + peso * saturar(envolver(medido - rumbo_mapa), math.radians(30)))
            conos_o = [a_global(pose, k) for k in conos]
            if self.puerta is None:
                if math.dist(c, self.odom.mundo_a_odom((p.x, p.y))) < 1.5:   # el croquis no esta a escala
                    self.puerta = {"c": c, "rumbo": rumbo_obs, "t": r["t"], "conos": conos_o}
                    self.reg.evento("Puerta %s vista a %.2f m" % (p.nombre, dist))
                    self._foto("puerta")
                return
            if math.dist(c, self.puerta["c"]) < 0.4:
                a = 0.5 if dist < 1.5 else 0.25
                q = self.puerta
                q["c"] = (q["c"][0] + a * (c[0] - q["c"][0]), q["c"][1] + a * (c[1] - q["c"][1]))
                q["rumbo"] = envolver(q["rumbo"] + a * envolver(rumbo_obs - q["rumbo"]))
                q["conos"], q["t"] = conos_o, r["t"]
        elif self.puerta is not None:
            # Un solo cono visible: corrige la posicion de la memoria sin tocar su orientacion.
            q = self.puerta
            for k in r["conos"]:
                g = a_global(pose, (k["xr"], k["yr"]))
                cerca = min(q["conos"], key=lambda m: math.dist(m, g))
                if math.dist(cerca, g) < 0.25 and k["dist"] < 2.0:
                    dx, dy = 0.3 * (g[0] - cerca[0]), 0.3 * (g[1] - cerca[1])
                    q["c"] = (q["c"][0] + dx, q["c"][1] + dy)
                    q["conos"] = [(m[0] + dx, m[1] + dy) for m in q["conos"]]
                    q["t"] = r["t"]
                    break

    def _foto(self, etiqueta):
        if self.vision is not None and self.vision.ultimo_cuadro is not None:
            self.reg.foto(self.vision.ultimo_cuadro, etiqueta)

    def ignorar_conos(self):
        if not self.puerta:
            return ()
        pose = self.odom.pose_odom()
        return [(*a_local(pose, k), config.RADIO_IGNORAR_CONO) for k in self.puerta["conos"]]

    # ------------------------------------------------------------ comportamientos
    def paso(self):
        ahora = time.monotonic()
        armado = self.loco.armado
        if armado and not self._armado_antes:
            if self.estado == "ESPERA":
                self.odom.fijar_mundo((mapa.INICIO[0], mapa.INICIO[1], math.radians(mapa.INICIO[2])))
                self.perc.barrer()
                self.cambiar("RUTA", "(boton)")
            else:
                self.reg.evento("Rearmado: continua en %s" % self.estado)
            self._ultimo_avance = (ahora, self.odom.recorrido)
        if not armado and self._armado_antes:
            self.reg.evento("Desarmado (paro) en %s" % self.estado)
        self._armado_antes = armado
        self.perc.keepalive()
        self.actualizar_vision()
        self.cmd = (0.0, 0.0, 0.0)
        self.libre = math.inf
        if armado and self.estado not in ("ESPERA", "FIN"):
            getattr(self, "_" + self.estado.lower())(ahora)
            self._vigilar_avance(ahora)
        if armado:
            self.loco.velocidad(*self.cmd)
        if self.vision is not None and config.FOTO_CADA_S and ahora - self._t_foto > config.FOTO_CADA_S \
                and self.estado not in ("ESPERA", "FIN"):
            self._t_foto = ahora
            self._foto(self.estado.lower())
        return self.cmd

    def _vigilar_avance(self, ahora):
        t0, d0 = self._ultimo_avance
        if self.odom.recorrido - d0 > 0.05 or self.estado == "BUSCAR":
            self._ultimo_avance = (ahora, self.odom.recorrido)
        elif ahora - t0 > config.T_SIN_AVANCE_S:
            self._ultimo_avance = (ahora, self.odom.recorrido)
            self.reg.evento("Sin avance %d s en %s: se fuerza EVITAR" % (config.T_SIN_AVANCE_S, self.estado))
            self.evitar_lado = -self.evitar_lado
            self._entrar_evitar()

    def _entrar_evitar(self):
        if self.estado != "EVITAR":
            self.previo = self.estado
        izq = self.falda.pasillo(math.radians(60))
        der = self.falda.pasillo(math.radians(-60))
        self.evitar_lado = 1 if izq >= der else -1
        self._evitar_inicio = self.odom.recorrido
        self.cambiar("EVITAR", "libre=%.2f izq=%.2f der=%.2f" % (self.libre, izq, der))

    def _ruta(self, ahora):
        obj = self.objetivo()
        if obj is None:
            return self.cambiar("FIN", "sin objetivos")
        if obj["tipo"] == "puerta":
            if self.puerta is not None:
                return self.cambiar("ALINEAR")
            meta = self.odom.mundo_a_odom(obj["puerta"].punto(-config.D_APROX_PUERTA))
            d = self.ir_a(meta, self.rumbo_hacia(meta), config.V_CRUCERO)
            if d < 0.25:
                self.buscar_fase = 0
                return self.cambiar("BUSCAR", "no veo %s" % obj["puerta"].nombre)
        elif obj["tipo"] == "paso":
            meta = self.odom.mundo_a_odom(obj["xy"])
            if self.ir_a(meta, self.rumbo_hacia(meta), config.V_CRUCERO) < 0.15:
                return self.siguiente()
        else:   # fin: acercarse buscando el marcador
            if self.vision is not None:
                self.vision.buscar_aruco_siempre = True
            if self.marcador is not None:
                return self.cambiar("ESTACIONAR")
            fin = obj["xy"]
            aprox = (fin[0] - 0.6 * math.cos(obj["rumbo"]), fin[1] - 0.6 * math.sin(obj["rumbo"]))
            meta = self.odom.mundo_a_odom(aprox)
            if self.ir_a(meta, self._rumbo_odom(obj["rumbo"]), config.V_CRUCERO) < 0.1:
                self.buscar_fase = 0
                return self.cambiar("BUSCAR", "sin ArUco a la vista")
        if self.bloqueado():
            self._entrar_evitar()

    def _rumbo_odom(self, rumbo_mundo):
        return envolver(rumbo_mundo - self.odom.correccion[2])

    def _buscar(self, ahora):
        obj = self.objetivo()
        if obj["tipo"] == "puerta" and self.puerta is not None:
            return self.cambiar("ALINEAR")
        if obj["tipo"] == "fin" and self.marcador is not None:
            return self.cambiar("ESTACIONAR")
        base = self._rumbo_odom(obj["puerta"].rumbo if obj["tipo"] == "puerta" else obj["rumbo"])
        giros = [math.radians(config.BUSCAR_GIRO_DEG), -math.radians(config.BUSCAR_GIRO_DEG), 0.0]
        if self.buscar_fase >= len(giros):
            if obj["tipo"] == "fin":
                self.buscar_fase = 0              # sigue buscando el marcador
                return
            p = obj["puerta"]                     # cruzar con la posicion del croquis
            self.puerta = {"c": self.odom.mundo_a_odom((p.x, p.y)), "rumbo": self._rumbo_odom(p.rumbo),
                           "t": ahora, "conos": [self.odom.mundo_a_odom(k) for k in p.conos()]}
            self.reg.evento("Puerta %s no vista: se cruza con el croquis" % p.nombre)
            return self.cambiar("ALINEAR")
        objetivo = envolver(base + giros[self.buscar_fase])
        err = envolver(objetivo - self.odom.pose_odom()[2])
        self.cmd = (0.0, 0.0, saturar(1.5 * err, math.radians(35)))
        if abs(err) < math.radians(4):
            self.buscar_fase += 1

    def _alinear(self, ahora):
        p = self.puerta
        n = (math.cos(p["rumbo"]), math.sin(p["rumbo"]))
        meta = (p["c"][0] - config.D_APROX_PUERTA * n[0], p["c"][1] - config.D_APROX_PUERTA * n[1])
        d = self.ir_a(meta, p["rumbo"], config.V_CRUCERO, ignorar=self.ignorar_conos())
        err = abs(envolver(p["rumbo"] - self.odom.pose_odom()[2]))
        if d < config.TOL_POS * 2 and err < config.TOL_RUMBO:
            return self.cambiar("CRUZAR")
        if self.bloqueado():
            self._entrar_evitar()

    def _cruzar(self, ahora):
        p = self.puerta
        n = (math.cos(p["rumbo"]), math.sin(p["rumbo"]))
        meta = (p["c"][0] + config.D_SALIDA_PUERTA * n[0], p["c"][1] + config.D_SALIDA_PUERTA * n[1])
        # Mientras cruza no esquiva: los conos de la puerta se ignoran y el resto lo cuida el freno.
        d = self.ir_a(meta, p["rumbo"], config.V_CRUCE, ignorar=self.ignorar_conos(), ancho=config.ROBOT_ANCHO_M)
        if d < 0.08:
            obj = self.objetivo()["puerta"]
            rel = a_local((p["c"][0], p["c"][1], p["rumbo"]), self.odom.pose_odom()[:2])
            dth = envolver(self.odom.pose_odom()[2] - p["rumbo"])
            c, s = math.cos(obj.rumbo), math.sin(obj.rumbo)
            mundo = (obj.x + c * rel[0] - s * rel[1], obj.y + s * rel[0] + c * rel[1], envolver(obj.rumbo + dth))
            antes = self.odom.pose_mundo()
            self.odom.fijar_mundo(mundo)
            self.reg.evento("Cruzada %s; pose corregida %.2f,%.2f,%.0f -> %.2f,%.2f,%.0f" % (
                obj.nombre, antes[0], antes[1], math.degrees(antes[2]), mundo[0], mundo[1], math.degrees(mundo[2])))
            self.siguiente()

    def _evitar(self, ahora):
        lado = self.evitar_lado
        rumbo_lat = lado * math.radians(60)        # borde del barrido del TF: sigue viendo hacia donde va
        lat = self.falda.pasillo(rumbo_lat, ignorar=self.ignorar_conos())
        if lat < 0.12:
            if ahora - self.t_estado > 0.5:
                self.evitar_lado = -lado
                self.t_estado = ahora
                self.reg.evento("EVITAR: lado bloqueado, cambio de lado")
            self.cmd = (-0.08, 0.0, 0.0) if not self.perc.ir_trasero else (0.0, 0.0, 0.0)
            return
        v = config.V_LATERAL_EVITAR
        self.cmd = (v * math.cos(rumbo_lat), v * math.sin(rumbo_lat), 0.0)
        meta = a_local(self.odom.pose_odom(), self._meta) if self._meta else (1.0, 0.0)
        self.libre = self.falda.pasillo(math.atan2(meta[1], meta[0]), ignorar=self.ignorar_conos())
        movido = self.odom.recorrido - self._evitar_inicio
        if self.libre > config.D_LIBRE_OBST and movido > 0.10:
            self.cambiar(self.previo or "RUTA", "pasillo libre")
        elif ahora - self.t_estado > 10:
            self.evitar_lado = -lado
            self.t_estado = ahora

    def _estacionar(self, ahora):
        mk = self.marcador
        d = self.ir_a(mk["meta"], mk["rumbo"], config.V_APROX)
        err = abs(envolver(mk["rumbo"] - self.odom.pose_odom()[2]))
        if d < config.TOL_FINAL_POS and err < config.TOL_FINAL_RUMBO:
            self.cmd = (0.0, 0.0, 0.0)
            self.loco.velocidad(0, 0, 0)
            self.cambiar("FIN", "estacionado (marcador %d)" % mk["id"])

    # ------------------------------------------------------------ registro
    def fila(self):
        pm, po = self.odom.pose_mundo(), self.odom.pose_odom()
        r = self.vision.resultado if self.vision else {"conos": [], "puerta": None, "aruco": None}
        return {
            "estado": self.estado, "obj": self.i,
            "x": "%.3f" % pm[0], "y": "%.3f" % pm[1], "th": "%.1f" % math.degrees(pm[2]),
            "xo": "%.3f" % po[0], "yo": "%.3f" % po[1], "tho": "%.1f" % math.degrees(po[2]),
            "vx": "%.3f" % self.cmd[0], "vy": "%.3f" % self.cmd[1], "wz": "%.3f" % self.cmd[2],
            "libre": "%.2f" % min(self.libre, 9.99), "conos": len(r["conos"]),
            "puerta_vista": int(r["puerta"] is not None), "aruco": int(r["aruco"] is not None),
            "ir_f": int(self.loco.ir_frente), "ir_t": int(self.perc.ir_trasero), "armado": int(self.loco.armado),
        }
