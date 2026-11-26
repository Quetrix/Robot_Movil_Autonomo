// ============================================================
// MT9005 - MODULO DE PERCEPCION (ESP32, entrega final)
// Esclavo de la Raspberry Pi 5 por USB. Sin WiFi ni Bluetooth.
//
// Montaje v2 (Documentos/Geometria_robot.md): TF-Luna sobre el servo
// MG996R con barrido continuo, 2 Sharp GP2Y0A21 fijos en L4 y, opcional,
// el KY-032 trasero.
//
// Reemplaza a _respaldo/Percepcion_MG996_180 (v1.2): se elimino el panel
// web, la memoria angular y la fusion. El ESP32 entrega cada muestra con
// su tiempo y su angulo estimado; la Pi arma la matriz de angulos y
// distancias (Proyecto_Final/falda.py).
//
// Frecuencias de consulta (maximas de cada sensor):
//   TF-Luna: modo continuo a 100 Hz; se lee cuando su pin 6 (data ready)
//            sube, asi que no se repiten ni se pierden tramas.
//   Sharp:   25 Hz (el sensor actualiza cada 38 +/- 10 ms).
// Angulo del TF-Luna: el servo no tiene realimentacion. Se guarda la
// trayectoria comandada y el angulo de cada muestra es el comandado
// LAG ms antes (un LAG por sentido, calibrable con el comando Z).
//
// Protocolo (115200 baud, lineas ASCII, sufijo *HH = XOR opcional en
// lo que llega y siempre en lo que sale). Marco: angulos antihorarios
// positivos (izquierda), 0 = frente.
//   Pi -> ESP32
//     B,amp,vel     barrido continuo +/-amp grados a vel grados/s
//     F,ang         mantener un angulo fijo (amp 0)
//     R             retirar la senal del servo
//     Z,lag_ccw,lag_cw   retardo del servo en ms por sentido
//     ?             identificacion
//     (cualquier linea valida sirve de keepalive; sin lineas en 3 s el
//      cabezal vuelve al centro y se queda quieto)
//   ESP32 -> Pi
//     I,PERC,version
//     T,seq,ms,ang_cdeg,dist_mm,amp,flags          (100 Hz)
//        flags: bit0 invalida, bit1 cerca del extremo del barrido,
//               bit2 barriendo, bit3 servo sin senal
//     S,seq,ms,izq_mm,der_mm,izq_mv,der_mv,ir_tras (25 Hz)
//        mm = 0: nada dentro de 100..800 mm; ir_tras 1 = obstaculo
//     D,seq,ms,lazo_max_us,rx_malas,tx_perdidas,i2c_err,tf_validas_s,modo  (1 Hz)
//     # texto
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/mcpwm_prelude.h"

const char VERSION[] = "perc-final-1.1";

// ---------------- Pines ----------------
const int SHARP_PIN[2] = {34, 35};   // izquierdo, derecho (ADC1)
const int PIN_SDA = 21, PIN_SCL = 22;
const int TF_DR = 27;                // pin 6 del TF-Luna (data ready en modo I2C)
const int SERVO = 25;
const int IR_TRASERO = 32;           // KY-032 trasero, opcional (pull-up interno)
const int LED_ESTADO = 2;

// ---------------- Servo ----------------
// Anclas iniciales, no calibradas: medir -60, 0 y +60 grados en ESTE servo.
const float SERVO_US_MENOS60 = 1167, SERVO_US_CENTRO = 1500, SERVO_US_MAS60 = 1833;
const int SERVO_SIGNO = 1;           // -1 si un angulo positivo gira a la derecha
const float SERVO_LIMITE_DEG = 60;   // limite de firmware (sin topes mecanicos)
const float ACEL_DPS2 = 1500;        // rampa en los extremos del barrido
const uint32_t SERVO_TS_MS = 20;     // nueva consigna cada periodo de PWM (50 Hz)
float lagCcwMs = 60, lagCwMs = 60;   // [CALIBRAR] calibrar.py lag

// ---------------- TF-Luna ----------------
const uint8_t TF_DIR = 0x10;
const uint16_t TF_FPS = 100;
const uint32_t TF_RETARDO_US = 5000; // la medicion ocurre ~medio cuadro antes del data ready
const float TF_MIN_MM = 200, TF_MAX_MM = 8000;
// Correccion medida en el Avance 2 (200..2100 mm): mm = 1.0068*crudo - 11.3
const float TF_ESCALA = 1.0068f, TF_OFFSET_MM = -11.3f, TF_CAL_MIN = 200, TF_CAL_MAX = 2100;

// ---------------- Sharp GP2Y0A21YK0F ----------------
// Curva generica d[cm] = 29.988 * V^-1.173 (ajuste comun de la hoja de datos).
// [CALIBRAR] con calibrar.py sharp y reemplazar por la tabla medida.
struct PuntoCal { float mv; float mm; };
const PuntoCal SHARP_TABLA[2][1] = {{{0, 0}}, {{0, 0}}};
const int SHARP_N[2] = {0, 0};       // 0 = usar la curva generica
const float SHARP_MV_LEJOS = 450;    // por debajo: nada dentro de ~80 cm
const float SHARP_MV_CERCA = 2750;   // por encima: < 10 cm (zona ambigua del sensor)
const uint32_t SHARP_TS_MS = 40;

// ---------------- Enlace ----------------
const uint32_t BAUDIOS = 115200, DIAG_MS = 1000, KEEPALIVE_MS = 3000;

// ============================================================
// ESTADO
// ============================================================
enum Modo { SIN_SENAL, FIJO, BARRIDO };
Modo modo = SIN_SENAL;
float amplitud = 60, velocidad = 150, anguloFijo = 0;
float thCmd = 0, vCmd = 0;           // consigna actual [deg, deg/s]
int sentido = 1;
bool servoConectado = false;

struct Consigna { uint32_t ms; float th; float v; };
const int N_HIST = 64;               // 64 x 20 ms = 1.28 s de historia
Consigna hist[N_HIST];
int iHist = 0, nHist = 0;

bool tfListo = false;
uint16_t tfTickAnterior = 0;
uint32_t i2cErr = 0, tfValidas = 0, tfValidasS = 0;
uint32_t seqT = 0, seqS = 0, seqD = 0;
uint32_t rxMalas = 0, txPerdidas = 0, lazoMaxUs = 0, ultimaLinea = 0;
bool keepaliveVencido = true;

// ============================================================
// SALIDA SERIAL NO BLOQUEANTE (igual que locomocion)
// ============================================================
uint8_t xorBytes(const char *s, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}
void enviarTrama(char *buf, int n, size_t cap) {
  if (n <= 0 || (size_t)n + 5 > cap) { txPerdidas++; return; }
  n += snprintf(buf + n, cap - n, "*%02X\n", xorBytes(buf, n));
  if (Serial.availableForWrite() < n) { txPerdidas++; return; }
  Serial.write((const uint8_t *)buf, n);
}
void mensaje(const char *texto) {
  if (Serial.availableForWrite() >= (int)strlen(texto) + 3) { Serial.print("# "); Serial.println(texto); }
}

// ============================================================
// SERVO
// ============================================================
uint32_t pulsoUs(float th) {
  float a = th * SERVO_SIGNO;
  float us = a >= 0 ? SERVO_US_CENTRO + (SERVO_US_MAS60 - SERVO_US_CENTRO) * a / 60
                    : SERVO_US_CENTRO + (SERVO_US_CENTRO - SERVO_US_MENOS60) * a / 60;
  return (uint32_t)lroundf(us);
}
// PWM del servo con MCPWM (1 tick = 1 us, periodo 20000 us). En v1.0 se
// usaba ledcWrite, que en este core bloqueaba ~37 ms por llamada y bajaba
// el TF-Luna a 25 Hz durante el barrido. El comparador de MCPWM solo
// escribe un registro sombra que se aplica al inicio del siguiente
// periodo, asi que no bloquea.
mcpwm_timer_handle_t pwmTimer = nullptr;
mcpwm_oper_handle_t pwmOper = nullptr;
mcpwm_cmpr_handle_t pwmCmp = nullptr;
mcpwm_gen_handle_t pwmGen = nullptr;
bool servoIniciar() {
  mcpwm_timer_config_t tc = {};
  tc.group_id = 0;
  tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tc.resolution_hz = 1000000;
  tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  tc.period_ticks = 20000;
  mcpwm_operator_config_t oc = {};
  oc.group_id = 0;
  mcpwm_comparator_config_t cc = {};
  cc.flags.update_cmp_on_tez = true;
  mcpwm_generator_config_t gc = {};
  gc.gen_gpio_num = SERVO;
  bool ok = mcpwm_new_timer(&tc, &pwmTimer) == ESP_OK &&
            mcpwm_new_operator(&oc, &pwmOper) == ESP_OK &&
            mcpwm_operator_connect_timer(pwmOper, pwmTimer) == ESP_OK &&
            mcpwm_new_comparator(pwmOper, &cc, &pwmCmp) == ESP_OK &&
            mcpwm_new_generator(pwmOper, &gc, &pwmGen) == ESP_OK &&
            mcpwm_comparator_set_compare_value(pwmCmp, 1500) == ESP_OK &&
            mcpwm_generator_set_action_on_timer_event(pwmGen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH)) == ESP_OK &&
            mcpwm_generator_set_action_on_compare_event(pwmGen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwmCmp,
                                               MCPWM_GEN_ACTION_LOW)) == ESP_OK &&
            mcpwm_generator_set_force_level(pwmGen, 0, true) == ESP_OK &&   // arranca en LOW
            mcpwm_timer_enable(pwmTimer) == ESP_OK &&
            mcpwm_timer_start_stop(pwmTimer, MCPWM_TIMER_START_NO_STOP) == ESP_OK;
  return ok;
}
void escribirServo(float th) {
  if (!pwmGen) return;
  mcpwm_comparator_set_compare_value(pwmCmp, pulsoUs(th));
  if (!servoConectado) { mcpwm_generator_set_force_level(pwmGen, -1, true); servoConectado = true; }
}
void soltarServo() {
  if (pwmGen) mcpwm_generator_set_force_level(pwmGen, 0, true);   // senal en LOW fijo
  servoConectado = false;
  modo = SIN_SENAL;
  vCmd = 0;
}
void guardarConsigna(uint32_t ms) {
  hist[iHist] = {ms, thCmd, vCmd};
  iHist = (iHist + 1) % N_HIST;
  if (nHist < N_HIST) nHist++;
}

// Perfil: velocidad constante con rampa de aceleracion al llegar a cada
// extremo; en FIJO va al angulo pedido con el mismo limite de velocidad.
void actualizarServo(float dt) {
  if (modo == SIN_SENAL) return;
  float destino = modo == BARRIDO ? sentido * amplitud : anguloFijo;
  float dist = destino - thCmd;
  float vMax = sqrtf(2 * ACEL_DPS2 * fabsf(dist));   // frenar a tiempo
  float vObj = (dist > 0 ? 1 : -1) * fminf(velocidad, vMax);
  if (fabsf(dist) < 0.05f) vObj = 0;
  float dv = ACEL_DPS2 * dt;
  vCmd = vObj > vCmd + dv ? vCmd + dv : (vObj < vCmd - dv ? vCmd - dv : vObj);
  thCmd += vCmd * dt;
  if ((dist > 0 && thCmd > destino) || (dist < 0 && thCmd < destino)) { thCmd = destino; vCmd = 0; }
  if (modo == BARRIDO && fabsf(destino - thCmd) < 0.05f) sentido = -sentido;
  thCmd = constrain(thCmd, -SERVO_LIMITE_DEG, SERVO_LIMITE_DEG);
  escribirServo(thCmd);
}

// Angulo real estimado en el instante t: consigna de (t - lag).
bool anguloEstimado(uint32_t t, float &th, bool &extremo) {
  if (nHist < 2) return false;
  auto en = [&](uint32_t tq, Consigna &out) -> bool {
    for (int k = 1; k < nHist; k++) {
      const Consigna &b = hist[(iHist - k + N_HIST) % N_HIST];
      const Consigna &a = hist[(iHist - k - 1 + N_HIST) % N_HIST];
      if ((int32_t)(tq - a.ms) >= 0 && (int32_t)(b.ms - tq) >= 0) {
        float u = b.ms == a.ms ? 0 : float(tq - a.ms) / float(b.ms - a.ms);
        out = {tq, a.th + u * (b.th - a.th), a.v + u * (b.v - a.v)};
        return true;
      }
    }
    const Consigna &ult = hist[(iHist - 1 + N_HIST) % N_HIST];
    if ((int32_t)(tq - ult.ms) >= 0) { out = ult; return true; }   // posterior a la ultima
    return false;
  };
  Consigna c;
  if (!en(t - (uint32_t)lagCcwMs, c)) return false;
  if (c.v < 0 && !en(t - (uint32_t)lagCwMs, c)) return false;
  th = c.th;
  extremo = modo == BARRIDO && fabsf(c.v) < 0.5f * velocidad;
  return true;
}

// ============================================================
// TF-LUNA (I2C, modo continuo)
// ============================================================
bool tfLeer(uint8_t reg, uint8_t *p, uint8_t n) {
  Wire.beginTransmission(TF_DIR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(TF_DIR, n) != n) {
    while (Wire.available()) Wire.read();
    i2cErr++;
    return false;
  }
  for (uint8_t i = 0; i < n; i++) p[i] = Wire.read();
  return true;
}
bool tfEscribir(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(TF_DIR);
  Wire.write(reg);
  Wire.write(v);
  if (Wire.endTransmission() == 0) return true;
  i2cErr++;
  return false;
}
bool tfEsperarDato(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) if (digitalRead(TF_DR)) return true;
  return false;
}
void tfIniciar() {
  uint8_t basura[10];
  delay(300);
  bool ok = tfEscribir(0x23, 0x00);                          // modo continuo
  ok = tfEscribir(0x26, TF_FPS & 0xFF) && ok;                // 100 Hz
  ok = tfEscribir(0x27, TF_FPS >> 8) && ok;
  ok = tfEscribir(0x28, 0x00) && ok;                         // potencia normal
  delay(50);
  tfLeer(0, basura, 10);
  // El manual A03 y las librerias no coinciden en el valor de ENABLE (0x25).
  // Si no llegan datos, se prueban ambos.
  if (!tfEsperarDato(200)) { tfEscribir(0x25, 0x01); delay(50); tfLeer(0, basura, 10); }
  if (!tfEsperarDato(200)) { tfEscribir(0x25, 0x00); delay(50); tfLeer(0, basura, 10); }
  tfListo = ok && tfEsperarDato(200);
  mensaje(tfListo ? "TF-Luna listo (continuo 100 Hz)" : "TF-Luna SIN DATOS: revisar cableado/pin 6");
}
void servirTF() {
  if (!digitalRead(TF_DR)) return;
  uint32_t tMedida = millis() - TF_RETARDO_US / 1000;
  uint8_t d[10];
  if (!tfLeer(0, d, 10)) return;        // la lectura baja el pin 6
  uint16_t tick = d[6] | (d[7] << 8);
  if (tick == tfTickAnterior) return;   // trama repetida
  tfTickAnterior = tick;
  float crudo = 10.0f * (d[0] | (d[1] << 8));
  uint16_t amp = d[2] | (d[3] << 8), err = d[8] | (d[9] << 8);
  float mm = crudo;
  if (mm >= TF_CAL_MIN && mm <= TF_CAL_MAX) mm = TF_ESCALA * mm + TF_OFFSET_MM;
  bool valida = err == 0 && amp >= 100 && amp < 32768 && mm >= TF_MIN_MM && mm <= TF_MAX_MM;
  float th = 0;
  bool extremo = false;
  bool conAngulo = modo != SIN_SENAL && anguloEstimado(tMedida, th, extremo);
  uint8_t flags = (valida ? 0 : 1) | (extremo ? 2 : 0) | (modo == BARRIDO ? 4 : 0) |
                  (conAngulo ? 0 : 8);
  if (valida) tfValidas++;
  char buf[80];
  int n = snprintf(buf, sizeof(buf), "T,%lu,%lu,%ld,%ld,%u,%u", (unsigned long)seqT++,
                   (unsigned long)tMedida, lroundf(th * 100), lroundf(mm), amp, flags);
  enviarTrama(buf, n, sizeof(buf));
}

// ============================================================
// SHARP + IR TRASERO
// ============================================================
float sharpMv(int pin) {
  analogReadMilliVolts(pin);           // descarta la primera tras cambiar de canal
  uint32_t v[5];
  for (int i = 0; i < 5; i++) v[i] = analogReadMilliVolts(pin);
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 5; j++)
      if (v[i] > v[j]) { uint32_t t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[2];
}
long sharpMm(int lado, float mv) {
  if (mv < SHARP_MV_LEJOS) return 0;
  if (mv > SHARP_MV_CERCA) return 100;
  float mm;
  int n = SHARP_N[lado];
  if (n >= 3) {
    const PuntoCal *t = SHARP_TABLA[lado];
    if (mv <= t[0].mv) return 0;
    mm = t[n - 1].mm;
    for (int i = 1; i < n; i++)
      if (mv <= t[i].mv) {
        mm = t[i - 1].mm + (mv - t[i - 1].mv) / (t[i].mv - t[i - 1].mv) * (t[i].mm - t[i - 1].mm);
        break;
      }
  } else {
    mm = 299.88f * powf(mv / 1000.0f, -1.173f);
  }
  if (mm > 800) return 0;
  return lroundf(fmaxf(mm, 100));
}
void servirSharp() {
  float mv[2] = {sharpMv(SHARP_PIN[0]), sharpMv(SHARP_PIN[1])};
  int irTras = digitalRead(IR_TRASERO) == LOW ? 1 : 0;
  char buf[80];
  int n = snprintf(buf, sizeof(buf), "S,%lu,%lu,%ld,%ld,%ld,%ld,%d", (unsigned long)seqS++,
                   (unsigned long)millis(), sharpMm(0, mv[0]), sharpMm(1, mv[1]),
                   lroundf(mv[0]), lroundf(mv[1]), irTras);
  enviarTrama(buf, n, sizeof(buf));
}

// ============================================================
// COMANDOS
// ============================================================
void identificar() {
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "I,PERC,%s", VERSION);
  enviarTrama(buf, n, sizeof(buf));
}
void procesarLinea(char *linea) {
  char *ast = strchr(linea, '*');
  if (ast) {
    if (xorBytes(linea, ast - linea) != strtoul(ast + 1, nullptr, 16)) { rxMalas++; return; }
    *ast = '\0';
  }
  float a = 0, b = 0;
  int n = sscanf(linea + 1, ",%f,%f", &a, &b);
  bool ok = true;
  switch (linea[0]) {
    case 'B':
      if (n == 2 && a >= 0 && b > 0) {
        amplitud = fminf(a, SERVO_LIMITE_DEG);
        velocidad = fminf(b, 400);
        if (amplitud < 0.5f) { anguloFijo = 0; modo = FIJO; }
        else modo = BARRIDO;
      } else ok = false;
      break;
    case 'F':
      if (n >= 1) { anguloFijo = constrain(a, -SERVO_LIMITE_DEG, SERVO_LIMITE_DEG); modo = FIJO; }
      else ok = false;
      break;
    case 'R': soltarServo(); break;
    case 'Z': if (n == 2 && a >= 0 && b >= 0) { lagCcwMs = a; lagCwMs = b; } else ok = false; break;
    case 'K': break;                     // keepalive explicito
    case '?': identificar(); break;
    default: ok = false;
  }
  if (!ok) { rxMalas++; return; }
  ultimaLinea = millis();
  keepaliveVencido = false;
}
char bufRx[96];
uint8_t nRx = 0;
bool descartar = false;
void leerSerial() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (!descartar && nRx > 0) { bufRx[nRx] = '\0'; procesarLinea(bufRx); }
      nRx = 0;
      descartar = false;
    } else if (!descartar) {
      if (nRx < sizeof(bufRx) - 1) bufRx[nRx++] = ch;
      else { rxMalas++; descartar = true; }
    }
  }
}

void enviarDiagnostico() {
  char buf[112];
  int n = snprintf(buf, sizeof(buf), "D,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d", (unsigned long)seqD++,
                   (unsigned long)millis(), (unsigned long)lazoMaxUs, (unsigned long)rxMalas,
                   (unsigned long)txPerdidas, (unsigned long)i2cErr, (unsigned long)tfValidasS,
                   (int)modo);
  enviarTrama(buf, n, sizeof(buf));
  lazoMaxUs = 0;
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  setCpuFrequencyMhz(80);
  Serial.setRxBufferSize(512);
  Serial.setTxBufferSize(2048);
  Serial.begin(BAUDIOS);
  pinMode(LED_ESTADO, OUTPUT);
  pinMode(IR_TRASERO, INPUT_PULLUP);
  pinMode(TF_DR, INPUT);
  bool pwmOk = servoIniciar();
  soltarServo();                        // sin movimiento hasta que la Pi lo pida
  analogReadResolution(12);
  for (int i = 0; i < 2; i++) analogSetPinAttenuation(SHARP_PIN[i], ADC_11db);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(10);
  identificar();
  if (!pwmOk) mensaje("MCPWM del servo NO inicio");
  tfIniciar();
}

void loop() {
  static uint32_t tServo = millis(), tSharp = millis(), tDiag = millis();
  uint32_t t0 = micros();
  leerSerial();
  uint32_t ahora = millis();
  if (!keepaliveVencido && ahora - ultimaLinea > KEEPALIVE_MS) {
    keepaliveVencido = true;
    if (modo == BARRIDO) { anguloFijo = 0; modo = FIJO; mensaje("Sin enlace: cabezal al centro"); }
  }
  if (ahora - tServo >= SERVO_TS_MS) {
    float dt = (ahora - tServo) / 1000.0f;
    tServo = ahora;
    actualizarServo(dt);
    guardarConsigna(ahora);
  }
  servirTF();
  if (ahora - tSharp >= SHARP_TS_MS) { tSharp = ahora; servirSharp(); }
  if (ahora - tDiag >= DIAG_MS) {
    tDiag = ahora;
    tfValidasS = tfValidas;
    tfValidas = 0;
    enviarDiagnostico();
  }
  digitalWrite(LED_ESTADO, modo == BARRIDO ? HIGH : (keepaliveVencido && ((ahora / 250) & 1)));
  uint32_t dur = micros() - t0;
  if (dur > lazoMaxUs) lazoMaxUs = dur;
  delay(1);
}
