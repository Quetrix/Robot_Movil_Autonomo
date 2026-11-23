// ============================================================
// MT9005 - ROBOT MECANUM - MODULO DE LOCOMOCION (ESP32, entrega final)
// Esclavo de la Raspberry Pi 5 por USB. Sin WiFi ni Bluetooth.
//
// Version derivada de _respaldo/locomocion_esp32_ir_frontal (Tarea 8).
// Se conservan sin cambios: pinout (verificado 9 nov 2026), signos,
// CPR, geometria, ganancias PI, feedforward, rampas y deteccion de stall.
// Se eliminaron: secuencia del Avance 1, giro 360, Tarea 5, comandos
// W/S/A/D/R/H, parpadeos de LED y textos de ayuda.
//
// Marco robot (igual que el enunciado y Documentos/Geometria_robot.md):
//   X adelante, Y izquierda, giro antihorario positivo.
// Ruedas: 0 = FL, 1 = FR, 2 = RL, 3 = RR.
//
// Protocolo (230400 baud, lineas ASCII terminadas en \n). Detalle en
// Proyecto_Final/README.md. Sufijo opcional *HH = XOR de los bytes
// antes de '*', en hexadecimal; si viene, se verifica.
//   Pi -> ESP32
//     V,vx,vy,wz      velocidades enteras: mm/s, mm/s, grados/s
//     X               parada inmediata (sigue armado)
//     A,1 | A,0       armar / desarmar (para pruebas; en la prueba lo hace el boton)
//     P,p0,p1,p2,p3   PWM en lazo abierto -255..255 (calibracion de feedforward)
//     Q,1 | Q,0       habilitar / deshabilitar el IR frontal
//     ?               identificacion
//   ESP32 -> Pi
//     I,LOCO,version
//     L,seq,ms,c0,c1,c2,c3,r0,r1,r2,r3,p0,p1,p2,p3,flags   (50 Hz)
//        c = conteos x4 acumulados (+ = rueda hacia adelante)
//        r = referencia RPM, p = PWM aplicado con signo
//     D,seq,ms,lazo_max_us,rx_malas,tx_perdidas,watchdogs,stalls   (1 Hz)
//     # texto        mensajes para humanos (se registran, no se interpretan)
// flags: bit0 armado, bit1 IR frontal con obstaculo, bit2 parado por
//        watchdog, bit3 stall, bit4 frenando, bit5 lazo abierto,
//        bit6 IR frontal deshabilitado
//
// Boton (GPIO12 a GND): cada pulsacion alterna armado/desarmado.
// Desarmar = paro de emergencia: motores en cero al instante.
// LED (GPIO2): encendido = armado; parpadeo rapido = stall.
// ============================================================
#include <Arduino.h>
#include <math.h>
#include "soc/gpio_reg.h"

const char VERSION[] = "loco-final-1.0";

// ============================================================
// HARDWARE (cableado actual, verificado 9 nov 2026)
// ============================================================
const int EN[4]    = {13, 23, 17, 19};
const int IN1[4]   = {14, 22, 16, 18};
const int IN2[4]   = {27, 21, 4, 5};
const int ENC_A[4] = {15, 32, 25, 36};
const int ENC_B[4] = {35, 33, 26, 39};
const int LED_ESTADO = 2;
const int BOTON      = 12;   // INPUT_PULLUP, boton a GND
const int IR_FRENTE  = 34;   // KY-032 OUT, LOW = obstaculo, sin pull-up interno

const int MOTOR_SIGN[4]   = { 1, 1, -1, 1};
const int ENCODER_SIGN[4] = {-1, 1, -1, 1};
const float CPR[4] = {903.50f, 939.93f, 903.21f, 905.21f};   // conteos x4 por vuelta

// ============================================================
// GEOMETRIA Y LIMITES
// ============================================================
const float RADIO  = 0.024f;          // [m] radio usado en la calibracion del Avance 1
const float L_GEOM = 0.08f + 0.07f;   // a + b [m]
const float RPM_MAX = 120.0f;         // tope de rueda (limite de la tabla de feedforward)

// ============================================================
// TEMPORIZACION Y ENLACE
// ============================================================
const uint32_t BAUDIOS       = 230400;
const uint32_t TS_MS         = 20;     // lazo de control y telemetria (50 Hz)
const uint32_t DIAG_MS       = 1000;
const uint32_t WATCHDOG_MS   = 500;    // sin V/P en este tiempo -> frena con rampa
const uint32_t BOTON_DEBOUNCE_MS = 80;
const uint32_t IR_DEBOUNCE_MS = 20;    // LOW estable para confirmar obstaculo
const uint32_t IR_LIBRE_MS    = 60;    // HIGH estable para liberar

// ============================================================
// CONTROL PI + FEEDFORWARD (valores del Avance 1 / Tarea 8, sin cambios)
// ============================================================
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8;
const float KP[4] = {0.50f, 0.50f, 0.50f, 0.50f};
const float KI[4] = {0.45f, 0.45f, 0.45f, 0.45f};
const float I_TERM_MAX = 30.0f;
const uint32_t I_DELAY_MS = 300;
const float ALPHA_RPM = 0.30f;
const float PWM_SLEW_NORMAL = 12.0f;   // PWM por ciclo
const float PWM_SLEW_FRENO  = 35.0f;
const float ACEL_RPM_S  = 650.0f;      // rampa de la referencia (igual al frenado original)
const float RPM_REPOSO  = 3.0f;
// Feedforward medido con los motores a 5 V (PowerBoost). Con 2x18650 hay que
// volver a medirlo: Proyecto_Final/calibrar.py ff. FF_ESCALA permite un ajuste
// global mientras tanto (1.0 = tabla original).
const float FF_ESCALA = 1.0f;
const int N_FF_FWD = 3;
const float RPM_FF_FWD[N_FF_FWD] = {80.0f, 100.0f, 120.0f};
const float PWM_FF_FWD[4][N_FF_FWD] = {
  {216.5f, 230.7f, 241.4f}, {220.2f, 233.7f, 245.3f},
  {217.4f, 231.1f, 243.4f}, {218.3f, 231.1f, 242.6f}};
const int N_REV = 5;
const float PWM_REV[N_REV] = {195, 210, 225, 240, 255};
const float RPM_REV[4][N_REV] = {
  {70.34f, 79.30f, 104.26f, 132.82f, 158.99f}, {62.77f, 79.16f, 104.64f, 133.63f, 159.11f},
  {75.79f, 82.54f, 105.85f, 134.24f, 159.60f}, {74.35f, 84.51f, 109.53f, 138.97f, 164.60f}};

// ============================================================
// ESTADO
// ============================================================
volatile int32_t conteos[4] = {0, 0, 0, 0};
volatile uint8_t estadoAnterior[4] = {0, 0, 0, 0};
const int8_t TABLA[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
portMUX_TYPE muxEncoder = portMUX_INITIALIZER_UNLOCKED;

int32_t conteoAnterior[4] = {0, 0, 0, 0};
float rpmFiltrada[4] = {0, 0, 0, 0};
float referenciaRPM[4] = {0, 0, 0, 0};   // referencia que sigue el PI (con rampa)
float objetivoRPM[4] = {0, 0, 0, 0};     // referencia pedida por el comando
float iTerm[4] = {0, 0, 0, 0};
float pwmAplicado[4] = {0, 0, 0, 0};
float pwmLazoAbierto[4] = {0, 0, 0, 0};

bool armado = false, controlActivo = false, lazoAbierto = false;
bool paroWatchdog = false, stall = false, irHabilitado = true;
float cmdVx = 0, cmdVy = 0, cmdWz = 0;   // ultimo comando V [m/s, m/s, rad/s]
uint32_t ultimoComando = 0, inicioControl = 0;
int contadorStall = 0;

// IR frontal
int lecturaIR = HIGH;
uint32_t cambioIR = 0;
bool irObstaculo = false;

// Boton
int lecturaBoton = HIGH, estadoBoton = HIGH;
uint32_t cambioBoton = 0;

// Diagnostico
uint32_t seqL = 0, seqD = 0, lazoMaxUs = 0, rxMalas = 0, txPerdidas = 0;
uint32_t nWatchdog = 0, nStall = 0;

// ============================================================
// ENCODERS (lectura directa de registros: rapida y valida en IRAM)
// ============================================================
static inline uint32_t IRAM_ATTR leerPin(int pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1U
                  : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1U;
}
void IRAM_ATTR actualizarEncoder(int i) {
  uint8_t nuevo = (leerPin(ENC_A[i]) << 1) | leerPin(ENC_B[i]);
  portENTER_CRITICAL_ISR(&muxEncoder);
  conteos[i] += TABLA[(estadoAnterior[i] << 2) | nuevo];
  estadoAnterior[i] = nuevo;
  portEXIT_CRITICAL_ISR(&muxEncoder);
}
void IRAM_ATTR ISR_FL() { actualizarEncoder(0); }
void IRAM_ATTR ISR_FR() { actualizarEncoder(1); }
void IRAM_ATTR ISR_RL() { actualizarEncoder(2); }
void IRAM_ATTR ISR_RR() { actualizarEncoder(3); }

void copiarConteos(int32_t destino[4]) {
  portENTER_CRITICAL(&muxEncoder);
  for (int i = 0; i < 4; i++) destino[i] = conteos[i];
  portEXIT_CRITICAL(&muxEncoder);
}

// ============================================================
// SALIDA SERIAL NO BLOQUEANTE
// Si el buffer de TX no tiene espacio se descarta la linea completa y se
// cuenta en tx_perdidas: el lazo de control nunca espera al USB.
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
  int n = strlen(texto);
  if (Serial.availableForWrite() >= n + 3) { Serial.print("# "); Serial.println(texto); }
}

// ============================================================
// MOTORES
// ============================================================
float limitar(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float acercar(float actual, float objetivo, float paso) {
  if (objetivo > actual + paso) return actual + paso;
  if (objetivo < actual - paso) return actual - paso;
  return objetivo;
}
void aplicarMotor(int i, float comando) {
  comando = limitar(comando, -255.0f, 255.0f);
  if (fabsf(comando) < 0.5f) {
    ledcWrite(EN[i], 0);
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
    return;
  }
  bool positivo = (comando > 0 ? 1 : -1) * MOTOR_SIGN[i] > 0;
  digitalWrite(IN1[i], positivo ? HIGH : LOW);
  digitalWrite(IN2[i], positivo ? LOW : HIGH);
  ledcWrite(EN[i], (uint32_t)lroundf(fabsf(comando)));
}
void pararMotores() {
  controlActivo = false;
  lazoAbierto = false;
  cmdVx = cmdVy = cmdWz = 0;
  for (int i = 0; i < 4; i++) {
    referenciaRPM[i] = objetivoRPM[i] = iTerm[i] = 0;
    pwmAplicado[i] = pwmLazoAbierto[i] = 0;
    aplicarMotor(i, 0);
  }
}

// ============================================================
// CINEMATICA INVERSA MECANUM (X adelante, Y izquierda, wz antihorario)
// ============================================================
void cinematica(float vx, float vy, float wz, float salida[4]) {
  float w[4] = {(vx - vy - L_GEOM * wz) / RADIO, (vx + vy + L_GEOM * wz) / RADIO,
                (vx + vy - L_GEOM * wz) / RADIO, (vx - vy + L_GEOM * wz) / RADIO};
  float maximo = 0;
  for (int i = 0; i < 4; i++) {
    salida[i] = w[i] * 60.0f / (2.0f * PI);
    maximo = fmaxf(maximo, fabsf(salida[i]));
  }
  if (maximo > RPM_MAX)   // escala uniforme: conserva la direccion del movimiento
    for (int i = 0; i < 4; i++) salida[i] *= RPM_MAX / maximo;
}

// ============================================================
// FEEDFORWARD (por debajo del primer punto: extrapolacion lineal a cero,
// como en el modo continuo de la Tarea 8)
// ============================================================
float feedForward(int i, float ref) {
  float r = fabsf(ref);
  if (r < 1.0f) return 0;
  float pwm;
  if (ref > 0) {
    if (r <= RPM_FF_FWD[0]) pwm = PWM_FF_FWD[i][0] * r / RPM_FF_FWD[0];
    else {
      pwm = PWM_FF_FWD[i][N_FF_FWD - 1];
      for (int k = 0; k < N_FF_FWD - 1; k++)
        if (r <= RPM_FF_FWD[k + 1]) {
          float f = (r - RPM_FF_FWD[k]) / (RPM_FF_FWD[k + 1] - RPM_FF_FWD[k]);
          pwm = PWM_FF_FWD[i][k] + f * (PWM_FF_FWD[i][k + 1] - PWM_FF_FWD[i][k]);
          break;
        }
    }
  } else {
    if (r <= RPM_REV[i][0]) pwm = PWM_REV[0] * r / RPM_REV[i][0];
    else {
      pwm = 255.0f;
      for (int k = 0; k < N_REV - 1; k++)
        if (r <= RPM_REV[i][k + 1]) {
          float f = (r - RPM_REV[i][k]) / (RPM_REV[i][k + 1] - RPM_REV[i][k]);
          pwm = PWM_REV[k] + f * (PWM_REV[k + 1] - PWM_REV[k]);
          break;
        }
    }
    pwm = -pwm;
  }
  return pwm * FF_ESCALA;
}

// ============================================================
// LAZO DE CONTROL (cada TS_MS)
// ============================================================
void medir(float dt) {
  int32_t actual[4];
  copiarConteos(actual);
  for (int i = 0; i < 4; i++) {
    int32_t delta = (actual[i] - conteoAnterior[i]) * ENCODER_SIGN[i];
    conteoAnterior[i] = actual[i];
    float rpm = delta * 60.0f / (CPR[i] * dt);
    rpmFiltrada[i] = ALPHA_RPM * rpm + (1.0f - ALPHA_RPM) * rpmFiltrada[i];
  }
}

void actualizarObjetivo() {
  // El IR frontal anula solo la componente hacia adelante (vx > 0).
  float vx = (irObstaculo && irHabilitado && cmdVx > 0) ? 0.0f : cmdVx;
  cinematica(vx, cmdVy, cmdWz, objetivoRPM);
}

void controlPI(float dt) {
  bool frenando = true;
  for (int i = 0; i < 4; i++) if (fabsf(objetivoRPM[i]) >= 1.0f) frenando = false;
  float paso = ACEL_RPM_S * dt;
  for (int i = 0; i < 4; i++) {
    float anterior = referenciaRPM[i];
    referenciaRPM[i] = acercar(referenciaRPM[i], objetivoRPM[i], paso);
    if ((anterior > 1.0f && referenciaRPM[i] < -1.0f) || (anterior < -1.0f && referenciaRPM[i] > 1.0f))
      iTerm[i] = 0;   // cambio de sentido: sin arrastre del integrador
    float ref = referenciaRPM[i];
    float slew = frenando ? PWM_SLEW_FRENO : PWM_SLEW_NORMAL;
    if (frenando) iTerm[i] = 0;   // al frenar, sin memoria integral (igual que el original)
    if (fabsf(ref) < 1.0f) {
      iTerm[i] = 0;
      pwmAplicado[i] = acercar(pwmAplicado[i], 0.0f, slew);
      aplicarMotor(i, pwmAplicado[i]);
      continue;
    }
    float error = ref - rpmFiltrada[i];
    float ff = feedForward(i, ref);
    float p = KP[i] * error;
    float lo = ref > 0 ? 0.0f : -255.0f, hi = ref > 0 ? 255.0f : 0.0f;
    if (!frenando && millis() - inicioControl >= I_DELAY_MS) {
      float nuevoI = limitar(iTerm[i] + KI[i] * error * dt, -I_TERM_MAX, I_TERM_MAX);
      float u = ff + p + nuevoI;   // anti-windup: integra solo si no empuja la saturacion
      if ((u >= lo && u <= hi) || (u > hi && error < 0) || (u < lo && error > 0)) iTerm[i] = nuevoI;
    }
    float objetivo = limitar(ff + p + iTerm[i], lo, hi);
    pwmAplicado[i] = acercar(pwmAplicado[i], objetivo, slew);
    aplicarMotor(i, pwmAplicado[i]);
  }
  // Fin del frenado: referencias en cero y ruedas casi quietas.
  if (frenando) {
    bool quieto = true;
    for (int i = 0; i < 4; i++)
      if (fabsf(referenciaRPM[i]) > 0.5f || fabsf(rpmFiltrada[i]) > RPM_REPOSO) quieto = false;
    if (quieto) pararMotores();
  }
}

void revisarStall() {
  if (millis() - inicioControl < 1500) return;
  float rpm = 0, pwm = 0;
  for (int i = 0; i < 4; i++) { rpm += fabsf(rpmFiltrada[i]); pwm += fabsf(pwmAplicado[i]); }
  contadorStall = (rpm / 4 < 10.0f && pwm / 4 > 245.0f) ? contadorStall + 1 : 0;
  if (contadorStall >= 20) {
    contadorStall = 0;
    stall = true;
    nStall++;
    pararMotores();
    mensaje("STALL: motores detenidos");
  }
}

// ============================================================
// COMANDOS
// ============================================================
void desarmar(const char *motivo) {
  armado = false;
  pararMotores();
  mensaje(motivo);
}

void comandoVelocidad(long vx, long vy, long wz) {
  if (!armado) return;
  ultimoComando = millis();
  paroWatchdog = false;
  lazoAbierto = false;
  cmdVx = vx / 1000.0f;
  cmdVy = vy / 1000.0f;
  cmdWz = wz * (PI / 180.0f);
  bool quieto = vx == 0 && vy == 0 && wz == 0;
  if (!quieto) {
    stall = false;
    if (!controlActivo) {
      controlActivo = true;
      inicioControl = millis();
      for (int i = 0; i < 4; i++) iTerm[i] = 0;
    }
  }
  actualizarObjetivo();   // con quieto=true baja con rampa hasta parar
}

void comandoLazoAbierto(long p[4]) {
  if (!armado) return;
  ultimoComando = millis();
  paroWatchdog = false;
  lazoAbierto = true;
  controlActivo = false;
  for (int i = 0; i < 4; i++) pwmLazoAbierto[i] = limitar((float)p[i], -255, 255);
}

void identificar() {
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "I,LOCO,%s", VERSION);
  enviarTrama(buf, n, sizeof(buf));
}

void procesarLinea(char *linea) {
  // Checksum opcional: si viene, debe coincidir.
  char *ast = strchr(linea, '*');
  if (ast) {
    unsigned long hh = strtoul(ast + 1, nullptr, 16);
    if (xorBytes(linea, ast - linea) != hh) { rxMalas++; return; }
    *ast = '\0';
  }
  long a[4] = {0, 0, 0, 0};
  char c = linea[0];
  int n = sscanf(linea + 1, ",%ld,%ld,%ld,%ld", &a[0], &a[1], &a[2], &a[3]);
  switch (c) {
    case 'V': if (n == 3) { comandoVelocidad(a[0], a[1], a[2]); return; } break;
    case 'X': if (linea[1] == '\0') { pararMotores(); return; } break;
    case 'A':
      if (n == 1) {
        if (a[0]) { armado = true; stall = false; mensaje("ARMADO por serial"); }
        else desarmar("DESARMADO por serial");
        return;
      }
      break;
    case 'P': if (n == 4) { comandoLazoAbierto(a); return; } break;
    case 'Q': if (n == 1) { irHabilitado = a[0] != 0; return; } break;
    case '?': identificar(); return;
  }
  rxMalas++;
}

char bufRx[96];
uint8_t nRx = 0;
bool descartarLinea = false;
void leerSerial() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (!descartarLinea && nRx > 0) { bufRx[nRx] = '\0'; procesarLinea(bufRx); }
      nRx = 0;
      descartarLinea = false;
    } else if (!descartarLinea) {
      if (nRx < sizeof(bufRx) - 1) bufRx[nRx++] = ch;
      else { rxMalas++; descartarLinea = true; }
    }
  }
}

// ============================================================
// ENTRADAS: BOTON E IR FRONTAL (sondeo con debounce; el lazo corre a ~1 kHz)
// ============================================================
void actualizarBoton() {
  int l = digitalRead(BOTON);
  if (l != lecturaBoton) { lecturaBoton = l; cambioBoton = millis(); }
  if (millis() - cambioBoton >= BOTON_DEBOUNCE_MS && l != estadoBoton) {
    estadoBoton = l;
    if (estadoBoton == LOW) {
      if (armado) desarmar("BOTON: paro (desarmado)");
      else { armado = true; stall = false; mensaje("BOTON: armado"); }
    }
  }
}

void actualizarIR() {
  int l = digitalRead(IR_FRENTE);
  uint32_t ahora = millis();
  if (l != lecturaIR) { lecturaIR = l; cambioIR = ahora; }
  uint32_t estable = ahora - cambioIR;
  bool antes = irObstaculo;
  if (!irObstaculo && l == LOW && estable >= IR_DEBOUNCE_MS) irObstaculo = true;
  else if (irObstaculo && l == HIGH && estable >= IR_LIBRE_MS) irObstaculo = false;
  if (antes != irObstaculo && controlActivo) actualizarObjetivo();   // reaccion inmediata
}

void actualizarLED() {
  bool on = stall ? ((millis() / 125) & 1) : armado;
  digitalWrite(LED_ESTADO, on ? HIGH : LOW);
}

// ============================================================
// TELEMETRIA
// ============================================================
void enviarTelemetria() {
  int32_t c[4];
  copiarConteos(c);
  uint8_t flags = (armado ? 1 : 0) | (irObstaculo ? 2 : 0) | (paroWatchdog ? 4 : 0) |
                  (stall ? 8 : 0) | (lazoAbierto ? 32 : 0) | (irHabilitado ? 0 : 64);
  bool frenando = controlActivo;
  for (int i = 0; i < 4; i++) if (fabsf(objetivoRPM[i]) >= 1.0f) frenando = false;
  if (frenando) flags |= 16;
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "L,%lu,%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%u",
                   (unsigned long)seqL++, (unsigned long)millis(),
                   (long)(c[0] * ENCODER_SIGN[0]), (long)(c[1] * ENCODER_SIGN[1]),
                   (long)(c[2] * ENCODER_SIGN[2]), (long)(c[3] * ENCODER_SIGN[3]),
                   (int)lroundf(referenciaRPM[0]), (int)lroundf(referenciaRPM[1]),
                   (int)lroundf(referenciaRPM[2]), (int)lroundf(referenciaRPM[3]),
                   (int)lroundf(pwmAplicado[0]), (int)lroundf(pwmAplicado[1]),
                   (int)lroundf(pwmAplicado[2]), (int)lroundf(pwmAplicado[3]), flags);
  enviarTrama(buf, n, sizeof(buf));
}

void enviarDiagnostico() {
  char buf[96];
  int n = snprintf(buf, sizeof(buf), "D,%lu,%lu,%lu,%lu,%lu,%lu,%lu", (unsigned long)seqD++,
                   (unsigned long)millis(), (unsigned long)lazoMaxUs, (unsigned long)rxMalas,
                   (unsigned long)txPerdidas, (unsigned long)nWatchdog, (unsigned long)nStall);
  enviarTrama(buf, n, sizeof(buf));
  lazoMaxUs = 0;
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  setCpuFrequencyMhz(80);          // suficiente para ~7 k interrupciones/s; menos consumo
  Serial.setRxBufferSize(512);
  Serial.setTxBufferSize(1024);
  Serial.begin(BAUDIOS);
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    ledcAttach(EN[i], PWM_FREQ, PWM_RESOLUTION);
    aplicarMotor(i, 0);
    pinMode(ENC_A[i], INPUT);
    pinMode(ENC_B[i], INPUT);
    estadoAnterior[i] = (leerPin(ENC_A[i]) << 1) | leerPin(ENC_B[i]);
  }
  pinMode(LED_ESTADO, OUTPUT);
  pinMode(BOTON, INPUT_PULLUP);
  pinMode(IR_FRENTE, INPUT);
  void (*isr[4])() = {ISR_FL, ISR_FR, ISR_RL, ISR_RR};
  for (int i = 0; i < 4; i++) {
    attachInterrupt(digitalPinToInterrupt(ENC_A[i]), isr[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B[i]), isr[i], CHANGE);
  }
  copiarConteos(conteoAnterior);
  lecturaIR = digitalRead(IR_FRENTE);
  irObstaculo = lecturaIR == LOW;
  cambioIR = cambioBoton = millis();
  pararMotores();
  identificar();
}

void loop() {
  static uint32_t ultimoControl = millis(), ultimoDiag = millis();
  uint32_t t0 = micros();
  leerSerial();
  actualizarBoton();
  actualizarIR();
  actualizarLED();

  uint32_t ahora = millis();
  if (ahora - ultimoControl >= TS_MS) {
    float dt = (ahora - ultimoControl) / 1000.0f;
    ultimoControl = ahora;
    medir(dt);
    bool moviendo = controlActivo || lazoAbierto;
    if (moviendo && ahora - ultimoComando > WATCHDOG_MS) {
      if (!paroWatchdog) { paroWatchdog = true; nWatchdog++; }
      if (lazoAbierto) pararMotores();
      else { cmdVx = cmdVy = cmdWz = 0; actualizarObjetivo(); }
    }
    if (lazoAbierto) for (int i = 0; i < 4; i++) {
      pwmAplicado[i] = pwmLazoAbierto[i];
      aplicarMotor(i, pwmAplicado[i]);
    } else if (controlActivo) {
      controlPI(dt);
      if (controlActivo) revisarStall();
    }
    enviarTelemetria();
  }
  if (ahora - ultimoDiag >= DIAG_MS) {
    ultimoDiag = ahora;
    enviarDiagnostico();
  }
  uint32_t dur = micros() - t0;
  if (dur > lazoMaxUs) lazoMaxUs = dur;
  delay(1);   // cede la CPU a la tarea idle (menos consumo); el lazo sigue a ~1 kHz
}
