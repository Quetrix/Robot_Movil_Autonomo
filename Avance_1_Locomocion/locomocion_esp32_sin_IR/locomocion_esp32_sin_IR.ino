#include <Arduino.h>
#include <math.h>
// ============================================================
// MT9005 - ROBOT MECANUM ESP32 - MODULO DE LOCOMOCION
// VARIANTE SIN IR DISCRETOS + MODOS DE PRUEBA (continuo,
// temporizado y mantener tecla)
//
// Basado en locomocion_esp32.ino (reconstruccion del Avance 1).
// Cambios: sin KY-032 (GPIO0 y GPIO34 libres), canales de RL/RR
// intercambiados segun el cableado actual, signo de motor FL
// corregido, comandos temporizados y comando H con timeout.
//
// Orden ruedas: 0 = FL, 1 = FR, 2 = RL, 3 = RR
// Boton: GPIO12 -> boton -> GND (INPUT_PULLUP)
// LED integrado: GPIO2 (encendido = detenido, apagado = en marcha)
// Encoder FL canal A: GPIO15
//
// Comandos (115200 baud, cada linea termina en \n):
//   W S A D [ms]           80 RPM; con ms se detiene solo
//   R                      giro horario de 360 grados
//   X                      parada inmediata
//   V vx vy omega [ms]     continuo, o temporizado con ms
//   H vx vy omega          mantener: frena si no se repite en 300 ms
//   vx derecha+ [m/s], vy adelante+ [m/s], omega antihorario+ [rad/s]
// ============================================================
// ============================================================
// HARDWARE (cableado actual, verificado 9 nov 2026)
// ============================================================
const int EN[4]  = {13, 23, 17, 19};
const int IN1[4] = {14, 22, 16, 18};
const int IN2[4] = {27, 21, 4, 5};
const int ENC_A[4] = {15, 32, 25, 36};
const int ENC_B[4] = {35, 33, 26, 39};
// Boton y LED de estado
const int LED_ESTADO    = 2;
const int BOTON         = 12;
// ============================================================
// SIGNOS EXPERIMENTALES DEFINITIVOS
// ============================================================
const int MOTOR_SIGN[4] = {
   1,  // FL
   1,  // FR
  -1,  // RL
   1   // RR
};
const int ENCODER_SIGN[4] = {
  -1,  // FL
   1,  // FR
  -1,  // RL
   1   // RR
};
// ============================================================
// CPR x4
// ============================================================
const float CPR[4] = {
  903.50f,
  939.93f,
  903.21f,
  905.21f
};
// ============================================================
// GEOMETRIA REAL DEL ROBOT
// ============================================================
const float RADIO  = 0.024f;
const float A_GEOM = 0.08f;
const float B_GEOM = 0.07f;
const float L_GEOM = A_GEOM + B_GEOM;  // 0.15 m
// ============================================================
// VELOCIDAD BASE
// ============================================================
const float RPM_MOVIMIENTO = 80.0f;
// Limite para comandos manuales V vx vy omega
const float RPM_MAX_MANUAL = 120.0f;
const float OMEGA_RUEDA_MOV =
  RPM_MOVIMIENTO * 2.0f * PI / 60.0f;
const float V_MOV =
  RADIO * OMEGA_RUEDA_MOV;
const float OMEGA_ROBOT_MOV =
  RADIO * OMEGA_RUEDA_MOV / L_GEOM;
// ============================================================
// GIRO 360
// ============================================================
const float ANGULO_VUELTA = 2.0f * PI;
float FACTOR_GIRO = 1.0f;
float thetaAcumulado = 0.0f;
// ============================================================
// TAREA 5 - VERSION PROBADA EXPERIMENTALMENTE
// Traslacion recta global + giro simultaneo de ~180 grados
//
// Esta parte NO usa los encoders para calcular posicion o angulo.
// Los encoders siguen usandose unicamente para el PI de velocidad.
//
// Parametros que ya se probaron fisicamente:
// omega = 0.50 rad/s
// V = omega / PI
// duracion = 6200 ms
// ============================================================
const float TAREA5_OMEGA_CCW = 0.50f;          // rad/s
const float TAREA5_V_GLOBAL = TAREA5_OMEGA_CCW / PI;
const float TAREA5_RPM_MAX = 120.0f;
const unsigned long TAREA5_MS = 6200;
// ============================================================
// PWM
// ============================================================
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
// ============================================================
// TEMPORIZACION
// ============================================================
const unsigned long TS_MS = 20;
const unsigned long REPORTE_MS = 200;
unsigned long ultimoControl = 0;
unsigned long ultimoReporte = 0;
unsigned long inicioControl = 0;
// ============================================================
// CONTROL PI ORIGINAL
// ============================================================
const float KP[4] = {
  0.50f, 0.50f, 0.50f, 0.50f
};
const float KI[4] = {
  0.45f, 0.45f, 0.45f, 0.45f
};
const float I_TERM_MAX = 30.0f;
const unsigned long I_DELAY_MS = 300;
const float ALPHA_RPM = 0.30f;
// Rampa PWM normal y rampa mas rapida para frenado
const float PWM_SLEW_NORMAL = 12.0f;
const float PWM_SLEW_FRENO  = 35.0f;
// Referencia baja de 80 a 0 rapidamente durante el frenado
const float FRENO_RPM_POR_SEG = 650.0f;
const float RPM_REPOSO = 3.0f;
const unsigned long FRENO_TIMEOUT_MS = 1200;
// ============================================================
// FEEDFORWARD POSITIVO REFINADO
// ============================================================
const int N_FF_FWD = 3;
const float RPM_FF_FWD[N_FF_FWD] = {
  80.0f,
  100.0f,
  120.0f
};
const float PWM_FF_FWD[4][N_FF_FWD] = {
  {216.5f, 230.7f, 241.4f},
  {220.2f, 233.7f, 245.3f},
  {217.4f, 231.1f, 243.4f},
  {218.3f, 231.1f, 242.6f}
};
// ============================================================
// FEEDFORWARD REVERSA ORIGINAL
// ============================================================
const int N_REV = 5;
const float PWM_REV[N_REV] = {
  195, 210, 225, 240, 255
};
const float RPM_REV[4][N_REV] = {
  {70.34f, 79.30f, 104.26f, 132.82f, 158.99f},
  {62.77f, 79.16f, 104.64f, 133.63f, 159.11f},
  {75.79f, 82.54f, 105.85f, 134.24f, 159.60f},
  {74.35f, 84.51f, 109.53f, 138.97f, 164.60f}
};
// ============================================================
// ENCODERS
// ============================================================
volatile int32_t conteos[4] = {0, 0, 0, 0};
volatile uint8_t estadoAnterior[4] = {0, 0, 0, 0};
const int8_t TABLA[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};
portMUX_TYPE muxEncoder = portMUX_INITIALIZER_UNLOCKED;
// ============================================================
// MEDICION
// ============================================================
int32_t conteoAnterior[4] = {0, 0, 0, 0};
float rpmRaw[4] = {0, 0, 0, 0};
float rpmFiltrada[4] = {0, 0, 0, 0};
bool filtroInicializado[4] = {false, false, false, false};
// ============================================================
// REFERENCIAS Y PI
// ============================================================
float referenciaRPM[4] = {0, 0, 0, 0};
float errorRPM[4] = {0, 0, 0, 0};
float iTerm[4] = {0, 0, 0, 0};
float pwmObjetivo[4] = {0, 0, 0, 0};
float pwmAplicado[4] = {0, 0, 0, 0};
// ============================================================
// MODOS
// ============================================================
enum ModoRobot {
  PARADO,
  MOVIMIENTO_CONTINUO,
  GIRO_360,
  TAREA5
};
ModoRobot modoRobot = PARADO;
bool controlActivo = false;
// ============================================================
// SECUENCIA
// ============================================================
enum PasoSecuencia {
  PASO_ESPERA_INICIAL,
  PASO_ADELANTE,
  PASO_ATRAS,
  PASO_IZQUIERDA,
  PASO_DERECHA,
  PASO_GIRO360,
  PASO_TAREA5
};
PasoSecuencia pasoSecuencia = PASO_ESPERA_INICIAL;
PasoSecuencia pasoPendiente = PASO_ESPERA_INICIAL;
bool secuenciaActiva = false;
bool secuenciaPausada = false;
bool pausaPorObstaculo = false;
bool transicionPendiente = false;
bool finalizarSecuenciaPendiente = false;
unsigned long tiempoActivoPaso = 0;
unsigned long ultimoTickPaso = 0;
const unsigned long ESPERA_INICIAL_MS = 3000;
const unsigned long PASO_MOVIMIENTO_MS = 3000;
// ============================================================
// FRENADO CONTROLADO POR PI
// ============================================================
bool frenandoPI = false;
unsigned long inicioFreno = 0;
// ============================================================
// BOTON - DEBOUNCE ESTABLE
// ============================================================
int ultimaLecturaBoton = HIGH;
int estadoBoton = HIGH;
unsigned long ultimoCambioBoton = 0;
const unsigned long BOTON_DEBOUNCE_MS = 80;
// ============================================================
// STALL
// ============================================================
int contadorStall = 0;
// ============================================================
// MODOS DE PRUEBA: TEMPORIZADO Y MANTENER
// ============================================================
const unsigned long HOLD_TIMEOUT_MS = 300;
unsigned long finTemporizadoMs = 0;
bool temporizadoActivo = false;
bool holdActivo = false;
unsigned long ultimoHoldMs = 0;
// ============================================================
// LED DE ESTADO
// Encendido = detenido o pausado. Apagado = en marcha.
// Cada pulsacion valida del boton produce 2 destellos.
// ============================================================
bool parpadeoLEDActivo = false;
uint8_t faseParpadeoLED = 0;
unsigned long ultimoCambioLED = 0;
const unsigned long LED_PARPADEO_MS = 140;
void escribirLED(bool encendido) {
  digitalWrite(LED_ESTADO, encendido ? HIGH : LOW);
}
bool robotFuncionandoParaLED() {
  if (secuenciaActiva && !secuenciaPausada) return true;
  if (controlActivo || frenandoPI) return true;
  return false;
}
void iniciarParpadeoDobleLED() {
  parpadeoLEDActivo = true;
  faseParpadeoLED = 0;
  ultimoCambioLED = millis();
  escribirLED(true);
}
void actualizarLED() {
  if (!parpadeoLEDActivo) {
    escribirLED(!robotFuncionandoParaLED());
    return;
  }
  if (millis() - ultimoCambioLED < LED_PARPADEO_MS) return;
  ultimoCambioLED = millis();
  faseParpadeoLED++;
  switch (faseParpadeoLED) {
    case 1: escribirLED(false); break;
    case 2: escribirLED(true);  break;
    case 3: escribirLED(false); break;
    default:
      parpadeoLEDActivo = false;
      escribirLED(!robotFuncionandoParaLED());
      break;
  }
}
// ============================================================
// AUXILIARES
// ============================================================
float limitarFloat(float valor, float minimo, float maximo) {
  if (valor < minimo) return minimo;
  if (valor > maximo) return maximo;
  return valor;
}
float acercar(float actual, float objetivo, float paso) {
  if (objetivo > actual + paso) return actual + paso;
  if (objetivo < actual - paso) return actual - paso;
  return objetivo;
}
// ============================================================
// ENCODERS
// ============================================================
void IRAM_ATTR actualizarEncoder(int i) {
  portENTER_CRITICAL_ISR(&muxEncoder);
  uint8_t a = digitalRead(ENC_A[i]);
  uint8_t b = digitalRead(ENC_B[i]);
  uint8_t nuevo = (a << 1) | b;
  uint8_t indice = (estadoAnterior[i] << 2) | nuevo;
  conteos[i] += TABLA[indice];
  estadoAnterior[i] = nuevo;
  portEXIT_CRITICAL_ISR(&muxEncoder);
}
void IRAM_ATTR ISR_FL() { actualizarEncoder(0); }
void IRAM_ATTR ISR_FR() { actualizarEncoder(1); }
void IRAM_ATTR ISR_RL() { actualizarEncoder(2); }
void IRAM_ATTR ISR_RR() { actualizarEncoder(3); }
// ============================================================
// COPIAR CONTEOS
// ============================================================
void copiarConteos(int32_t destino[4]) {
  portENTER_CRITICAL(&muxEncoder);
  for (int i = 0; i < 4; i++) {
    destino[i] = conteos[i];
  }
  portEXIT_CRITICAL(&muxEncoder);
}
// ============================================================
// MEDICION RPM + ANGULO DEL GIRO 360
// ============================================================
void actualizarMedicion(float dt) {
  if (dt <= 0) return;
  int32_t actual[4];
  copiarConteos(actual);
  float deltaPhi[4];
  for (int i = 0; i < 4; i++) {
    int32_t deltaRaw = actual[i] - conteoAnterior[i];
    conteoAnterior[i] = actual[i];
    int32_t deltaFisico = deltaRaw * ENCODER_SIGN[i];
    rpmRaw[i] =
      (deltaFisico * 60.0f) /
      (CPR[i] * dt);
    if (!filtroInicializado[i]) {
      rpmFiltrada[i] = rpmRaw[i];
      filtroInicializado[i] = true;
    } else {
      rpmFiltrada[i] =
        ALPHA_RPM * rpmRaw[i] +
        (1.0f - ALPHA_RPM) * rpmFiltrada[i];
    }
    deltaPhi[i] =
      ((float)deltaFisico / CPR[i]) * 2.0f * PI;
  }
  // Durante GIRO_360 seguimos acumulando incluso mientras frena,
  // para no perder el giro que ocurre durante la desaceleracion.
  if (modoRobot == GIRO_360) {
    float dTheta =
      RADIO /
      (4.0f * L_GEOM) *
      (
        deltaPhi[0]
        - deltaPhi[1]
        + deltaPhi[2]
        - deltaPhi[3]
      );
    thetaAcumulado += dTheta;
  }
}
// ============================================================
// FEEDFORWARD
// ============================================================
bool usarFeedForwardBajoRPM() {
  return (
    frenandoPI ||
    modoRobot == MOVIMIENTO_CONTINUO ||
    (secuenciaActiva && pasoSecuencia == PASO_TAREA5)
  );
}
float feedForwardPositivo(int i, float rpm) {
  float r = fabsf(rpm);
  if (r < 1.0f) return 0;
  // El FF original empieza en 80 RPM.
  // Para frenado y Tarea 5 necesitamos referencias menores.
  // En esa zona extrapolamos linealmente hacia cero.
  if (r <= RPM_FF_FWD[0]) {
    if (usarFeedForwardBajoRPM()) {
      return PWM_FF_FWD[i][0] * (r / RPM_FF_FWD[0]);
    }
    return PWM_FF_FWD[i][0];
  }
  for (int k = 0; k < N_FF_FWD - 1; k++) {
    if (r <= RPM_FF_FWD[k + 1]) {
      float fraccion =
        (r - RPM_FF_FWD[k]) /
        (RPM_FF_FWD[k + 1] - RPM_FF_FWD[k]);
      return
        PWM_FF_FWD[i][k] +
        fraccion *
        (PWM_FF_FWD[i][k + 1] - PWM_FF_FWD[i][k]);
    }
  }
  return PWM_FF_FWD[i][N_FF_FWD - 1];
}
float feedForwardReversa(int i, float rpm) {
  float objetivo = fabsf(rpm);
  if (objetivo < 1.0f) return 0;
  if (objetivo <= RPM_REV[i][0]) {
    if (usarFeedForwardBajoRPM()) {
      return -PWM_REV[0] * (objetivo / RPM_REV[i][0]);
    }
    return -PWM_REV[0];
  }
  for (int k = 0; k < N_REV - 1; k++) {
    if (objetivo <= RPM_REV[i][k + 1]) {
      float fraccion =
        (objetivo - RPM_REV[i][k]) /
        (RPM_REV[i][k + 1] - RPM_REV[i][k]);
      float pwm =
        PWM_REV[k] +
        fraccion *
        (PWM_REV[k + 1] - PWM_REV[k]);
      return -pwm;
    }
  }
  return -255.0f;
}
float feedForward(int i, float referencia) {
  if (referencia > 0) return feedForwardPositivo(i, referencia);
  if (referencia < 0) return feedForwardReversa(i, referencia);
  return 0;
}
// ============================================================
// MOTOR
// ============================================================
void aplicarMotor(int i, float comando) {
  comando = limitarFloat(comando, -255.0f, 255.0f);
  if (fabsf(comando) < 0.5f) {
    ledcWrite(EN[i], 0);
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
    return;
  }
  int sentidoFisico = comando > 0 ? 1 : -1;
  int direccionElectrica = sentidoFisico * MOTOR_SIGN[i];
  if (direccionElectrica > 0) {
    digitalWrite(IN1[i], HIGH);
    digitalWrite(IN2[i], LOW);
  } else {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], HIGH);
  }
  int duty = constrain(
    (int)roundf(fabsf(comando)),
    0,
    PWM_MAX
  );
  ledcWrite(EN[i], duty);
}
// ============================================================
// RESET PI
// ============================================================
void resetPI() {
  for (int i = 0; i < 4; i++) {
    errorRPM[i] = 0;
    iTerm[i] = 0;
    pwmObjetivo[i] = 0;
    pwmAplicado[i] = 0;
  }
  contadorStall = 0;
  inicioControl = millis();
}
// ============================================================
// STOP INMEDIATO
// Usado al iniciar, X y fallos duros.
// ============================================================
void detenerRobotInmediato(bool imprimir = true) {
  temporizadoActivo = false;
  holdActivo = false;
  controlActivo = false;
  frenandoPI = false;
  modoRobot = PARADO;
  for (int i = 0; i < 4; i++) {
    referenciaRPM[i] = 0;
    iTerm[i] = 0;
    pwmObjetivo[i] = 0;
    pwmAplicado[i] = 0;
    aplicarMotor(i, 0);
  }
  if (imprimir) {
    Serial.println("# ROBOT DETENIDO");
  }
}
// ============================================================
// REFERENCIAS
// ============================================================
void establecerReferencias(float fl, float fr, float rl, float rr) {
  referenciaRPM[0] = fl;
  referenciaRPM[1] = fr;
  referenciaRPM[2] = rl;
  referenciaRPM[3] = rr;
  resetPI();
  controlActivo = true;
}
void establecerReferenciasDinamicas(const float nuevas[4]) {
  for (int i = 0; i < 4; i++) {
    // Si una rueda cambia de signo durante Tarea 5,
    // limpiamos integral para evitar arrastre del signo anterior.
    if (
      (referenciaRPM[i] > 1.0f && nuevas[i] < -1.0f) ||
      (referenciaRPM[i] < -1.0f && nuevas[i] > 1.0f)
    ) {
      iTerm[i] = 0;
    }
    referenciaRPM[i] = nuevas[i];
  }
  controlActivo = true;
}
// ============================================================
// CINEMATICA MECANUM ORIGINAL
// vx: derecha (+)
// vy: adelante (+)
// omegaCW: horario (+)
// ============================================================
void calcularRPMMecanum(
  float vx,
  float vy,
  float omegaCW,
  float maxRPM,
  float salidaRPM[4]
) {
  float w[4];
  w[0] = (vy + vx + L_GEOM * omegaCW) / RADIO;
  w[1] = (vy - vx - L_GEOM * omegaCW) / RADIO;
  w[2] = (vy - vx + L_GEOM * omegaCW) / RADIO;
  w[3] = (vy + vx - L_GEOM * omegaCW) / RADIO;
  float maxEncontrada = 0;
  for (int i = 0; i < 4; i++) {
    salidaRPM[i] = w[i] * 60.0f / (2.0f * PI);
    float magnitud = fabsf(salidaRPM[i]);
    if (magnitud > maxEncontrada) maxEncontrada = magnitud;
  }
  if (maxEncontrada > maxRPM) {
    float escala = maxRPM / maxEncontrada;
    for (int i = 0; i < 4; i++) {
      salidaRPM[i] *= escala;
    }
  }
}
void cinematicaMecanum(
  float vx,
  float vy,
  float omegaCW,
  float maxRPM
) {
  float rpmTemp[4];
  calcularRPMMecanum(vx, vy, omegaCW, maxRPM, rpmTemp);
  establecerReferencias(
    rpmTemp[0],
    rpmTemp[1],
    rpmTemp[2],
    rpmTemp[3]
  );
}
void cinematicaMecanumDinamica(
  float vx,
  float vy,
  float omegaCW,
  float maxRPM
) {
  float rpmTemp[4];
  calcularRPMMecanum(vx, vy, omegaCW, maxRPM, rpmTemp);
  establecerReferenciasDinamicas(rpmTemp);
}
// ============================================================
// MOVIMIENTOS BASICOS
// ============================================================
void avanzar() {
  modoRobot = MOVIMIENTO_CONTINUO;
  cinematicaMecanum(0.0f, V_MOV, 0.0f, RPM_MOVIMIENTO);
  Serial.println("# ADELANTE 80 RPM");
}
void retroceder() {
  modoRobot = MOVIMIENTO_CONTINUO;
  cinematicaMecanum(0.0f, -V_MOV, 0.0f, RPM_MOVIMIENTO);
  Serial.println("# ATRAS 80 RPM");
}
void derecha() {
  modoRobot = MOVIMIENTO_CONTINUO;
  cinematicaMecanum(V_MOV, 0.0f, 0.0f, RPM_MOVIMIENTO);
  Serial.println("# DERECHA 80 RPM");
}
void izquierda() {
  modoRobot = MOVIMIENTO_CONTINUO;
  cinematicaMecanum(-V_MOV, 0.0f, 0.0f, RPM_MOVIMIENTO);
  Serial.println("# IZQUIERDA 80 RPM");
}
// ============================================================
// GIRO 360
// ============================================================
void iniciarGiro360(bool reiniciarAngulo = true) {
  if (reiniciarAngulo) {
    thetaAcumulado = 0.0f;
  }
  modoRobot = GIRO_360;
  cinematicaMecanum(
    0.0f,
    0.0f,
    OMEGA_ROBOT_MOV,
    RPM_MOVIMIENTO
  );
  modoRobot = GIRO_360;
  if (reiniciarAngulo) {
    Serial.println("# GIRO HORARIO 360 GRADOS");
  } else {
    Serial.println("# GIRO 360 REANUDADO");
  }
}
// ============================================================
// FRENADO PI
// ============================================================
void iniciarFrenadoPI() {
  if (frenandoPI) return;
  // Si ya esta quieto, no necesitamos rampa.
  if (!controlActivo) {
    frenandoPI = false;
    return;
  }
  // Quitamos memoria integral para que no siga empujando.
  for (int i = 0; i < 4; i++) {
    iTerm[i] = 0;
  }
  temporizadoActivo = false;
  holdActivo = false;
  frenandoPI = true;
  inicioFreno = millis();
  Serial.println("# FRENADO PI");
}
void actualizarFrenadoPI(float dt) {
  if (!frenandoPI) return;
  float pasoRPM = FRENO_RPM_POR_SEG * dt;
  for (int i = 0; i < 4; i++) {
    referenciaRPM[i] = acercar(referenciaRPM[i], 0.0f, pasoRPM);
    if (fabsf(referenciaRPM[i]) < 0.5f) {
      referenciaRPM[i] = 0.0f;
    }
  }
}
bool robotCasiQuieto() {
  for (int i = 0; i < 4; i++) {
    if (fabsf(rpmFiltrada[i]) > RPM_REPOSO) {
      return false;
    }
  }
  return true;
}
// Declaraciones adelantadas por uso cruzado
void entrarPaso(PasoSecuencia nuevoPaso);
void reanudarPasoActual();
void completarSecuencia() {
  detenerRobotInmediato(false);
  secuenciaActiva = false;
  secuenciaPausada = false;
  pausaPorObstaculo = false;
  transicionPendiente = false;
  finalizarSecuenciaPendiente = false;
  Serial.println();
  Serial.println("====================================");
  Serial.println(" SECUENCIA COMPLETADA");
  Serial.println(" Boton listo para iniciar de nuevo");
  Serial.println("====================================");
  Serial.println();
}
void procesarFinFreno() {
  if (!frenandoPI) return;
  bool timeout =
    (millis() - inicioFreno) >= FRENO_TIMEOUT_MS;
  if (!robotCasiQuieto() && !timeout) {
    return;
  }
  // Asegurar salida cero.
  for (int i = 0; i < 4; i++) {
    referenciaRPM[i] = 0;
    iTerm[i] = 0;
    pwmObjetivo[i] = 0;
    pwmAplicado[i] = 0;
    aplicarMotor(i, 0);
  }
  controlActivo = false;
  frenandoPI = false;
  modoRobot = PARADO;
  if (finalizarSecuenciaPendiente) {
    completarSecuencia();
    return;
  }
  // Si habia una transicion automatica pero el usuario
  // pauso la secuencia, dejamos el siguiente paso pendiente.
  if (transicionPendiente) {
    if (!secuenciaPausada) {
      PasoSecuencia siguiente = pasoPendiente;
      transicionPendiente = false;
      entrarPaso(siguiente);
    }
    return;
  }
  // Si simplemente se pauso, permanece detenido hasta otro botonazo.
}
// ============================================================
// SECUENCIA
// ============================================================
void solicitarTransicion(PasoSecuencia siguiente) {
  transicionPendiente = true;
  pasoPendiente = siguiente;
  iniciarFrenadoPI();
}
void solicitarFinSecuencia() {
  finalizarSecuenciaPendiente = true;
  iniciarFrenadoPI();
}
void entrarPaso(PasoSecuencia nuevoPaso) {
  pasoSecuencia = nuevoPaso;
  tiempoActivoPaso = 0;
  ultimoTickPaso = millis();
  switch (pasoSecuencia) {
    case PASO_ESPERA_INICIAL:
      detenerRobotInmediato(false);
      Serial.println("# Secuencia iniciada");
      Serial.println("# Esperando 3 s...");
      break;
    case PASO_ADELANTE:
      Serial.println("# PASO 1: ADELANTE");
      avanzar();
      break;
    case PASO_ATRAS:
      Serial.println("# PASO 2: ATRAS");
      retroceder();
      break;
    case PASO_IZQUIERDA:
      Serial.println("# PASO 3: IZQUIERDA");
      izquierda();
      break;
    case PASO_DERECHA:
      Serial.println("# PASO 4: DERECHA");
      derecha();
      break;
    case PASO_GIRO360:
      Serial.println("# PASO 5: GIRO 360");
      iniciarGiro360(true);
      break;
    case PASO_TAREA5:
      Serial.println("# PASO 6: TAREA 5");
      Serial.println("# Traslacion + giro 6.2 s");
      modoRobot = TAREA5;
      resetPI();
      controlActivo = true;
      break;
  }
}
void iniciarSecuencia() {
  detenerRobotInmediato(false);
  secuenciaActiva = true;
  secuenciaPausada = false;
  pausaPorObstaculo = false;
  transicionPendiente = false;
  finalizarSecuenciaPendiente = false;
  entrarPaso(PASO_ESPERA_INICIAL);
}
void pausarSecuencia(bool porObstaculo) {
  if (!secuenciaActiva || secuenciaPausada) return;
  secuenciaPausada = true;
  pausaPorObstaculo = porObstaculo;
  ultimoTickPaso = millis();
  if (controlActivo) {
    iniciarFrenadoPI();
  }
  if (porObstaculo) {
    Serial.println("# SECUENCIA PAUSADA POR OBSTACULO");
  } else {
    Serial.println("# SECUENCIA PAUSADA POR BOTON");
  }
}
void reanudarPasoActual() {
  if (!secuenciaActiva) return;
  ultimoTickPaso = millis();
  switch (pasoSecuencia) {
    case PASO_ESPERA_INICIAL:
      // Solo sigue contando el tiempo restante.
      break;
    case PASO_ADELANTE:
      avanzar();
      break;
    case PASO_ATRAS:
      retroceder();
      break;
    case PASO_IZQUIERDA:
      izquierda();
      break;
    case PASO_DERECHA:
      derecha();
      break;
    case PASO_GIRO360:
      // No se borra thetaAcumulado.
      iniciarGiro360(false);
      break;
    case PASO_TAREA5:
      modoRobot = TAREA5;
      resetPI();
      controlActivo = true;
      break;
  }
}
void reanudarSecuencia() {
  if (!secuenciaActiva || !secuenciaPausada) return;
  if (frenandoPI) {
    Serial.println("# Espere frenado");
    return;
  }
  secuenciaPausada = false;
  pausaPorObstaculo = false;
  ultimoTickPaso = millis();
  Serial.println("# SECUENCIA REANUDADA");
  if (transicionPendiente) {
    PasoSecuencia siguiente = pasoPendiente;
    transicionPendiente = false;
    entrarPaso(siguiente);
  } else {
    reanudarPasoActual();
  }
}
void abortarSecuencia() {
  secuenciaActiva = false;
  secuenciaPausada = false;
  pausaPorObstaculo = false;
  transicionPendiente = false;
  finalizarSecuenciaPendiente = false;
  detenerRobotInmediato(true);
  Serial.println("# SECUENCIA CANCELADA");
}
// ============================================================
// BOTON
// Primer botonazo: inicia.
// Durante secuencia: pausa / reanuda.
// Al terminar: vuelve a ser inicio.
// ============================================================
void eventoBoton() {
  // Todo botonazo normal produce dos destellos.
  iniciarParpadeoDobleLED();
  if (!secuenciaActiva) {
    iniciarSecuencia();
    return;
  }
  if (finalizarSecuenciaPendiente) {
    return;
  }
  if (!secuenciaPausada) {
    pausarSecuencia(false);
  } else {
    reanudarSecuencia();
  }
}
void actualizarBoton() {
  int lectura = digitalRead(BOTON);
  if (lectura != ultimaLecturaBoton) {
    ultimoCambioBoton = millis();
  }
  if ((millis() - ultimoCambioBoton) >= BOTON_DEBOUNCE_MS) {
    if (lectura != estadoBoton) {
      estadoBoton = lectura;
      if (estadoBoton == LOW) {
        eventoBoton();
      }
    }
  }
  ultimaLecturaBoton = lectura;
}
// ============================================================
// TAREA 5 DINAMICA - VERSION PROBADA
//
// Queremos que el centro del robot se traslade sobre una linea
// global mientras el chasis rota.
//
// El angulo es TEORICO:
// theta = omega * tiempoActivoPaso
//
// No se usan los encoders para estimar los 180 grados.
// Los encoders solamente mantienen las RPM mediante PI.
//
// Transformacion usada en la prueba que funciono:
// xdot = V * sin(theta)
// ydot = V * cos(theta)
//
// Nuestra cinematica usa giro horario positivo, por eso:
// omegaCW = -TAREA5_OMEGA_CCW
// ============================================================
void actualizarComandoTarea5() {
  if (
    !secuenciaActiva ||
    secuenciaPausada ||
    frenandoPI ||
    pasoSecuencia != PASO_TAREA5
  ) {
    return;
  }
  float t = tiempoActivoPaso / 1000.0f;
  if (t > (TAREA5_MS / 1000.0f)) {
    t = TAREA5_MS / 1000.0f;
  }
  float theta = TAREA5_OMEGA_CCW * t;
  float xdot =
    TAREA5_V_GLOBAL * sinf(theta);
  float ydot =
    TAREA5_V_GLOBAL * cosf(theta);
  float omegaCW =
    -TAREA5_OMEGA_CCW;
  cinematicaMecanumDinamica(
    xdot,
    ydot,
    omegaCW,
    TAREA5_RPM_MAX
  );
  modoRobot = TAREA5;
}
// ============================================================
// ACTUALIZAR SECUENCIA
// El tiempo NO avanza durante pausa o frenado.
// ============================================================
void actualizarSecuencia() {
  if (!secuenciaActiva) return;
  unsigned long ahora = millis();
  unsigned long delta = ahora - ultimoTickPaso;
  ultimoTickPaso = ahora;
  if (
    secuenciaPausada ||
    frenandoPI ||
    transicionPendiente ||
    finalizarSecuenciaPendiente
  ) {
    return;
  }
  tiempoActivoPaso += delta;
  switch (pasoSecuencia) {
    case PASO_ESPERA_INICIAL:
      if (tiempoActivoPaso >= ESPERA_INICIAL_MS) {
        entrarPaso(PASO_ADELANTE);
      }
      break;
    case PASO_ADELANTE:
      if (tiempoActivoPaso >= PASO_MOVIMIENTO_MS) {
        solicitarTransicion(PASO_ATRAS);
      }
      break;
    case PASO_ATRAS:
      if (tiempoActivoPaso >= PASO_MOVIMIENTO_MS) {
        solicitarTransicion(PASO_IZQUIERDA);
      }
      break;
    case PASO_IZQUIERDA:
      if (tiempoActivoPaso >= PASO_MOVIMIENTO_MS) {
        solicitarTransicion(PASO_DERECHA);
      }
      break;
    case PASO_DERECHA:
      if (tiempoActivoPaso >= PASO_MOVIMIENTO_MS) {
        solicitarTransicion(PASO_GIRO360);
      }
      break;
    case PASO_GIRO360:
      // Termina por encoders, no por tiempo.
      break;
    case PASO_TAREA5:
      if (tiempoActivoPaso >= TAREA5_MS) {
        solicitarFinSecuencia();
      }
      break;
  }
}
// ============================================================
// ACTUALIZAR GIRO 360
// ============================================================
void actualizarGiro360() {
  if (modoRobot != GIRO_360 || frenandoPI) return;
  float objetivo = ANGULO_VUELTA * FACTOR_GIRO;
  if (fabsf(thetaAcumulado) >= objetivo) {
    Serial.print("# GIRO 360 COMPLETADO: ");
    Serial.print(
      fabsf(thetaAcumulado) * 180.0f / PI,
      1
    );
    Serial.println(" grados estimados");
    if (
      secuenciaActiva &&
      pasoSecuencia == PASO_GIRO360
    ) {
      solicitarTransicion(PASO_TAREA5);
    } else {
      iniciarFrenadoPI();
    }
  }
}
// ============================================================
// PI
// ============================================================
void actualizarPI(float dt) {
  for (int i = 0; i < 4; i++) {
    float ref = referenciaRPM[i];
    if (fabsf(ref) < 1.0f) {
      iTerm[i] = 0;
      pwmObjetivo[i] = 0;
      float pasoSlew =
        frenandoPI ? PWM_SLEW_FRENO : PWM_SLEW_NORMAL;
      pwmAplicado[i] =
        acercar(pwmAplicado[i], 0.0f, pasoSlew);
      aplicarMotor(i, pwmAplicado[i]);
      continue;
    }
    errorRPM[i] = ref - rpmFiltrada[i];
    float ff = feedForward(i, ref);
    float pTerm = KP[i] * errorRPM[i];
    float nuevoITerm = iTerm[i];
    if (
      millis() - inicioControl >= I_DELAY_MS &&
      !frenandoPI
    ) {
      nuevoITerm += KI[i] * errorRPM[i] * dt;
      nuevoITerm = limitarFloat(
        nuevoITerm,
        -I_TERM_MAX,
         I_TERM_MAX
      );
    }
    float minimo;
    float maximo;
    if (ref > 0) {
      minimo = 0;
      maximo = 255;
    } else {
      minimo = -255;
      maximo = 0;
    }
    float uCandidate = ff + pTerm + nuevoITerm;
    bool puedeIntegrar = false;
    if (uCandidate >= minimo && uCandidate <= maximo) {
      puedeIntegrar = true;
    } else if (uCandidate > maximo && errorRPM[i] < 0) {
      puedeIntegrar = true;
    } else if (uCandidate < minimo && errorRPM[i] > 0) {
      puedeIntegrar = true;
    }
    if (puedeIntegrar && !frenandoPI) {
      iTerm[i] = nuevoITerm;
    }
    pwmObjetivo[i] = ff + pTerm + iTerm[i];
    pwmObjetivo[i] = limitarFloat(
      pwmObjetivo[i],
      minimo,
      maximo
    );
    float pasoSlew =
      frenandoPI ? PWM_SLEW_FRENO : PWM_SLEW_NORMAL;
    pwmAplicado[i] = acercar(
      pwmAplicado[i],
      pwmObjetivo[i],
      pasoSlew
    );
    aplicarMotor(i, pwmAplicado[i]);
  }
}
// ============================================================
// STALL
// ============================================================
void revisarStall() {
  if (!controlActivo || frenandoPI) return;
  if (millis() - inicioControl < 1500) return;
  float rpmPromedio = 0;
  float pwmPromedio = 0;
  for (int i = 0; i < 4; i++) {
    rpmPromedio += fabsf(rpmFiltrada[i]);
    pwmPromedio += fabsf(pwmAplicado[i]);
  }
  rpmPromedio /= 4.0f;
  pwmPromedio /= 4.0f;
  if (rpmPromedio < 10.0f && pwmPromedio > 245.0f) {
    contadorStall++;
  } else {
    contadorStall = 0;
  }
  if (contadorStall >= 20) {
    Serial.println();
    Serial.println("# ERROR: CAIDA GENERAL / STALL");
    secuenciaActiva = false;
    secuenciaPausada = false;
    transicionPendiente = false;
    finalizarSecuenciaPendiente = false;
    detenerRobotInmediato(true);
  }
}
// ============================================================
// TELEMETRIA
// ============================================================
void mostrarDatos() {
  Serial.print(millis());
  for (int i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(referenciaRPM[i], 1);
  }
  for (int i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(rpmFiltrada[i], 1);
  }
  for (int i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(pwmAplicado[i], 1);
  }
  Serial.print(",");
  Serial.print(thetaAcumulado * 180.0f / PI, 1);
  Serial.print(",");
  Serial.print(secuenciaActiva ? 1 : 0);
  Serial.print(",");
  Serial.print(secuenciaPausada ? 1 : 0);
  Serial.println();
}
// ============================================================
// SERIAL MANUAL
// ============================================================
void programarTemporizado(unsigned long ms) {
  if (ms == 0) return;
  temporizadoActivo = true;
  finTemporizadoMs = millis() + ms;
  Serial.print("# DURACION ");
  Serial.print(ms);
  Serial.println(" ms");
}
bool moverBasico(char c) {
  switch (c) {
    case 'W': avanzar();    return true;
    case 'S': retroceder(); return true;
    case 'A': izquierda();  return true;
    case 'D': derecha();    return true;
  }
  return false;
}
void procesarComando(char c, unsigned long ms) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c == 'X') {
    abortarSecuencia();
    return;
  }
  if (secuenciaActiva) {
    Serial.println("# MANUAL BLOQUEADO DURANTE SECUENCIA");
    return;
  }
  frenandoPI = false;
  if (c == 'R') {
    temporizadoActivo = false;
    holdActivo = false;
    iniciarGiro360(true);
    return;
  }
  temporizadoActivo = false;
  holdActivo = false;
  if (moverBasico(c)) {
    programarTemporizado(ms);
    return;
  }
  Serial.println("# COMANDO INVALIDO");
}
// ------------------------------------------------------------
// VELOCIDAD MANUAL X / Y / ANGULAR
// modo: 'V' continuo (o temporizado si ms > 0), 'H' mantener
// ------------------------------------------------------------
void establecerVelocidadManual(
  float vx,
  float vy,
  float omegaCCW,
  char modo,
  unsigned long ms
) {
  if (secuenciaActiva) {
    Serial.println("# V BLOQUEADO: SECUENCIA ACTIVA");
    return;
  }
  if (
    fabsf(vx) < 0.0001f &&
    fabsf(vy) < 0.0001f &&
    fabsf(omegaCCW) < 0.0001f
  ) {
    if (controlActivo) {
      iniciarFrenadoPI();
    }
    return;
  }
  // Un comando nuevo durante el frenado lo cancela y retoma.
  if (frenandoPI) {
    frenandoPI = false;
  }
  float omegaCW = -omegaCCW;
  float rpmTemp[4];
  calcularRPMMecanum(
    vx,
    vy,
    omegaCW,
    RPM_MAX_MANUAL,
    rpmTemp
  );
  if (!controlActivo || modoRobot != MOVIMIENTO_CONTINUO) {
    resetPI();
  }
  modoRobot = MOVIMIENTO_CONTINUO;
  establecerReferenciasDinamicas(rpmTemp);
  if (modo == 'H') {
    bool nuevo = !holdActivo;
    temporizadoActivo = false;
    holdActivo = true;
    ultimoHoldMs = millis();
    if (!nuevo) return;
    Serial.print("# H MANTENER");
  } else {
    holdActivo = false;
    temporizadoActivo = false;
    Serial.print("# V MANUAL");
  }
  Serial.print(" vx=");
  Serial.print(vx, 3);
  Serial.print(" vy=");
  Serial.print(vy, 3);
  Serial.print(" omega=");
  Serial.println(omegaCCW, 3);
  if (modo == 'V') {
    programarTemporizado(ms);
  }
}
void actualizarModosPrueba() {
  unsigned long ahora = millis();
  if (temporizadoActivo && (long)(ahora - finTemporizadoMs) >= 0) {
    temporizadoActivo = false;
    Serial.println("# FIN DE DURACION");
    iniciarFrenadoPI();
  }
  if (holdActivo && ahora - ultimoHoldMs >= HOLD_TIMEOUT_MS) {
    holdActivo = false;
    Serial.println("# H SOLTADO");
    iniciarFrenadoPI();
  }
}
// ------------------------------------------------------------
// PARSER DE LINEAS DEL SERIAL
// ------------------------------------------------------------
char bufferSerial[80];
uint8_t indiceSerial = 0;
void ayudaComandos() {
  Serial.println("# Use: W S A D [ms] | R | X");
  Serial.println("#      V vx vy omega [ms] | H vx vy omega");
}
void procesarLineaSerial(char *linea) {
  while (*linea == ' ' || *linea == '\t') linea++;
  if (*linea == '\0') return;
  char comando = linea[0];
  if (comando >= 'a' && comando <= 'z') comando -= 32;
  if (comando == '?') {
    ayudaComandos();
    return;
  }
  if (comando == 'V' || comando == 'H') {
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    unsigned long ms = 0;
    int n = sscanf(linea + 1, " %f %f %f %lu", &vx, &vy, &omega, &ms);
    if (n >= 3) {
      establecerVelocidadManual(vx, vy, omega, comando, ms);
      return;
    }
  } else {
    unsigned long ms = 0;
    char extra = '\0';
    int n = sscanf(linea + 1, " %lu %c", &ms, &extra);
    if (linea[1] == '\0' || n == 1) {
      procesarComando(comando, ms);
      return;
    }
  }
  Serial.println("# COMANDO INVALIDO");
  ayudaComandos();
}
void actualizarSerialManual() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (indiceSerial > 0) {
        bufferSerial[indiceSerial] = '\0';
        procesarLineaSerial(bufferSerial);
        indiceSerial = 0;
      }
      continue;
    }
    if (indiceSerial < sizeof(bufferSerial) - 1) {
      bufferSerial[indiceSerial++] = c;
    } else {
      indiceSerial = 0;
      Serial.println("# LINEA SERIAL MUY LARGA");
    }
  }
}
// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  // Motores
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    ledcAttach(
      EN[i],
      PWM_FREQ,
      PWM_RESOLUTION
    );
    ledcWrite(EN[i], 0);
  }
  // Encoders
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_A[i], INPUT);
    pinMode(ENC_B[i], INPUT);
    estadoAnterior[i] =
      (digitalRead(ENC_A[i]) << 1) |
      digitalRead(ENC_B[i]);
  }
  pinMode(LED_ESTADO, OUTPUT);
  escribirLED(true);
  pinMode(BOTON, INPUT_PULLUP);
  // Interrupciones encoders
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), ISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), ISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), ISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), ISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), ISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), ISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), ISR_RR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), ISR_RR, CHANGE);
  detenerRobotInmediato(false);
  int32_t actual[4];
  copiarConteos(actual);
  for (int i = 0; i < 4; i++) {
    conteoAnterior[i] = actual[i];
  }
  ultimoControl = millis();
  ultimoReporte = millis();
  ultimoTickPaso = millis();
  Serial.println();
  Serial.println("========================================");
  Serial.println(" ROBOT MECANUM - LOCOMOCION SIN IR");
  Serial.println("========================================");
  Serial.println("LED: GPIO2 | Boton: GPIO12");
  Serial.println("Manual: W S A D [ms] | R | X");
  Serial.println("Vector: V vx vy omega [ms]");
  Serial.println("Mantener: H vx vy omega (timeout 300 ms)");
  Serial.println();
  Serial.println(
    "CSV: ms,FL_ref,FR_ref,RL_ref,RR_ref,FL_rpm,FR_rpm,RL_rpm,RR_rpm,FL_pwm,FR_pwm,RL_pwm,RR_pwm,theta_deg,seq,pausa"
  );
  Serial.println();
}
// ============================================================
// LOOP
// ============================================================
void loop() {
  actualizarSerialManual();
  actualizarModosPrueba();
  actualizarBoton();
  // LED no bloqueante: no interfiere con el PI.
  actualizarLED();
  // Maquina de estados de la secuencia
  actualizarSecuencia();
  unsigned long ahora = millis();
  // Control cada 20 ms
  if (ahora - ultimoControl >= TS_MS) {
    float dt =
      (ahora - ultimoControl) /
      1000.0f;
    ultimoControl = ahora;
    actualizarMedicion(dt);
    // Giro 360 por encoder
    actualizarGiro360();
    // Tarea 5 actualiza referencias continuamente
    actualizarComandoTarea5();
    // Freno: baja referencias rapidamente hacia cero
    actualizarFrenadoPI(dt);
    if (controlActivo) {
      actualizarPI(dt);
      revisarStall();
    }
    // Comprueba si el frenado ya termino
    procesarFinFreno();
  }
  // Telemetria cada 200 ms
  if (ahora - ultimoReporte >= REPORTE_MS) {
    ultimoReporte = ahora;
    if (
      controlActivo ||
      secuenciaActiva
    ) {
      mostrarDatos();
    }
  }
}
