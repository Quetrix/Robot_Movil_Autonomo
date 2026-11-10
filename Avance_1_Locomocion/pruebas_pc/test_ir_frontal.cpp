// Prueba en la PC de la logica del IR frontal del firmware de locomocion.
// Compilar y correr desde esta carpeta:
//   g++ -std=gnu++17 -I. -x c++ test_ir_frontal.cpp -o test_ir && ./test_ir
#include "Arduino.h"
unsigned long sim_ms = 0;
int sim_pin[40] = {0};
int sim_ledc[40] = {0};
isr_t sim_isr[40] = {nullptr};
SimSerial Serial;

#ifndef LOCO_INO
#define LOCO_INO "../locomocion_esp32_ir_frontal/locomocion_esp32_ir_frontal.ino"
#endif
#include LOCO_INO

static int fallas = 0;
static void check(bool ok, const char* msg) {
  printf("%s  %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok) fallas++;
}
// Modelo ideal de rueda: la RPM real sigue a la referencia.
static double acum[4] = {0, 0, 0, 0};
static void paso1ms() {
  sim_ms++;
  for (int i = 0; i < 4; i++) {
    acum[i] += referenciaRPM[i] * CPR[i] / 60.0 * 0.001;
    int entero = (int)acum[i];
    acum[i] -= entero;
    conteos[i] += entero * ENCODER_SIGN[i];
  }
  loop();
}
static std::string hostH;           // comando H que la "Pi" repite a 10 Hz
static void correr(unsigned long ms) {
  for (unsigned long k = 0; k < ms; k++) {
    if (!hostH.empty() && sim_ms % 100 == 0) Serial.in += hostH + "\n";
    paso1ms();
  }
}
static void enviar(const char* s) { Serial.in += std::string(s) + "\n"; }
static void ir(int nivel) {
  sim_pin[IR_FRENTE] = nivel;
  if (sim_isr[IR_FRENTE]) sim_isr[IR_FRENTE]();
}
static size_t cuenta(const std::string& t, const char* s) {
  size_t n = 0, p = 0;
  while ((p = t.find(s, p)) != std::string::npos) { n++; p++; }
  return n;
}
static float media() { return (referenciaRPM[0] + referenciaRPM[1] + referenciaRPM[2] + referenciaRPM[3]) / 4; }

int main() {
  sim_pin[IR_FRENTE] = HIGH;
  sim_pin[BOTON] = HIGH;
  setup();
  check(Serial.out.find("# IR FRENTE: LIBRE") != std::string::npos, "arranque con IR libre");
  check(Serial.out.find("theta_deg,irF,seq,pausa") != std::string::npos, "encabezado CSV con irF");

  // 1) Avance con H a 10 Hz, aparece obstaculo -> frenado controlado
  Serial.out.clear();
  hostH = "H 0 0.200 0.000";
  correr(600);
  check(controlActivo && media() > 70, "H adelante: referencias positivas (~80 RPM)");
  ir(LOW);
  correr(10);
  check(!irFrenteObstaculo, "10 ms en LOW: aun no confirma (debounce)");
  correr(15);
  check(irFrenteObstaculo, "25 ms en LOW: OBSTACULO confirmado");
  check(Serial.out.find("# IR FRENTE: OBSTACULO") != std::string::npos, "mensaje OBSTACULO");
  check(frenandoPI || !controlActivo, "inicia frenado PI al detectar");
  correr(1500);
  check(!controlActivo && fabsf(media()) < 0.5f, "robot detenido mientras sigue llegando H adelante");
  check(cuenta(Serial.out, "AVANCE BLOQUEADO") == 1, "aviso de bloqueo una sola vez por episodio");

  // 2) Con obstaculo: retroceso, lateral y giro permitidos; W bloqueado
  hostH = "H 0 -0.200 0.000";
  correr(400);
  check(controlActivo && media() < -70, "retroceso permitido con obstaculo");
  hostH = "H 0.200 0 0.000";
  correr(400);
  check(controlActivo && fabsf(media()) < 1 && referenciaRPM[0] > 50 && referenciaRPM[1] < -50,
        "lateral derecha permitido (FL+, FR-)");
  hostH = "H 0 0.200 1.000";
  correr(400);
  check(controlActivo && fabsf(media()) < 1 && referenciaRPM[1] > 30 && referenciaRPM[0] < -30,
        "adelante+giro antihorario: queda solo el giro (FR+, FL-)");
  hostH.clear();
  correr(800);
  check(Serial.out.find("# H SOLTADO") != std::string::npos && !controlActivo, "timeout de H frena (300 ms)");
  enviar("W");
  correr(300);
  check(!controlActivo, "W rechazado con obstaculo");
  enviar("V 0 0.2 0 2000");
  correr(300);
  check(!controlActivo, "V adelante temporizado rechazado con obstaculo");
  enviar("R");
  correr(300);
  check(controlActivo && modoRobot == GIRO_360, "R (giro) permitido con obstaculo");
  enviar("X");
  correr(50);

  // 3) Obstaculo desaparece -> LIBRE y se puede avanzar
  Serial.out.clear();
  ir(HIGH);
  correr(40);
  check(irFrenteObstaculo, "40 ms en HIGH: aun obstaculo (histeresis 60 ms)");
  correr(40);
  check(!irFrenteObstaculo && Serial.out.find("# IR FRENTE: LIBRE") != std::string::npos, "LIBRE tras 60 ms en HIGH");
  hostH = "H 0 0.200 0.000";
  correr(500);
  check(controlActivo && media() > 70, "avance de nuevo permitido");

  // 4) Pulso corto (ruido) no dispara
  ir(LOW); correr(8); ir(HIGH); correr(100);
  check(!irFrenteObstaculo && controlActivo && media() > 70, "pulso LOW de 8 ms ignorado");

  // 5) Movimiento en curso adelante+lateral: al detectar sigue solo lateral
  hostH.clear();
  enviar("V 0.150 0.150 0");
  correr(400);
  Serial.out.clear();
  ir(LOW);
  correr(30);
  check(Serial.out.find("AVANCE ANULADO, SIGUE LATERAL/GIRO") != std::string::npos && controlActivo &&
        fabsf(media()) < 1 && referenciaRPM[0] > 20, "V adelante+lateral: queda solo la parte lateral");
  ir(HIGH);
  correr(100);
  check(controlActivo && fabsf(media()) < 1, "al quedar libre NO reanuda solo el avance de un V");
  enviar("X");
  correr(50);

  // 6) W continuo y obstaculo -> frena
  enviar("W");
  correr(400);
  check(controlActivo && media() > 70, "W continuo avanza");
  ir(LOW);
  correr(1500);
  check(!controlActivo, "W continuo frena al detectar");
  ir(HIGH);
  correr(100);

  // 7) CSV con columna irF
  hostH = "H 0 -0.1 0";
  ir(LOW);
  Serial.out.clear();
  correr(500);
  size_t p = Serial.out.rfind('\n', Serial.out.size() - 2);
  std::string ultima;
  for (size_t a = 0, b; (b = Serial.out.find('\n', a)) != std::string::npos; a = b + 1) {
    std::string ln = Serial.out.substr(a, b - a);
    if (!ln.empty() && isdigit((unsigned char)ln[0])) ultima = ln;
  }
  (void)p;
  int comas = 0;
  for (char c : ultima) comas += (c == ',');
  check(comas == 16, "CSV con 17 columnas");
  check(ultima.size() > 4 && ultima.substr(ultima.size() - 6) == ",1,0,0", "columna irF = 1 con obstaculo");
  printf("ultima linea CSV: %s\n", ultima.c_str());

  printf("\n%s (%d fallas)\n", fallas ? "HAY FALLAS" : "TODO OK", fallas);
  return fallas ? 1 : 0;
}
