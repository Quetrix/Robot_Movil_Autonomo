// Stub minimo de Arduino/ESP32 para probar la logica del firmware en la PC.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#ifndef PI
#define PI 3.14159265358979f
#endif
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
#define portENTER_CRITICAL_ISR(x) (void)(x)
#define portEXIT_CRITICAL_ISR(x) (void)(x)

extern unsigned long sim_ms;
extern int sim_pin[40];
extern int sim_ledc[40];
typedef void (*isr_t)();
extern isr_t sim_isr[40];

inline unsigned long millis() { return sim_ms; }
inline void delay(unsigned long ms) { sim_ms += ms; }
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { sim_pin[p] = v; }
inline int digitalRead(int p) { return sim_pin[p]; }
inline void ledcAttach(int, int, int) {}
inline void ledcWrite(int p, int d) { sim_ledc[p] = d; }
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int p, isr_t f, int) { sim_isr[p] = f; }

struct SimSerial {
  std::string out, in;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in[0]; in.erase(0, 1); return c; }
  void print(const char* s) { out += s; }
  void print(char c) { out += c; }
  void print(int v) { out += std::to_string(v); }
  void print(long v) { out += std::to_string(v); }
  void print(unsigned long v) { out += std::to_string(v); }
  void print(unsigned int v) { out += std::to_string(v); }
  void print(double v, int d = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", d, v); out += b; }
  void println() { out += "\n"; }
  template <typename T> void println(T v) { print(v); out += "\n"; }
  void println(double v, int d) { print(v, d); out += "\n"; }
};
extern SimSerial Serial;
