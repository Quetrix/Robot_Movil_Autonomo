#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
class Sensors {
public:
  uint32_t errors=0;
  bool configured=false;
  uint16_t lastTick=0;
  bool pending=false, discardFirst=true, haveTick=false;
  uint32_t triggeredAt=0, lastTrigger=0;
  bool read(uint8_t reg,uint8_t* p,uint8_t n) {
    Wire.beginTransmission(cfg::TF_ADDRESS); Wire.write(reg);
    if(Wire.endTransmission(false)!=0) { ++errors; return false; }
    if(Wire.requestFrom(cfg::TF_ADDRESS,n)!=n) {
      while(Wire.available()) Wire.read();
      ++errors; return false;
    }
    for(uint8_t i=0;i<n;++i) p[i]=Wire.read();
    return true;
  }
  bool write(uint8_t reg,uint8_t v) {
    Wire.beginTransmission(cfg::TF_ADDRESS); Wire.write(reg); Wire.write(v);
    bool ok=Wire.endTransmission()==0; if(!ok) ++errors; return ok;
  }
  static uint16_t u16(const uint8_t* p) { return uint16_t(p[0])|(uint16_t(p[1])<<8); }
  void begin() {
    Wire.begin(cfg::SDA,cfg::SCL,100000); Wire.setTimeOut(20);
    pinMode(cfg::TF_READY,INPUT);
    analogReadResolution(12);
    for(int c=0;c<2;++c) analogSetPinAttenuation(cfg::IR_PINS[c],ADC_11db);
    delay(500);
    bool ok=write(0x1F,0); delay(100);
    ok=write(0x28,0)&&ok; delay(100);
    ok=write(0x25,1)&&ok; delay(100);
    ok=write(0x23,1)&&ok; delay(100); // I2C single-shot mode, no EEPROM save.
    uint8_t mode=0, junk[10];
    ok=read(0x23,&mode,1)&&ok;
    ok=read(0,junk,10)&&ok; // Clear old data-ready.
    configured=ok && mode==1;
  }
  float irVolts(int channel, float& adcMv) {
    analogReadMilliVolts(cfg::IR_PINS[channel]); // Dummy after mux switch.
    uint32_t values[9];
    for(int i=0;i<9;++i) { values[i]=analogReadMilliVolts(cfg::IR_PINS[channel]); delayMicroseconds(60); }
    for(int i=0;i<8;++i) for(int j=i+1;j<9;++j) if(values[i]>values[j]) { uint32_t t=values[i]; values[i]=values[j]; values[j]=t; }
    adcMv=values[4];
    if(adcMv<150 || adcMv>2800) return NAN;
    return adcMv/(1000.0f*cfg::DIVIDER_RATIO[channel]);
  }
};
