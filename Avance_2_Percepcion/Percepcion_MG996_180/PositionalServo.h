#pragma once
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "Config.h"
class PositionalServo {
  bool attached_=false;
  float head_=NAN;
public:
  bool attached()const{return attached_;}
  float head()const{return head_;}
  static bool angleAllowed(float head){return isfinite(head)&&head>=cfg::HEAD_MIN_DEG&&head<=cfg::HEAD_MAX_DEG;}
  static uint16_t pulseFor(float head){
    float a=head*cfg::SERVO_SIGN;
    float us=a>=0?cfg::SERVO_US_CENTER+(cfg::SERVO_US_PLUS60-cfg::SERVO_US_CENTER)*a/60:
                       cfg::SERVO_US_CENTER+(cfg::SERVO_US_CENTER-cfg::SERVO_US_MINUS60)*a/60;
    return uint16_t(lroundf(us));
  }
  void begin(){pinMode(cfg::SERVO,OUTPUT);digitalWrite(cfg::SERVO,LOW);}
  bool writeDegrees(float head){
    if(!angleAllowed(head))return false;
    if(!attached_){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      if(!ledcAttach(cfg::SERVO,50,16))return false;
#else
      double hz=ledcSetup(0,50,16);if(hz<49||hz>51)return false;
      ledcAttachPin(cfg::SERVO,0);
#endif
      attached_=true;
    }
    uint32_t duty=(uint32_t(pulseFor(head))*65536UL+10000UL)/20000UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if(!ledcWrite(cfg::SERVO,duty)){release();return false;}
#else
    ledcWrite(0,duty);
#endif
    head_=head;return true;
  }
  void release(){
    if(attached_){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcDetach(cfg::SERVO);
#else
      ledcWrite(0,0);ledcDetachPin(cfg::SERVO);
#endif
    }
    pinMode(cfg::SERVO,OUTPUT);digitalWrite(cfg::SERVO,LOW);attached_=false;head_=NAN;
  }
};
