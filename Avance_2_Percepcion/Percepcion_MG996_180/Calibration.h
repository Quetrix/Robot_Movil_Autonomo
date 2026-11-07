#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "CoreMath.h"
class Calibration {
public:
  CalPoint points[2][cfg::CAL_MAX_POINTS]{};
  int count[2]={0,0};
  bool consistent(int ch) const {
    int n=count[ch]; if(n<0 || n>cfg::CAL_MAX_POINTS) return false;
    for(int i=0;i<n;++i) {
      if(!finiteRange(points[ch][i].volts,0.1f,5.5f) ||
         !finiteRange(points[ch][i].mm,100,800)) return false;
      if(i && (points[ch][i].volts<=points[ch][i-1].volts || points[ch][i].mm>=points[ch][i-1].mm)) return false;
    }
    return true;
  }
  void begin() {
    const CalPoint* defaults[]={IR_LEFT_DEFAULT,IR_RIGHT_DEFAULT};
    const int sizes[]={IR_LEFT_COUNT,IR_RIGHT_COUNT};
    for(int c=0;c<2;++c) {
      count[c]=sizes[c];
      if(count[c]<=cfg::CAL_MAX_POINTS) for(int i=0;i<count[c];++i) points[c][i]=defaults[c][i];
      if(!consistent(c)) count[c]=0;
    }
    Preferences p;
    if(!p.begin("gp21v1",true)) return;
    for(int c=0;c<2;++c) {
      const char* key=c==0?"left":"right";
      size_t bytes=p.getBytesLength(key);
      if(bytes && bytes%sizeof(CalPoint)==0 && bytes<=sizeof(points[c])) {
        count[c]=bytes/sizeof(CalPoint); p.getBytes(key,points[c],bytes);
        if(!consistent(c)) count[c]=0;
      }
    }
    p.end();
  }
  bool ready(int ch) const { return tableValid(points[ch],count[ch]); }
  float distance(int ch,float volts,bool& preliminary) const {
    return irDistance(points[ch],count[ch],volts,preliminary);
  }
};
