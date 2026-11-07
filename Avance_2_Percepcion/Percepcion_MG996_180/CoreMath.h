#pragma once
#include <math.h>
#include <stdint.h>
#include "Config.h"
#include "CalDefaults.h"
inline bool finiteRange(float v, float lo, float hi) {
  return isfinite(v) && v >= lo && v <= hi;
}
inline bool tableValid(const CalPoint* p, int n) {
  if(n < 3 || n > cfg::CAL_MAX_POINTS) return false;
  for(int i=0;i<n;++i) {
    if(!finiteRange(p[i].volts,0.1f,5.5f) ||
       !finiteRange(p[i].mm,cfg::IR_MIN_MM,cfg::IR_MAX_MM)) return false;
    if(i && (p[i].volts<=p[i-1].volts || p[i].mm>=p[i-1].mm)) return false;
  }
  return true;
}
inline float interpolate(const CalPoint* p, int n, float volts) {
  if(!tableValid(p,n) || !finiteRange(volts,p[0].volts,p[n-1].volts)) return NAN;
  for(int i=1;i<n;++i) if(volts<=p[i].volts) {
    float u=(volts-p[i-1].volts)/(p[i].volts-p[i-1].volts);
    return p[i-1].mm + u*(p[i].mm-p[i-1].mm);
  }
  return NAN;
}
inline float gp21Preliminary(float volts) {
  // Reuse the old GP2Y0A02 curve SHAPE, not its distance scale. Normalize it
  // between the GP2Y0A21 typical endpoints. This is an unverified estimate.
  if(!finiteRange(volts,cfg::IR_PRELIM_FAR_V,cfg::IR_PRELIM_NEAR_V)) return NAN;
  float oldFar=cfg::IR_OLD_C0_CM*powf(cfg::IR_PRELIM_FAR_V,cfg::IR_OLD_EXPONENT);
  float oldNear=cfg::IR_OLD_C0_CM*powf(cfg::IR_PRELIM_NEAR_V,cfg::IR_OLD_EXPONENT);
  float oldValue=cfg::IR_OLD_C0_CM*powf(volts,cfg::IR_OLD_EXPONENT);
  float u=(oldValue-oldNear)/(oldFar-oldNear);
  float mm=cfg::IR_MIN_MM+(cfg::IR_MAX_MM-cfg::IR_MIN_MM)*u;
  return finiteRange(mm,cfg::IR_MIN_MM,cfg::IR_MAX_MM)?mm:NAN;
}
inline float irDistance(const CalPoint* p,int n,float volts,bool& preliminary,
                         bool allowPreliminary=cfg::IR_ALLOW_PRELIMINARY) {
  preliminary=false;
  // An individual valid table always wins. Never fall back outside its window.
  if(tableValid(p,n)) return interpolate(p,n,volts);
  if(!allowPreliminary) return NAN;
  preliminary=true;
  return gp21Preliminary(volts);
}
inline float tfDistance(float nativeMm, uint16_t amp, uint16_t error, bool& legacy) {
  legacy=false;
  if(!finiteRange(nativeMm,cfg::TF_MIN_MM,cfg::TF_MAX_MM) ||
     amp<100 || amp>=32768 || error!=0) return NAN;
  float out=nativeMm;
  if(cfg::TF_USE_LEGACY_CAL && finiteRange(nativeMm,cfg::TF_LEGACY_MIN_MM,cfg::TF_LEGACY_MAX_MM)) {
    out=cfg::TF_SCALE*nativeMm+10*cfg::TF_OFFSET_CM;
    legacy=true;
    if(!finiteRange(out,cfg::TF_LEGACY_MIN_MM,cfg::TF_LEGACY_MAX_MM)) return NAN;
  }
  return finiteRange(out,cfg::TF_MIN_MM,cfg::TF_MAX_MM)?out:NAN;
}
inline void headToRobot(float headDeg, float betaDeg, float originX, float originY,
                        float distance, float& x, float& y) {
  const float k=0.0174532925199433f;
  float a=headDeg*k, b=betaDeg*k;
  float hx=originX+distance*cosf(b), hy=originY+distance*sinf(b);
  x=hx*cosf(a)-hy*sinf(a); y=hx*sinf(a)+hy*cosf(a);
}
