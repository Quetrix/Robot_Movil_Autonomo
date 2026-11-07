#pragma once
#include "CoreMath.h"
struct Measurement {
  float mm=NAN,radius=NAN,head=NAN,ray=NAN,bearing=NAN,x=NAN,y=NAN;
  uint32_t at=0;
  bool valid=false,pose=false,preliminary=false;
};
inline Measurement measurement(float mm,int sensor,float head,uint32_t at,bool preliminary){
  Measurement m;m.at=at;m.preliminary=preliminary;m.valid=finiteRange(mm,sensor<2?cfg::IR_MIN_MM:cfg::TF_MIN_MM,sensor<2?cfg::IR_MAX_MM:cfg::TF_MAX_MM);
  if(!m.valid)return m;
  m.mm=mm;
  if(!isfinite(head))return m;
  m.head=head;m.ray=head+cfg::BETA_DEG[sensor];
  headToRobot(head,cfg::BETA_DEG[sensor],cfg::ORIGIN_X_MM[sensor],cfg::ORIGIN_Y_MM[sensor],mm,m.x,m.y);
  m.radius=hypotf(m.x,m.y);m.bearing=atan2f(m.y,m.x)*57.2957795131f;m.pose=true;return m;
}
struct Contribution {
  Measurement sample;
  bool accepted=false;
  float weight=0;
  const char* reason="sin_memoria";
};
struct Fusion {
  Measurement result;
  Contribution sources[3];
  uint8_t contributors=0;
};
class AngularMemory {
  Measurement cells[4][cfg::MEMORY_BINS]; // Izq, Der, TF y resultado; RAM solamente.
  float savedWeights[cfg::MEMORY_BINS][3]{};
  static bool better(const Measurement& a,const Measurement& b,float angle,uint32_t now){
    if(!b.valid)return true;
    float da=fabsf(a.bearing-angle),db=fabsf(b.bearing-angle);
    return da<db || (fabsf(da-db)<0.0001f&&now-a.at<now-b.at);
  }
public:
  static int bin(float bearing){
    if(!isfinite(bearing)||bearing<cfg::MEMORY_MIN_DEG||bearing>cfg::MEMORY_MAX_DEG)return -1;
    return int(lroundf((bearing-cfg::MEMORY_MIN_DEG)/cfg::MEMORY_BIN_DEG));
  }
  void clear(){for(int s=0;s<4;++s)for(int i=0;i<cfg::MEMORY_BINS;++i)cells[s][i]=Measurement{};for(int i=0;i<cfg::MEMORY_BINS;++i)for(int s=0;s<3;++s)savedWeights[i][s]=0;}
  bool store(int sensor,const Measurement& m){
    int i=bin(m.bearing);if(sensor<0||sensor>3||i<0||i>=cfg::MEMORY_BINS||!m.valid||!m.pose)return false;
    if(cells[sensor][i].valid&&int32_t(m.at-cells[sensor][i].at)<0)return false;
    cells[sensor][i]=m;return true;
  }
  void storeFusion(const Fusion& f){if(store(3,f.result)){int i=bin(f.result.bearing);for(int s=0;s<3;++s)savedWeights[i][s]=f.sources[s].weight;}}
  float weight(int i,int sensor)const{return i>=0&&i<cfg::MEMORY_BINS&&sensor>=0&&sensor<3?savedWeights[i][sensor]:0;}
  const Measurement* cell(int sensor,int i,uint32_t now)const{
    if(sensor<0||sensor>3||i<0||i>=cfg::MEMORY_BINS)return nullptr;
    const Measurement& m=cells[sensor][i];return m.valid&&now-m.at<=cfg::MEMORY_MAX_AGE_MS?&m:nullptr;
  }
  Contribution findIR(int sensor,const Measurement& tf,uint32_t now)const{
    Contribution out;if(sensor<0||sensor>1)return out;Measurement closest,best;
    for(int i=0;i<cfg::MEMORY_BINS;++i){
      const Measurement& m=cells[sensor][i];
      if(!m.valid||fabsf(m.bearing-tf.bearing)>cfg::ALIGN_TOLERANCE_DEG+0.0001f)continue;
      if(better(m,closest,tf.bearing,now))closest=m;
      if(now-m.at>cfg::MEMORY_MAX_AGE_MS)continue;
      if(fabsf(m.radius-tf.radius)>fmaxf(cfg::MAX_DISTANCE_ERROR_MM,cfg::MAX_DISTANCE_ERROR_FRACTION*tf.radius))continue;
      if(better(m,best,tf.bearing,now))best=m;
    }
    if(best.valid){out.sample=best;out.accepted=true;out.reason="alineado";}
    else if(closest.valid){out.sample=closest;out.reason=now-closest.at>cfg::MEMORY_MAX_AGE_MS?"vencido":"distancia_incompatible";}
    return out;
  }
  Fusion combine(const Measurement& tf,uint32_t now)const{
    Fusion f;
    if(!tf.valid||!tf.pose||now-tf.at>cfg::MAX_AGE_MS){f.sources[2].reason="TF_invalido_o_sin_pose";return f;}
    f.sources[2].sample=tf;f.sources[2].accepted=true;f.sources[2].reason="TF_actual";
    f.sources[0]=findIR(0,tf,now);f.sources[1]=findIR(1,tf,now);
    float sum=0,weighted=0;bool preliminary=false;
    for(int s=0;s<3;++s)if(f.sources[s].accepted){float w=cfg::NOMINAL_WEIGHTS[s];sum+=w;weighted+=w*f.sources[s].sample.radius;preliminary|=f.sources[s].sample.preliminary;++f.contributors;}
    for(int s=0;s<3;++s)if(f.sources[s].accepted)f.sources[s].weight=cfg::NOMINAL_WEIGHTS[s]/sum;
    f.result=tf;f.result.mm=f.result.radius=weighted/sum;f.result.preliminary=preliminary;
    float a=tf.bearing*0.0174532925199433f;f.result.x=f.result.radius*cosf(a);f.result.y=f.result.radius*sinf(a);
    return f;
  }
};
