/* ESP32-WROOM + MG996 POSICIONAL AD18394. Arduino IDE, sin librerias externas.
   Abra ESTE archivo dentro de su carpeta completa. No usar con servo continuo.
   No hay movimiento al arrancar: CENTER / SCAN / ANGLE / STOP / RELEASE.
   Todos los angulos de percepcion son comandados/estimados, no medidos. */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "Sensors.h"
#include "Calibration.h"
#include "PositionalServo.h"
#include "AngularMemory.h"
#include "Dashboard.h"

Sensors sensors;
Calibration calibration;
PositionalServo servo;
AngularMemory memory;
WebServer server(80);
Measurement current[3],pending[3];
Fusion fused;
const char* sensorStatus[3]={"sin_datos","sin_datos","sin_datos"};
const char* pendingStatus[3]={"sin_datos","sin_datos","sin_datos"};
const char* lastMessage="Primero CENTER: el servo arranca sin senal.";
const char* ids[4]={"IR_I","IR_D","TF","FINAL"};
const char* mode="hold";
bool webGuard=false,roundActive=false,irDone=false,tfDone=false,committed=false;
float manualTarget=0;
int direction=1;
uint32_t poseAt=0,settleMs=0,roundAt=0,sequence=0,captureAt=0,lastBeat=0;
uint32_t lastPublish=0;
String commandLine,serialPending;
size_t serialOffset=0;
bool discardLine=false,wantMatrix=false;
bool wantDetail=false,wantAll=false;
float irVoltage[2]={NAN,NAN},irAdcMv[2]={NAN,NAN},tfNativeMm=NAN;

String num(float value,int digits){return isfinite(value)?String(value,digits):String("null");}
String printable(float value,int digits){return isfinite(value)?String(value,digits):String("--");}
bool fresh(const Measurement& m,uint32_t now,uint32_t limit){return m.valid&&now-m.at<=limit;}

void resetRound(uint32_t now,uint32_t settling){
  poseAt=now;settleMs=settling;roundActive=false;committed=false;irDone=tfDone=false;sensors.pending=false;
}
bool commandHead(float head,uint32_t now,bool first){
  if(!servo.writeDegrees(head)){lastMessage="No se pudo aplicar el angulo/PWM.";return false;}
  resetRound(now,first?cfg::FIRST_SETTLE_MS:cfg::STEP_SETTLE_MS);
  return true;
}
bool scanActive(){return strcmp(mode,"scan")==0;}
const char* statusAt(int sensor,uint32_t now){return current[sensor].valid&&!fresh(current[sensor],now,cfg::MAX_AGE_MS)?"vencido":sensorStatus[sensor];}

bool execute(const String& text,float angle,bool fromWeb){
  uint32_t now=millis();
  if(text=="STOP"){
    mode="hold";webGuard=false;lastMessage="Detenido: mantiene el ultimo angulo, sin mandar un retorno.";return true;
  }
  if(text=="RELEASE"){
    mode="hold";webGuard=false;servo.release();memory.clear();fused=Fusion{};
    for(int i=0;i<3;++i)current[i]=Measurement{};
    resetRound(now,0);lastMessage="Senal retirada. La plataforma puede moverse; CENTER antes de barrer.";return true;
  }
  if(text=="CLEAR"){
    memory.clear();fused=Fusion{};resetRound(now,servo.attached()?cfg::STEP_SETTLE_MS:0);lastMessage="Memoria angular borrada; las nuevas lecturas la llenaran.";return true;
  }
  if(text=="CENTER"){
    webGuard=false;manualTarget=0;memory.clear();fused=Fusion{};
    if(!servo.attached()){
      if(!commandHead(0,now,true))return false;
      mode="hold";
    }else mode="manual";
    lastMessage="Centro solicitado. Ajustar fisicamente el montaje al frente calibrado.";return true;
  }
  if(text=="SCAN"){
    if(!servo.attached()){lastMessage="Ejecute CENTER antes de SCAN.";return false;}
    mode="scan";webGuard=fromWeb;lastBeat=now;
    direction=servo.head()>=cfg::HEAD_MAX_DEG?-1:1;
    lastMessage="Barrido por grados con pausa para medir.";return true;
  }
  if(text=="ANGLE"){
    if(!servo.attached()||!PositionalServo::angleAllowed(angle)){lastMessage="CENTER primero; ANGLE permite -60 a +60 grados finitos.";return false;}
    manualTarget=angle;mode="manual";webGuard=false;lastMessage="Angulo solicitado; aproximacion en pasos limitados.";return true;
  }
  lastMessage="Comando no valido.";return false;
}

void startRound(uint32_t now){
  roundAt=now;roundActive=true;committed=false;irDone=false;tfDone=!sensors.configured;
  for(int i=0;i<3;++i){pending[i]=Measurement{};pendingStatus[i]="esperando_lectura";}
  pendingStatus[2]=sensors.configured?"esperando_TF":"TF_sin_configuracion";
  if(sensors.configured){
    uint8_t discard[10];
    if(!sensors.read(0,discard,10)||!sensors.write(0x24,1)){
      tfDone=true;pendingStatus[2]="error_I2C";
    }else{sensors.pending=true;sensors.triggeredAt=millis();}
  }
}
void readIR(uint32_t now){
  for(int i=0;i<2;++i){
    float adcMv=NAN;float volts=sensors.irVolts(i,adcMv);bool preliminary=false;
    irVoltage[i]=volts;irAdcMv[i]=adcMv;
    float mm=calibration.distance(i,volts,preliminary);
    uint32_t at=millis(); // Muestra tomada tras asentamiento + ventana optica, con el cabezal quieto.
    pending[i]=measurement(mm,i,servo.head(),at,preliminary);
    pendingStatus[i]=pending[i].valid?(pending[i].pose?"ok":"ok_sin_pose"):(isfinite(volts)?"IR_fuera_de_rango":"IR_senal_invalida");
  }
  (void)now;irDone=true;
}
void pollTF(uint32_t now){
  if(tfDone)return;
  if(now-sensors.triggeredAt>=2&&digitalRead(cfg::TF_READY)){
    uint8_t data[10];sensors.pending=false;tfDone=true;
    if(!sensors.read(0,data,10)){pendingStatus[2]="error_I2C";return;}
    uint16_t tick=Sensors::u16(data+6),amp=Sensors::u16(data+2),error=Sensors::u16(data+8);
    if(sensors.discardFirst){sensors.discardFirst=false;pendingStatus[2]="sincronizando";}
    else if(sensors.haveTick&&tick==sensors.lastTick)pendingStatus[2]="TF_trama_repetida";
    else{
      float native=10.0f*Sensors::u16(data);tfNativeMm=native;bool legacy=false;
      float mm=tfDistance(native,amp,error,legacy);uint32_t received=millis();
      pending[2]=measurement(mm,2,servo.head(),sensors.triggeredAt+(received-sensors.triggeredAt)/2,false);
      pendingStatus[2]=pending[2].valid?(pending[2].pose?"ok":"ok_sin_pose"):
        (native>0&&native<cfg::TF_MIN_MM?"TF_menos_de_20cm_no_fiable":"TF_rango_o_calidad_invalida");
    }
    sensors.lastTick=tick;sensors.haveTick=true;
  }else if(now-sensors.triggeredAt>cfg::TF_TIMEOUT_MS){
    sensors.pending=false;tfDone=true;pendingStatus[2]="TF_timeout";
  }
}
void nextPose(uint32_t now){
  float head=servo.head();
  if(servo.attached()&&scanActive()){
    float next=head+direction*cfg::SCAN_STEP_DEG;
    if(next>cfg::HEAD_MAX_DEG){direction=-1;next=head-cfg::SCAN_STEP_DEG;}
    if(next<cfg::HEAD_MIN_DEG){direction=1;next=head+cfg::SCAN_STEP_DEG;}
    if(!commandHead(next,now,false))mode="hold";
  }else if(servo.attached()&&strcmp(mode,"manual")==0&&fabsf(head-manualTarget)>0.001f){
    float delta=manualTarget-head;float next=head+(delta>0?1:-1)*fminf(fabsf(delta),cfg::SCAN_STEP_DEG);
    if(!commandHead(next,now,false))mode="hold";
  }else{mode="hold";resetRound(now,0);}
}
void servicePerception(uint32_t now){
  if(webGuard&&scanActive()&&now-lastBeat>cfg::WEB_TIMEOUT_MS){mode="hold";webGuard=false;lastMessage="Barrido detenido por perdida de enlace; mantiene el angulo.";}
  if(now-poseAt<settleMs)return;
  if(!roundActive)startRound(now);
  pollTF(millis());
  if(!irDone&&millis()-roundAt>=cfg::IR_OPTICAL_WAIT_MS)readIR(millis());
  if(irDone&&tfDone&&!committed){
    uint32_t at=millis();
    for(int i=0;i<3;++i){current[i]=pending[i];sensorStatus[i]=pendingStatus[i];memory.store(i,current[i]);}
    fused=memory.combine(current[2],at);memory.storeFusion(fused);
    committed=true;++sequence;captureAt=at;
  }
  if(committed&&millis()-poseAt>=cfg::POSE_PERIOD_MS)nextPose(millis());
}

String measurementJson(const Measurement& m,uint32_t now,uint32_t maxAge){
  bool valid=fresh(m,now,maxAge),pose=valid&&m.pose;
  return String("{\"valid\":")+(valid?"true":"false")+",\"pose\":"+(pose?"true":"false")+
    ",\"preliminary\":"+(m.preliminary?"true":"false")+",\"mm\":"+num(valid?m.mm:NAN,1)+
    ",\"radius_mm\":"+num(pose?m.radius:NAN,1)+",\"head_deg\":"+num(pose?m.head:NAN,1)+
    ",\"ray_deg\":"+num(pose?m.ray:NAN,1)+",\"bearing_deg\":"+num(pose?m.bearing:NAN,1)+
    ",\"x_mm\":"+num(pose?m.x:NAN,1)+",\"y_mm\":"+num(pose?m.y:NAN,1)+
    ",\"sample_ms\":"+String(m.at)+",\"age_ms\":"+(m.valid?String(now-m.at):String("null"))+"}";
}
String weightedMatrixJson(uint32_t now){
  String out="[";out.reserve(2400);bool comma=false;
  for(int bin=0;bin<cfg::MEMORY_BINS;++bin){
    const Measurement* m=memory.cell(3,bin,now);if(!m)continue;
    if(comma)out+=',';
    comma=true;
    out+="["+num(m->bearing,2)+","+num(m->radius,1)+"]";
  }
  return out+"]";
}
String dataJson(){
  uint32_t now=millis();String j;j.reserve(24000);
  const char* state=now-poseAt<settleMs?"asentando":committed?"captura_completa":"midiendo";
  j=String("{\"version\":\"")+cfg::VERSION+"\",\"ms\":"+String(now)+",\"seq\":"+String(sequence)+
    ",\"capture_ms\":"+String(captureAt)+",\"head_command_deg\":"+num(servo.head(),1)+
    ",\"attached\":"+(servo.attached()?"true":"false")+",\"mode\":\""+mode+"\",\"state\":\""+state+
    "\",\"message\":\""+lastMessage+"\",\"i2c_errors\":"+String(sensors.errors)+
    ",\"bin_deg\":"+num(cfg::MEMORY_BIN_DEG,1)+",\"tolerance_deg\":"+num(cfg::ALIGN_TOLERANCE_DEG,1)+
    ",\"memory_ms\":"+String(cfg::MEMORY_MAX_AGE_MS)+",\"geometry_measured\":"+(cfg::GEOMETRY_MEASURED?"true":"false")+
    ",\"ir_volts\":["+num(irVoltage[0],4)+","+num(irVoltage[1],4)+"],\"ir_adc_mv\":["+num(irAdcMv[0],1)+","+num(irAdcMv[1],1)+
    "],\"tf_native_mm\":"+num(tfNativeMm,1)+",\"weighted_matrix\":"+weightedMatrixJson(now)+",\"samples\":[";
  for(int i=0;i<3;++i){if(i)j+=',';j+=measurementJson(current[i],now,cfg::MAX_AGE_MS);}
  j+=String("],\"sensor_status\":[\"")+statusAt(0,now)+"\",\""+statusAt(1,now)+"\",\""+statusAt(2,now)+"\"],\"fusion\":"+measurementJson(fused.result,now,cfg::MAX_AGE_MS)+",\"sources\":[";
  for(int i=0;i<3;++i){if(i)j+=',';const Contribution& c=fused.sources[i];
    j+="{\"used\":"+String(c.accepted&&fresh(fused.result,now,cfg::MAX_AGE_MS)?"true":"false")+
       ",\"weight\":"+num(fresh(fused.result,now,cfg::MAX_AGE_MS)?c.weight:0,4)+",\"reason\":\""+c.reason+"\",\"sample\":"+measurementJson(c.sample,now,cfg::MEMORY_MAX_AGE_MS)+"}";
  }
  j+="],\"rows\":[";bool comma=false;
  for(int bin=0;bin<cfg::MEMORY_BINS;++bin){
    const Measurement* cells[4];bool any=false;
    for(int i=0;i<4;++i){cells[i]=memory.cell(i,bin,now);any|=cells[i]!=nullptr;}
    if(!any)continue;
    if(comma)j+=',';
    comma=true;
    j+="{\"angle_bin_deg\":"+num(cfg::MEMORY_MIN_DEG+bin*cfg::MEMORY_BIN_DEG,0)+",\"cells\":[";
    for(int i=0;i<4;++i){
      if(i)j+=',';
      if(!cells[i]){j+="null";continue;}
      const Measurement& m=*cells[i];
      j+="{\"radius_mm\":"+num(m.radius,1)+",\"sensor_mm\":"+num(m.mm,1)+",\"head_deg\":"+num(m.head,1)+
        ",\"bearing_deg\":"+num(m.bearing,1)+",\"age_ms\":"+String(now-m.at)+",\"preliminary\":"+(m.preliminary?"true":"false")+"}";
    }
    j+="],\"weights\":["+num(cells[3]?memory.weight(bin,0):0,4)+","+num(cells[3]?memory.weight(bin,1):0,4)+","+num(cells[3]?memory.weight(bin,2):0,4)+"]}";
  }
  return j+"]}";
}
String serialTable(){
  uint32_t now=millis();String out;out.reserve(2200);
  out="\n# Captura "+String(sequence)+" t="+String(captureAt)+"ms cabezal_cmd="+printable(servo.head(),1)+"deg modo="+mode+"\n";
  out+="sensor;head_cmd_deg;rayo_est_deg;d_sensor_mm;r_eje_mm;x_est_mm;y_est_mm;edad_ms;estado\n";
  for(int i=0;i<4;++i){const Measurement& m=i<3?current[i]:fused.result;bool valid=fresh(m,now,cfg::MAX_AGE_MS),pose=valid&&m.pose;
    out+=String(ids[i])+";"+printable(pose?m.head:NAN,1)+";"+printable(pose?m.ray:NAN,1)+";"+
      (m.preliminary&&valid?"~":"")+printable(valid?m.mm:NAN,1)+";"+printable(pose?m.radius:NAN,1)+";"+
      printable(pose?m.x:NAN,1)+";"+printable(pose?m.y:NAN,1)+";"+(m.valid?String(now-m.at):String("--"))+";"+
      (i<3?statusAt(i,now):(valid?"ponderado":"sin_TF_valido_o_pose"))+"\n";
  }
  out+="FUENTES PARA LA DIRECCION TF: sensor;r_eje_mm;edad_ms;peso_efectivo;uso\n";
  bool valid=fresh(fused.result,now,cfg::MAX_AGE_MS);
  for(int i=0;i<3;++i){const Contribution& c=fused.sources[i];
    out+=String(ids[i])+";"+printable(c.sample.valid?c.sample.radius:NAN,1)+";"+(c.sample.valid?String(now-c.sample.at):String("--"))+";"+
       String(valid?c.weight:0,4)+";"+(valid?c.reason:"TF_invalido_o_vencido")+"\n";
  }
  out+="IR_volts="+printable(irVoltage[0],4)+","+printable(irVoltage[1],4)+" ADC_mV="+printable(irAdcMv[0],1)+","+printable(irAdcMv[1],1)+" TF_crudo_mm="+printable(tfNativeMm,1)+"\n";
  return out;
}
String serialWeightedMatrix(){
  uint32_t now=millis();String out;out.reserve(3000);
  out="\n# MATRIZ FINAL captura="+String(sequence)+" t="+String(now)+"ms vigencia="+String(cfg::MEMORY_MAX_AGE_MS)+"ms\n";
  out+="angulo_estimado_deg;distancia_ponderada_mm\n";
  bool any=false;
  for(int bin=0;bin<cfg::MEMORY_BINS;++bin){
    const Measurement* m=memory.cell(3,bin,now);if(!m)continue;
    any=true;out+=num(m->bearing,2)+";"+num(m->radius,1)+"\n";
  }
  if(!any)out+="# Sin resultado TF valido con orientacion conocida. CENTER y SCAN para llenar la matriz.\n";
  return out;
}
String serialMatrix(){
  uint32_t now=millis();String out="\n# MEMORIA ANGULAR: r_eje_mm(edad_ms), --=sin dato reciente\nangulo_bin_deg;IR_I;IR_D;TF;FINAL;w_I;w_D;w_TF\n";
  for(int bin=0;bin<cfg::MEMORY_BINS;++bin){bool any=false;const Measurement* c[4];for(int i=0;i<4;++i){c[i]=memory.cell(i,bin,now);any|=c[i]!=nullptr;}if(!any)continue;
    out+=num(cfg::MEMORY_MIN_DEG+bin*cfg::MEMORY_BIN_DEG,0);
    for(int i=0;i<4;++i)out+=c[i]?";"+String(c[i]->radius,1)+"("+String(now-c[i]->at)+")":String(";--");
    for(int i=0;i<3;++i)out+=";"+String(c[3]?memory.weight(bin,i):0,4);
    out+='\n';
  }
  return out;
}
void setupWeb(){
  WiFi.persistent(false);WiFi.mode(WIFI_AP);WiFi.softAP(cfg::SSID,cfg::PASSWORD);
  server.on("/",HTTP_GET,[](){server.sendHeader("Cache-Control","no-store");server.send_P(200,"text/html; charset=utf-8",DASHBOARD);});
  server.on("/data",HTTP_GET,[](){lastBeat=millis();server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",dataJson());});
  server.on("/matrix",HTTP_GET,[](){
    uint32_t now=millis();server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",String("{\"columns\":[\"angulo_deg\",\"distancia_ponderada_mm\"],\"max_age_ms\":")+
      String(cfg::MEMORY_MAX_AGE_MS)+",\"ms\":"+String(now)+",\"matrix\":"+weightedMatrixJson(now)+"}");
  });
  server.on("/action",HTTP_POST,[](){
    String cmd=server.arg("cmd");cmd.toUpperCase();float angle=NAN;
    if(cmd=="ANGLE"){String text=server.arg("deg");char* end=nullptr;angle=strtof(text.c_str(),&end);
      if(!text.length()||end==text.c_str()||*end||!isfinite(angle)){server.send(400,"text/plain","Angulo invalido.");return;}}
    bool ok=execute(cmd,angle,true);server.send(ok?200:400,"text/plain",lastMessage);
  });
  server.begin();
}
void serialCommands(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\r'||ch=='\n'){
      if(discardLine){discardLine=false;commandLine="";continue;}
      if(!commandLine.length())continue;
      commandLine.trim();commandLine.toUpperCase();float angle=NAN;char tail;
      if(commandLine=="MATRIX")wantMatrix=true;
      else if(commandLine=="DETAIL")wantDetail=true;
      else if(commandLine=="MATRIX_ALL")wantAll=true;
      else if(sscanf(commandLine.c_str(),"ANGLE %f %c",&angle,&tail)==1)execute("ANGLE",angle,false);
      else execute(commandLine,NAN,false);
      commandLine="";
    }else if(!discardLine){if(commandLine.length()<63)commandLine+=ch;else{commandLine="";discardLine=true;}}
  }
}
void pumpSerial(){
  if(!serialPending.length())return;
  int capacity=Serial.availableForWrite();if(capacity<=0)return;
  size_t left=serialPending.length()-serialOffset,n=left<size_t(capacity)?left:size_t(capacity);
  serialOffset+=Serial.write(reinterpret_cast<const uint8_t*>(serialPending.c_str()+serialOffset),n);
  if(serialOffset==serialPending.length()){serialPending="";serialOffset=0;}
}
void setup(){
  servo.begin();Serial.begin(115200);calibration.begin();sensors.begin();memory.clear();resetRound(millis(),0);
  setupWeb();
  Serial.println("# MG996 POSICIONAL: sin movimiento automatico. CENTER, SCAN, STOP, ANGLE -60..60, RELEASE, CLEAR, MATRIX, DETAIL, MATRIX_ALL.");
  Serial.println("# Salida principal: matriz [angulo estimado (grados); distancia ponderada desde eje (mm)], memoria reciente no simultanea.");
  Serial.println("# Angulos comandados/estimados; ~ indica Sharp preliminar. Centro servo nominal 90deg = frente 0deg. Calibrar geometria y anclas antes de afirmar precision.");
}
void loop(){
  serialCommands();servicePerception(millis());pumpSerial();server.handleClient();
  uint32_t now=millis();
  if(!serialPending.length()&&(wantMatrix||wantDetail||wantAll||now-lastPublish>=cfg::PUBLISH_MS)){
    if(wantDetail){serialPending=serialTable();wantDetail=false;}
    else if(wantAll){serialPending=serialMatrix();wantAll=false;}
    else{serialPending=serialWeightedMatrix();wantMatrix=false;}
    serialOffset=0;lastPublish=now;
  }
  delay(1);
}
