#pragma once
#include <stdint.h>
namespace cfg {
constexpr char VERSION[]="mg996-matriz-1.2-sharp-fijos";
constexpr uint8_t IR_PINS[2]={34,35},SDA=21,SCL=22,TF_READY=27,SERVO=25,TF_ADDRESS=0x10;
constexpr char SSID[]="Percepcion-ESP32",PASSWORD[]="percepcion26";
// ---- Geometria (montaje v2, nov 2026). Fuente: Documentos/Geometria_robot.md ----
// Marco de percepcion: origen = eje del servo, X adelante, Y izquierda, angulos antihorarios.
// El eje del servo esta 62 mm adelante del centro del chasis (placa L4, centrado en Y).
constexpr float AXIS_FROM_CENTER_X_MM=62,AXIS_FROM_CENTER_Y_MM=0;
// Solo el TF-Luna gira con el servo; los Sharp estan fijos en la placa L4.
constexpr bool ON_HEAD[3]={false,false,true}; // Sharp izquierdo, derecho, TF-Luna.
// Sharp: pivote del soporte a (-40, +/-36) del eje; giro hacia afuera leido en la escala grabada.
constexpr float SHARP_YAW_DEG=45;              // [AJUSTAR] valor real de la escala (0..90)
constexpr float SHARP_PIVOT_X_MM=-40,SHARP_PIVOT_Y_MM=36;
constexpr float SHARP_FACE_FROM_PIVOT_MM=20;   // [ESTIMADO] pivote -> cara del sensor, medir
constexpr float BETA_DEG[3]={SHARP_YAW_DEG,-SHARP_YAW_DEG,0};
// TF-Luna: origen en el marco del cabezal (lente ~7 mm delante del eje, calibrar).
constexpr float TF_ORIGIN_X_MM=7,TF_ORIGIN_Y_MM=0;
constexpr bool GEOMETRY_MEASURED=false; // true cuando SHARP_YAW y SHARP_FACE esten medidos.
constexpr float HEAD_MIN_DEG=-60,HEAD_MAX_DEG=60,SCAN_STEP_DEG=5;
constexpr int SERVO_SIGN=1; // Cambiar a -1 si aumenta hacia la derecha fisica.
// Anclas INICIALES, no calibradas. Solo el controlador convierte grados a PWM.
// Aproximacion conservadora basada en 1000..2000 us para -90..90 grados.
// No garantiza esos angulos fisicos: medir estas tres anclas en ESTA unidad.
constexpr uint16_t SERVO_US_MINUS60=1167,SERVO_US_CENTER=1500,SERVO_US_PLUS60=1833;
constexpr uint32_t FIRST_SETTLE_MS=1000,STEP_SETTLE_MS=100,IR_OPTICAL_WAIT_MS=55,POSE_PERIOD_MS=180;
constexpr uint32_t TF_TIMEOUT_MS=100,PUBLISH_MS=500,WEB_TIMEOUT_MS=1500;
constexpr uint32_t MAX_AGE_MS=250,MEMORY_MAX_AGE_MS=12000;
constexpr float MEMORY_MIN_DEG=-150,MEMORY_MAX_DEG=150,MEMORY_BIN_DEG=2,ALIGN_TOLERANCE_DEG=2;
constexpr int MEMORY_BINS=int((MEMORY_MAX_DEG-MEMORY_MIN_DEG)/MEMORY_BIN_DEG)+1;
constexpr float NOMINAL_WEIGHTS[3]={0.1f,0.1f,0.8f};
constexpr float MAX_DISTANCE_ERROR_MM=100,MAX_DISTANCE_ERROR_FRACTION=0.2f;
constexpr float IR_MIN_MM=100,IR_MAX_MM=800,TF_MIN_MM=200,TF_MAX_MM=8000;
constexpr bool IR_ALLOW_PRELIMINARY=true;
constexpr float IR_OLD_C0_CM=60.14f,IR_OLD_EXPONENT=-1.054f,IR_PRELIM_FAR_V=0.4f,IR_PRELIM_NEAR_V=2.3f;
constexpr float DIVIDER_RATIO[2]={1,1};
constexpr bool TF_USE_LEGACY_CAL=true;
constexpr float TF_SCALE=1.0068f,TF_OFFSET_CM=-1.13f,TF_LEGACY_MIN_MM=200,TF_LEGACY_MAX_MM=2100;
constexpr int CAL_MAX_POINTS=12;
static_assert(SHARP_YAW_DEG>=0&&SHARP_YAW_DEG<=90,"Ranura de los Sharp: 0..90 grados");
static_assert(HEAD_MIN_DEG>=-90 && HEAD_MAX_DEG<=90 && SCAN_STEP_DEG>0,"Revisar limites mecanicos calibrados");
static_assert(POSE_PERIOD_MS>=STEP_SETTLE_MS+IR_OPTICAL_WAIT_MS,"Falta tiempo para adquirir en reposo");
}

static_assert(cfg::SERVO_SIGN==1||cfg::SERVO_SIGN==-1,"SERVO_SIGN debe ser +1 o -1");
static_assert(cfg::SERVO_US_MINUS60<cfg::SERVO_US_CENTER&&cfg::SERVO_US_CENTER<cfg::SERVO_US_PLUS60,"Anclas PWM no monotonicas");

static_assert(cfg::NOMINAL_WEIGHTS[2]>0&&cfg::NOMINAL_WEIGHTS[0]>=0&&cfg::NOMINAL_WEIGHTS[1]>=0,"Pesos invalidos");
