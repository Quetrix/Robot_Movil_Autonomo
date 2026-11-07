#pragma once
// Introducir tablas individuales medidas: {voltios antes del divisor, mm}.
// Minimo 3 puntos por sensor, voltios crecientes y distancias decrecientes.
// NVS gp21v1 tiene prioridad. Ver README.md. No hay asistente en esta version.
// No copiar los coeficientes de GP2Y0A02 a GP2Y0A21.
struct CalPoint { float volts; float mm; };
constexpr CalPoint IR_LEFT_DEFAULT[] = {{0, 0}};
constexpr CalPoint IR_RIGHT_DEFAULT[] = {{0, 0}};
constexpr int IR_LEFT_COUNT = 0, IR_RIGHT_COUNT = 0;
