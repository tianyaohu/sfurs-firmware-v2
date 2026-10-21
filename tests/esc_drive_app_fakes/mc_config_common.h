#pragma once
#include <stdint.h>
typedef struct { uint16_t ConversionFactor; uint16_t raw; } BusVoltageSensor_Handle_t;
typedef struct { BusVoltageSensor_Handle_t _Super; } RDivider_Handle_t;
extern RDivider_Handle_t BusVoltageSensor_M1;
static inline uint16_t VBS_GetAvBusVoltage_d(const BusVoltageSensor_Handle_t *p) { return p->raw; }
