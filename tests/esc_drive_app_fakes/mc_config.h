#pragma once
#include <stdint.h>
#include "pid_regulator.h"
typedef struct { int16_t PhaseShift; int8_t Direction; uint8_t HallState; int16_t MeasuredElAngle;
                 uint16_t EdgesPos, EdgesNeg; uint8_t BufferFilled, SpeedBufferSize; } HALL_Handle_t;
extern PID_Handle_t PIDIqHandle_M1, PIDIdHandle_M1; extern HALL_Handle_t HALL_M1;
static inline void PID_SetKP(PID_Handle_t *p, int16_t v) { p->kp = v; }
static inline void PID_SetKI(PID_Handle_t *p, int16_t v) { p->ki = v; }
static inline int16_t PID_GetKP(PID_Handle_t *p) { return p->kp; }
static inline int16_t PID_GetKI(PID_Handle_t *p) { return p->ki; }
