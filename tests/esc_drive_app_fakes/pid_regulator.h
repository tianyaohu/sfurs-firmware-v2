#pragma once
#include <stdint.h>
typedef struct { int16_t kp, ki; int16_t up, low; int32_t iup, ilow; uint16_t kidiv; int32_t wIntegralTerm; } PID_Handle_t;
int16_t PI_Controller(PID_Handle_t *p, int32_t err);
