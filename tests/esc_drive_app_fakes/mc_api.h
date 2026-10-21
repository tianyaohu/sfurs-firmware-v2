#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { int16_t q, d; } qd_t;
enum { IDLE = 0, CHARGE_BOOT_CAP = 16, OFFSET_CALIB = 17, RUN = 6, STOP = 8, WAIT_STOP_MOTOR = 20,
       FAULT_NOW = 10, FAULT_OVER = 11 };
#define MC_NO_FAULTS 0U
#define U_RPM 60
#define SPEED_UNIT U_RPM
#define RPM_2_SPEED_UNIT(rpm) ((int16_t)(((rpm)*SPEED_UNIT)/U_RPM))
#define SPEED_UNIT_2_RPM(speed) ((int16_t)(((speed)*U_RPM)/SPEED_UNIT))
bool MC_StartMotor1(void); bool MC_StopMotor1(void);
void MC_ProgramSpeedRampMotor1(int16_t, uint16_t);
void MC_SetCurrentReferenceMotor1(qd_t);
int MC_GetSTMStateMotor1(void);
uint16_t MC_GetCurrentFaultsMotor1(void); uint16_t MC_GetOccurredFaultsMotor1(void);
bool MC_AcknowledgeFaultMotor1(void);
int16_t MC_GetMecSpeedAverageMotor1(void);
int16_t MC_GetMecSpeedReferenceMotor1(void);
qd_t MC_GetIqdMotor1(void); qd_t MC_GetIqdrefMotor1(void);
