/**
 * @file motor_controller_can.h
 * @brief Maps the MOTOR_NE/SE/SW/NW build macro to the CAN select bit MOTOR_CAN_ID.
 */

#ifndef MOTOR_CONTROLLER_CAN_H
#define MOTOR_CONTROLLER_CAN_H

#include "can_datatypes.h"

#if (defined(MOTOR_NE) && defined(MOTOR_SE)) || \
    (defined(MOTOR_NE) && defined(MOTOR_SW)) || \
    (defined(MOTOR_NE) && defined(MOTOR_NW)) || \
    (defined(MOTOR_SE) && defined(MOTOR_SW)) || \
    (defined(MOTOR_SE) && defined(MOTOR_NW)) || \
    (defined(MOTOR_SW) && defined(MOTOR_NW))
#error "Define only one motor macro: MOTOR_NE, MOTOR_SE, MOTOR_SW, or MOTOR_NW"
#endif

#if defined(MOTOR_NE)
  #define MOTOR_CAN_ID MOTOR_NE_CAN_ID
#elif defined(MOTOR_SE)
  #define MOTOR_CAN_ID MOTOR_SE_CAN_ID
#elif defined(MOTOR_SW)
  #define MOTOR_CAN_ID MOTOR_SW_CAN_ID
#elif defined(MOTOR_NW)
  #define MOTOR_CAN_ID MOTOR_NW_CAN_ID
#else
  #define MOTOR_CAN_ID 0U
#endif
/** @def MOTOR_CAN_ID
 *  @brief This ESC's CAN select bit; 0 if no wheel macro is defined. */


#endif
