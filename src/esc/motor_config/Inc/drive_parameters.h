
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM           2976 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** Hall sensors ************/
#define HALL_AVERAGING_FIFO_DEPTH           6 /* one electrical turn: less lag at low speed */ /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */
#define HALL_MTPA                            false

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       16000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           LS_PWM_TIMER
#define SW_DEADTIME_NS                      750 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops.
 * From esc_tool.py capture on all four motors (2026-10-16, 12.0 V): dq
 * R 0.30-0.32 Ohm, L 135-150 uH. Pole-zero cancellation at ~1 kHz
 * (Kp = L*wc, Ki = R*wc): 1 -> 3 A step rises in 188-250 us, ~2 %
 * overshoot, settled in ~0.4 ms. The Workbench values (3391/3343, for a
 * generic 0.22 Ohm / 110 uH motor) rose in 250-375 us. Tunable over CAN:
 * CONFIG_SET current_kp / current_ki. */
#define PID_TORQUE_KP_DEFAULT               4000
#define PID_TORQUE_KI_DEFAULT               4400
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 4000
#define PID_FLUX_KI_DEFAULT                 4400
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            512
#define TF_KIDIV                            4096
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((512))
#define TF_KIDIV_LOG                        LOG2((4096))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

/* Speed PI in current counts (544/A) per RPM of error (SPEED_UNIT = U_RPM).
 * Kp 1024/256 = 4 counts/RPM: a 100 RPM error asks for ~0.75 A.
 * Ki 553/16384 per 1 ms: a held 30 RPM error adds ~1.9 A/s.
 * Provisional, wheels raised (2026-10-16): 4096/4424 limit-cycled at +-6 A
 * (+-300 RPM at a 60 RPM command); these hold the command with Iq ~0.1 A.
 * Retune after the current loop and on the ground. Tunable over CAN
 * CONFIG_SET without reflashing. Positive: +Iq accelerates in +Hall
 * direction once HALL_PHASE_SHIFT is measured (CAN HALL_ALIGN). */
#define PID_SPEED_KP_DEFAULT                1024
#define PID_SPEED_KI_DEFAULT                553
#define PID_SPEED_KD_DEFAULT                0

/* Friction feed-forward (drive_app.h), mA. Off until tuned on the ground:
 * breakaway there takes 2.2-3.2 A per wheel, which the PI alone only reaches
 * by integrator windup (slow start, then overshoot). CAN CONFIG_SET keys
 * FF_STATIC_MA / FF_KINETIC_MA. */
#define FRICTION_FF_STATIC_MA_DEFAULT       0
#define FRICTION_FF_KINETIC_MA_DEFAULT      0

/* Startup speed loop (drive_speed_ctrl.h, notes/ff-design.md stage 1). Off by
 * default: enable and tune over CAN CONFIG_SET (ESC_CFG_DSC_*). Running ff
 * 600 mA ~ the carpet steady current at 50 RPM (0.4-0.9 A); the start ramp
 * 0.5 -> 3 A at 16 A/s crosses the 1.7-2.3 A carpet breakaway in ~80-110 ms.
 * Hold decay 8 mA/ms: a 1.5 A hand-off hold is gone in ~190 ms. */
#define DSC_ENABLE_DEFAULT                  0
#define DSC_FF_RUN_MA_DEFAULT               600
#define DSC_FF_PER_KRPM_MA_DEFAULT          0
#define DSC_START_FROM_MA_DEFAULT           500
#define DSC_START_CAP_MA_DEFAULT            3000
#define DSC_START_SLOPE_MA_DEFAULT          16
#define DSC_START_HOLD_MS_DEFAULT           150
#define DSC_PCAP_MA_DEFAULT                 500
#define DSC_PCAP_MS_DEFAULT                 50
#define DSC_PCAP_UPDATES_DEFAULT            2
#define DSC_PCAP_FALLBACK_MS_DEFAULT        150
#define DSC_HOLD_DECAY_MA_DEFAULT           8
/* Mat 2026-10-18 (M1 log): wind-up gives one edge per ~0.5-1 A; after a real
 * breakaway the next edge came 45-50 ms later at the slow legacy ramp. */
#define DSC_WATCH_MS_DEFAULT                40
#define DSC_PRELOAD_PCT_DEFAULT             70

/* Hall reversal filter (drive_speed_ctrl.h): a stuck wheel rocking across one
 * Hall edge reads +-200 RPM and the speed loop answers with +-6 A (FR stall
 * test 2026-10-17). On by default; CONFIG ESC_CFG_SPEED_REV_FILTER. */
#define SPEED_REV_FILTER_DEFAULT            1
/* Speed fed to either loop below DSC_EDGE_SPEED_MAX_RPM, or while the SDK
 * speed is still 0, from Hall edge timing (dsc_edge_rpm). Mat 2026-10-18: the
 * wheels creep at 7-18 RPM from ~1.2 A while the SDK speed reads 0, so both
 * loops kept raising current and each wheel lurched when its SDK estimate came
 * alive. CONFIG ESC_CFG_SPEED_EDGE_EST. */
#define SPEED_EDGE_EST_DEFAULT              0

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            256
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((256))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             6

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            0
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              18 /*!< Over-voltage threshold: 3S pack plus braking headroom */
#define UD_VOLTAGE_THRESHOLD_V              8 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
