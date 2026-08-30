# Build Configuration
When building, one of `MOTOR_NE`, `MOTOR_SE`, `MOTOR_SW`, or `MOTOR_NW` must be defined. This can be done by adding it to the `# Symbols` tab under the BUILD CONFIGURATIONS. 

# Project Layout

`motor_config.stwb6`
This file is used by the STM32 Motor Control Workbench application. This app should be used to configure all parameters for the motor controller. 

`motor_config/`
This is the generated STM32 CubeIDE project. Project specific configuration is handled in `motor_config.ioc`. This folder is the one you want to import into CubeIDE. Alternatively, you can import the entire `HW/` folder into CubeIDE and it will import all of the projects.

`Inc/motor_controller_can.h`
- has a guard that will fail the build if a motor direction is not defined 
- defines the motor controller's ID based on the motor direction we defined (these IDs are sourced from `can_datatypes.h`)

# Main Code Behaviour
## CAN Bus Terminators 
`MOTOR_NE` and `MOTOR_SE` are configured to set `GPIO_PIN_14` high which enables the 100 Ohm resistor. These two motors are configured as the bus terminations but can be changed. 
## I2T 
I2T is software thermal protection. It works by integrating the current squared (actually the difference of the current and nominal current squared) over time and derating the available peak current if I2T is over the limit. The calculations for I2T are done in the main loop and gated on a 1000 Hz timer interrupt. I2T will only update on the trip edge change. 
## CANFD Processing 
All CANFD processing is done in the main loop. There is already a hardware buffer configured so it is likely not necessary to use interrupt based CANFD processing. In the future, if an interrupt is configured the priority should be as low as possible (highest number). 

The filter is configured based on motor ID. More information can be found in `can_datatypes.h`.
## CANFD Parameters
Just use these parameters. If they ever need to be updated you can use https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/ 
```
hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
hfdcan1.Init.AutoRetransmission = ENABLE;
hfdcan1.Init.TransmitPause = DISABLE;
hfdcan1.Init.ProtocolException = DISABLE;
hfdcan1.Init.NominalPrescaler = 2;
hfdcan1.Init.NominalSyncJumpWidth = 17;
hfdcan1.Init.NominalTimeSeg1 = 152;
hfdcan1.Init.NominalTimeSeg2 = 17;
hfdcan1.Init.DataPrescaler = 17;
hfdcan1.Init.DataSyncJumpWidth = 9;
hfdcan1.Init.DataTimeSeg1 = 10;
hfdcan1.Init.DataTimeSeg2 = 9;
hfdcan1.Init.StdFiltersNbr = 1;
hfdcan1.Init.ExtFiltersNbr = 0;
```

# Troubleshooting
If the first build is running into missing or undefined references, try the following steps:
1. Re-checkout the `STM32CubeIDE/.project` and try to build again
2. Regenerate the project in motor controller workbench, then re-checkout `motor_config.ioc` and regenerate the code
