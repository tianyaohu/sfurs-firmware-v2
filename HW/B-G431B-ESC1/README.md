# Build Configuration
When building, one of `MOTOR_NE`, `MOTOR_SE`, `MOTOR_SW`, or `MOTOR_NW` must be defined. This can be done by adding it to the `# Symbols` tab under the BUILD CONFIGURATIONS. 

# Troubleshooting
If the first build is running into missing or undefined references, try the following steps:
1. Re-checkout the `STM32CubeIDE/.project` and try to build again
2. Regenerate the project in motor controller workbench, then re-checkout `motor_config.ioc` and regenerate the code
