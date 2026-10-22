host_test(test_motor_speed
  SOURCES src/esc/motor_config/Src/motor_speed.c
  INCLUDES src/esc/motor_config/Inc src/shared/inc
  CASES p_term ki_step conditional_integration limit_same_tick stall_ramp bleed bleed_quantization
        reversal_clears ff_slew hold stall_timer estimator interpolation four_instances
        stall_ramp_limit_drop)
