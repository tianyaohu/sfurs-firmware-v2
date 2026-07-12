#ifndef RS_I2T_H
#define RS_I2T_H

#include <stdint.h>

typedef rs_i2t_config_t
{
    float period;
    float nominal_current_A;
    float peak_current_A;
    float derate_current_A;
    float thermal_budget_A2S;
    float (*get_current_A) (void);
    void  (*set_max_current_A) (float max_current_A);
} rs_i2t_config_s;

typedef rs_i2t_instance_t
{
    bool initialized = false;
    rs_i2t_config_t* config = NULL;
} rs_i2t_instance_s;

uint32_t rs_i2t_init(rs_i2t_instance_t* instance, rs_i2t_config_t* config);

uint32_t rs_i2t_step(rs_i2t_config_t* rs_i2t_instance);

#endif
