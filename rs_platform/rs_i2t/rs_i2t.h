#ifndef RS_I2T_H
#define RS_I2T_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct rs_i2t_config_s
{
    float period_ms;
    float nominal_current_A;
    float peak_current_A;
    float derate_current_A;
    float thermal_budget_A2S;
    float (*get_current_A) (void);
    void  (*set_max_current_A) (float max_current_A);
} rs_i2t_config_t;

typedef struct rs_i2t_instance_s
{
    bool initialized;
    rs_i2t_config_t* config;
} rs_i2t_instance_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t rs_i2t_init(rs_i2t_instance_t* instance, rs_i2t_config_t* config);

uint32_t rs_i2t_step(rs_i2t_instance_t* rs_i2t_instance);

#ifdef __cplusplus
}
#endif

#endif
