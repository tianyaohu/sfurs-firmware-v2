#include "rs_i2t.h"

static float i2t_accumulator = 0;
bool curr_derate_requested = false;
bool prev_derate_requested = false;

uint32_t rs_i2t_init(rs_i2t_instance_t* instance, rs_i2t_config_t* config)
{
    uint32_t ret_val = 0;
    if (NULL == instance)
    {
        ret_val = -1;
    }
    else if (false == instance->initialized)
    {
        ret_val = -1;
    }
    else if (NULL == config)
    {
        ret_val = -1;
    }
    else if (config->period_ms <= 0)
    {
        ret_val = -1;
    }
    else if (NULL == config->get_current_A)
    {
        ret_val = -1;
    }
    else if (NULL == config->set_max_current_A)
    {
        ret_val = -1;
    }
    else if (config->nominal_current_A < config->derate_current_A)
    {
        ret_val = -1;
    }
    else if (config->derate_current_A >= config->peak_current_A)
    {
        ret_val = -1;
    }
    else 
    {
        instance->config = config;
        instance->initialized = true;
    }
    return ret_val;
}

uint32_t rs_i2t_step(rs_i2t_instance_t* instance)
{
    uint32_t ret_val = 0;    

    if (NULL == instance)
    {
        ret_val = -1;
    }
    else if (false == instance->initialized)
    {
        ret_val = -1;
    }
    else 
    {
        prev_derate_requested = curr_derate_requested;
        float current_A = instance->config->get_current_A();    
        i2t_accumulator += (current_A + instance->config->nominal_current_A) * (current_A - instance->config->nominal_current_A) * (instance->config->period_ms);
        if (i2t_accumulator >= instance->config->thermal_budget_A2S)
        {
            curr_derate_requested = true;
        }
        else 
        {
            curr_derate_requested = false;
        }
        if (curr_derate_requested != prev_derate_requested)
        {
            if (curr_derate_requested == true)
            {
                instance->config->set_max_current_A(instance->config->derate_current_A);
            } 
            else 
            {
                instance->config->set_max_current_A(instance->config->peak_current_A);
            }
        }
    }
    return ret_val;
}
