#include <gtest/gtest.h>

#include "rs_i2t.h"

#define SUCCESS_VALUE 0

static float fake_get_current_A_ret_val = 9.0f;

float fake_get_current_A(void)
{
    return fake_get_current_A_ret_val;
}

uint32_t fake_set_max_current_A(float max_current_A)
{
    return 0;
}

TEST(RsI2tTest, InitNullInstance)
{
    uint32_t ret_val = rs_i2t_init(NULL, NULL);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInstanceAlreadyInit)
{
    rs_i2t_instance_t test_instance = {0};
    test_instance.initialized = true;
    uint32_t ret_val = rs_i2t_init(&test_instance, NULL);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNullConfig)
{
    rs_i2t_instance_t test_instance = {0};
    uint32_t ret_val = rs_i2t_init(&test_instance, NULL);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInvalidPeriodMs)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInvalidNominalCurrent)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInvalidPeakCurrent)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 1.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInvalidDerateCurrent)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 1.0f;
    test_config.peak_current_A = 1.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInvalidThermalBudget)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 1.0f;
    test_config.peak_current_A = 1.0f;
    test_config.derate_current_A = 1.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNominalLessThanDerate)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 0.0f;
    test_config.peak_current_A = 1.0f;
    test_config.derate_current_A = 1.0f;
    test_config.thermal_budget_A2S = 30.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNominalGreaterThanPeak)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 1.0f;
    test_config.peak_current_A = 0.0f;
    test_config.derate_current_A = 1.0f;
    test_config.thermal_budget_A2S = 30.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNullGetCurrentA)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 3.26f;
    test_config.peak_current_A = 9.0f;
    test_config.derate_current_A = 3.0f;
    test_config.thermal_budget_A2S = 30.0f;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNullSetMaxCurrentA)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 3.26f;
    test_config.peak_current_A = 9.0f;
    test_config.derate_current_A = 3.0f;
    test_config.thermal_budget_A2S = 30.0f;
    test_config.get_current_A = fake_get_current_A;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, HappyPath)
{
    rs_i2t_instance_t test_instance = {0};
    rs_i2t_config_t test_config = {0};
    test_config.period_ms = 1.0f;
    test_config.nominal_current_A = 3.26f;
    test_config.peak_current_A = 9.0f;
    test_config.derate_current_A = 3.0f;
    test_config.thermal_budget_A2S = 30.0f;
    test_config.get_current_A = fake_get_current_A;
    test_config.set_max_current_A = fake_set_max_current_A;
    uint32_t ret_val = rs_i2t_init(&test_instance, &test_config);
    EXPECT_EQ(ret_val, SUCCESS_VALUE);}
