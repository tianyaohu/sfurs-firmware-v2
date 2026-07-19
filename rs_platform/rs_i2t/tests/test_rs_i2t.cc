#include <gtest/gtest.h>

#include "rs_i2t.h"

#define SUCCESS_VALUE 0

TEST(RsI2tTest, InitNullInstance)
{
    uint32_t ret_val = rs_i2t_init(NULL, NULL);
    ASSERT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitInstanceAlreadyInit)
{
    rs_i2t_instance_t test_instance;
    test_instance.initialized = true;
    uint32_t ret_val = rs_i2t_init(&test_instance, NULL);
    ASSERT_NE(ret_val, SUCCESS_VALUE);
}

TEST(RsI2tTest, InitNullConfig)
{
    rs_i2t_instance_t test_instance;
    uint32_t ret_val = rs_i2t_init(&test_instance, NULL);
    ASSERT_NE(ret_val, SUCCESS_VALUE);
}
