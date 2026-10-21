/* Every CONFIG_SET startup-loop key has a boot default, and the generated
 * values are the ones in drive_parameters.h. Ports test_defaults_cover_every_dsc_key. */
#include <string.h>

#include "check.h"
#include "esc_defaults.h"
#include "ik_link.h"

int main(void)
{
  int value, number;
  size_t dsc = 0;
  for (size_t i = 0; i < ik_config_key_count(); i++) {
    const char *key = ik_config_key_name(i, &number);
    if (strncmp(key, "dsc_", 4)) continue;
    dsc++;
    CHECK(esc_default(key, &value) == 0);
  }
  CHECK_EQ(dsc, 14);
  CHECK_EQ(esc_default("dsc_enable", &value), 0);
  CHECK_EQ(value, 0);
  CHECK_EQ(esc_default("kp", &value), 0);
  CHECK_EQ(value, 1024); /* PID_SPEED_KP_DEFAULT */
  CHECK(esc_default("current_kp", &value) != 0); /* not RAM-restored by the tools */
  printf("ok\n");
  return 0;
}
