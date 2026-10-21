#include "esc_defaults.h"

#include <string.h>

#include "gen_esc_defaults.h"

static const struct { const char *key; int value; } DEFAULTS[] = {
    {"kp", ESC_PID_SPEED_KP_DEFAULT}, {"ki", ESC_PID_SPEED_KI_DEFAULT},
    {"ff_static_ma", ESC_FRICTION_FF_STATIC_MA_DEFAULT}, {"ff_kinetic_ma", ESC_FRICTION_FF_KINETIC_MA_DEFAULT},
    {"speed_rev_filter", ESC_SPEED_REV_FILTER_DEFAULT}, {"speed_edge_est", ESC_SPEED_EDGE_EST_DEFAULT},
    {"dsc_enable", ESC_DSC_ENABLE_DEFAULT}, {"dsc_ff_run_ma", ESC_DSC_FF_RUN_MA_DEFAULT},
    {"dsc_ff_per_krpm_ma", ESC_DSC_FF_PER_KRPM_MA_DEFAULT}, {"dsc_start_from_ma", ESC_DSC_START_FROM_MA_DEFAULT},
    {"dsc_start_cap_ma", ESC_DSC_START_CAP_MA_DEFAULT}, {"dsc_start_slope_ma", ESC_DSC_START_SLOPE_MA_DEFAULT},
    {"dsc_start_hold_ms", ESC_DSC_START_HOLD_MS_DEFAULT}, {"dsc_pcap_ma", ESC_DSC_PCAP_MA_DEFAULT},
    {"dsc_pcap_ms", ESC_DSC_PCAP_MS_DEFAULT}, {"dsc_pcap_updates", ESC_DSC_PCAP_UPDATES_DEFAULT},
    {"dsc_pcap_fallback_ms", ESC_DSC_PCAP_FALLBACK_MS_DEFAULT},
    {"dsc_hold_decay_ma", ESC_DSC_HOLD_DECAY_MA_DEFAULT}, {"dsc_watch_ms", ESC_DSC_WATCH_MS_DEFAULT},
    {"dsc_preload_pct", ESC_DSC_PRELOAD_PCT_DEFAULT}};

int esc_default(const char *key, int *value)
{
  for (unsigned i = 0; i < sizeof DEFAULTS / sizeof DEFAULTS[0]; i++)
    if (!strcmp(DEFAULTS[i].key, key)) {
      *value = DEFAULTS[i].value;
      return 0;
    }
  return -1;
}
