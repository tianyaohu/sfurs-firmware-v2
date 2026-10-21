#include "tune_analysis.h"
#include "check.h"

#include <string.h>

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define MAX_LOG 512
#define MAX_STARTS 16

static const tune_limits LIMITS = {0.08, 0.03, 1.3, 3, 2, 0.05};

/* Extracted once with jq from tests/golden/tune_ground.json. */
static const tune_log_entry golden_log[] = {
  {0, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.01, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.02, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.03, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.04, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.05, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.06, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.07, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.08, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.09, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.1, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.11, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.12, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.13, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.14, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.15, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.16, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.17, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.18, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.19, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.2, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.21, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.22, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.23, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.24, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.25, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.26, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.27, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.28, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.29, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.3, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.31, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.32, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.33, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.34, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.35, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.36, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.37, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.38, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.39, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.4, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.41, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.42, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.43, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.44, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.45, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.46, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.47, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.48, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.49, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {0.5, {50, 48, -48, -50}, {0, 0, 0, 0}, {8.881784197001252e-16, 8.881784197001252e-16, -8.881784197001252e-16, -8.881784197001252e-16}, {true, true, true, true}, {true, true, true, true}},
  {0.51, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.040000000000000924, 0.040000000000000924, -0.040000000000000924, -0.040000000000000924}, {true, true, true, true}, {true, true, true, true}},
  {0.52, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.08000000000000096, 0.08000000000000096, -0.08000000000000096, -0.08000000000000096}, {true, true, true, true}, {true, true, true, true}},
  {0.53, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.120000000000001, 0.120000000000001, -0.120000000000001, -0.120000000000001}, {true, true, true, true}, {true, true, true, true}},
  {0.54, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.16000000000000103, 0.16000000000000103, -0.16000000000000103, -0.16000000000000103}, {true, true, true, true}, {true, true, true, true}},
  {0.55, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.20000000000000107, 0.20000000000000107, -0.20000000000000107, -0.20000000000000107}, {true, true, true, true}, {true, true, true, true}},
  {0.56, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.2400000000000011, 0.2400000000000011, -0.2400000000000011, -0.2400000000000011}, {true, true, true, true}, {true, true, true, true}},
  {0.57, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.28000000000000114, 0.28000000000000114, -0.28000000000000114, -0.28000000000000114}, {true, true, true, true}, {true, true, true, true}},
  {0.58, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.3200000000000012, 0.3200000000000012, -0.3200000000000012, -0.3200000000000012}, {true, true, true, true}, {true, true, true, true}},
  {0.59, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.3600000000000012, 0.3600000000000012, -0.3600000000000012, -0.3600000000000012}, {true, true, true, true}, {true, true, true, true}},
  {0.6, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.40000000000000124, 0.40000000000000124, -0.40000000000000124, -0.40000000000000124}, {true, true, true, true}, {true, true, true, true}},
  {0.61, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.4400000000000013, 0.4400000000000013, -0.4400000000000013, -0.4400000000000013}, {true, true, true, true}, {true, true, true, true}},
  {0.62, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.4800000000000013, 0.4800000000000013, -0.4800000000000013, -0.4800000000000013}, {true, true, true, true}, {true, true, true, true}},
  {0.63, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.5200000000000014, 0.5200000000000014, -0.5200000000000014, -0.5200000000000014}, {true, true, true, true}, {true, true, true, true}},
  {0.64, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.5600000000000014, 0.5600000000000014, -0.5600000000000014, -0.5600000000000014}, {true, true, true, true}, {true, true, true, true}},
  {0.65, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.6000000000000014, 0.6000000000000014, -0.6000000000000014, -0.6000000000000014}, {true, true, true, true}, {true, true, true, true}},
  {0.66, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.6400000000000015, 0.6400000000000015, -0.6400000000000015, -0.6400000000000015}, {true, true, true, true}, {true, true, true, true}},
  {0.67, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.6800000000000015, 0.6800000000000015, -0.6800000000000015, -0.6800000000000015}, {true, true, true, true}, {true, true, true, true}},
  {0.68, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.7200000000000015, 0.7200000000000015, -0.7200000000000015, -0.7200000000000015}, {true, true, true, true}, {true, true, true, true}},
  {0.69, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.7600000000000016, 0.7600000000000016, -0.7600000000000016, -0.7600000000000016}, {true, true, true, true}, {true, true, true, true}},
  {0.7, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.8000000000000016, 0.8000000000000016, -0.8000000000000016, -0.8000000000000016}, {true, true, true, true}, {true, true, true, true}},
  {0.71, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.8400000000000016, 0.8400000000000016, -0.8400000000000016, -0.8400000000000016}, {true, true, true, true}, {true, true, true, true}},
  {0.72, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.8800000000000017, 0.8800000000000017, -0.8800000000000017, -0.8800000000000017}, {true, true, true, true}, {true, true, true, true}},
  {0.73, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.9200000000000017, 0.9200000000000017, -0.9200000000000017, -0.9200000000000017}, {true, true, true, true}, {true, true, true, true}},
  {0.74, {50, 48, -48, -50}, {0, 0, 0, 0}, {0.9600000000000017, 0.9600000000000017, -0.9600000000000017, -0.9600000000000017}, {true, true, true, true}, {true, true, true, true}},
  {0.75, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.0000000000000018, -1.0000000000000018, -1.0000000000000018}, {true, true, true, true}, {true, true, true, true}},
  {0.76, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.0400000000000018, -1.0400000000000018, -1.0400000000000018}, {true, true, true, true}, {true, true, true, true}},
  {0.77, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.0800000000000018, -1.0800000000000018, -1.0800000000000018}, {true, true, true, true}, {true, true, true, true}},
  {0.78, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.1200000000000019, -1.1200000000000019, -1.1200000000000019}, {true, true, true, true}, {true, true, true, true}},
  {0.79, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.160000000000002, -1.160000000000002, -1.160000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.8, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.200000000000002, -1.200000000000002, -1.200000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.81, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.240000000000002, -1.240000000000002, -1.240000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.82, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.280000000000002, -1.280000000000002, -1.280000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.83, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.320000000000002, -1.320000000000002, -1.320000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.84, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.360000000000002, -1.360000000000002, -1.360000000000002}, {true, true, true, true}, {true, true, true, true}},
  {0.85, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.4000000000000021, -1.4000000000000021, -1.4000000000000021}, {true, true, true, true}, {true, true, true, true}},
  {0.86, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.4400000000000022, -1.4400000000000022, -1.4400000000000022}, {true, true, true, true}, {true, true, true, true}},
  {0.87, {50, 48, -48, -50}, {50, 0, 0, 0}, {0.3, 1.4800000000000022, -1.4800000000000022, -1.4800000000000022}, {true, true, true, true}, {true, true, true, true}},
  {0.88, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.5200000000000022, -1.5200000000000022}, {true, true, true, true}, {true, true, true, true}},
  {0.89, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.5600000000000023, -1.5600000000000023}, {true, true, true, true}, {true, true, true, true}},
  {0.9, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.6000000000000023, -1.6000000000000023}, {true, true, true, true}, {true, true, true, true}},
  {0.91, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.6400000000000023, -1.6400000000000023}, {true, true, true, true}, {true, true, true, true}},
  {0.92, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.6800000000000024, -1.6800000000000024}, {true, true, true, true}, {true, true, true, true}},
  {0.93, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.7200000000000024, -1.7200000000000024}, {true, true, true, true}, {true, true, true, true}},
  {0.94, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.7600000000000025, -1.7600000000000025}, {true, true, true, true}, {true, true, true, true}},
  {0.95, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.8000000000000025, -1.8000000000000025}, {true, true, true, true}, {true, true, true, true}},
  {0.96, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.8400000000000025, -1.8400000000000025}, {true, true, true, true}, {true, true, true, true}},
  {0.97, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.8800000000000026, -1.8800000000000026}, {true, true, true, true}, {true, true, true, true}},
  {0.98, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.9200000000000026, -1.9200000000000026}, {true, true, true, true}, {true, true, true, true}},
  {0.99, {50, 48, -48, -50}, {50, 48, 0, 0}, {0.3, 0.4, -1.9600000000000026, -1.9600000000000026}, {true, true, true, true}, {true, true, true, true}},
  {1, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.0000000000000027}, {true, true, true, true}, {true, true, true, true}},
  {1.01, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.0400000000000027}, {true, true, true, true}, {true, true, true, true}},
  {1.02, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.0800000000000027}, {true, true, true, true}, {true, true, true, true}},
  {1.03, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.1200000000000028}, {true, true, true, true}, {true, true, true, true}},
  {1.04, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.160000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.05, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.200000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.06, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.240000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.07, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.280000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.08, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.320000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.09, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.360000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.1, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.400000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.11, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.440000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.12, {50, 48, -48, -50}, {50, 48, -48, 0}, {0.3, 0.4, -0.5, -2.480000000000003}, {true, true, true, true}, {true, true, true, true}},
  {1.13, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.14, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.15, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.16, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.17, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.18, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.19, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.2, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.21, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.22, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.23, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.24, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.25, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.26, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.27, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.28, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.29, {50, 48, -48, -50}, {50, 48, -48, -50}, {0.3, 0.4, -0.5, -0.6}, {true, true, true, true}, {true, true, true, true}},
  {1.3, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.31, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.32, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.33, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.34, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.35, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.36, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.37, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.38, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.39, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.4, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.41, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.42, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.43, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.44, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.45, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.46, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.47, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.48, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.49, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.5, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.51, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.52, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.53, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.54, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.55, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.56, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.57, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.58, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.59, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.6, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.61, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.62, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.63, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.64, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.65, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.66, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.67, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.68, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.69, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.7, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.71, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.72, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.73, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.74, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.75, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.76, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.77, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.78, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.79, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.8, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.81, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.82, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.83, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.84, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.85, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.86, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.87, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.88, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.89, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.9, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.91, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.92, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.93, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.94, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.95, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.96, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.97, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.98, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {1.99, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.01, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.02, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.03, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.04, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.05, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.06, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.07, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.08, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.09, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.1, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.11, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.12, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.13, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.14, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.15, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.16, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.17, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.18, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.19, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.2, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.21, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.22, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.23, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.24, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.25, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.26, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.27, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.28, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.29, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {2.3, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-8.881784197001252e-16, -8.881784197001252e-16, 8.881784197001252e-16, 8.881784197001252e-16}, {true, true, true, true}, {true, true, true, true}},
  {2.31, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.040000000000000924, -0.040000000000000924, 0.040000000000000924, 0.040000000000000924}, {true, true, true, true}, {true, true, true, true}},
  {2.32, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.08000000000000096, -0.08000000000000096, 0.08000000000000096, 0.08000000000000096}, {true, true, true, true}, {true, true, true, true}},
  {2.33, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.120000000000001, -0.120000000000001, 0.120000000000001, 0.120000000000001}, {true, true, true, true}, {true, true, true, true}},
  {2.34, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.16000000000000103, -0.16000000000000103, 0.16000000000000103, 0.16000000000000103}, {true, true, true, true}, {true, true, true, true}},
  {2.35, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.20000000000000107, -0.20000000000000107, 0.20000000000000107, 0.20000000000000107}, {true, true, true, true}, {true, true, true, true}},
  {2.36, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.2400000000000011, -0.2400000000000011, 0.2400000000000011, 0.2400000000000011}, {true, true, true, true}, {true, true, true, true}},
  {2.37, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.28000000000000114, -0.28000000000000114, 0.28000000000000114, 0.28000000000000114}, {true, true, true, true}, {true, true, true, true}},
  {2.38, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.3200000000000012, -0.3200000000000012, 0.3200000000000012, 0.3200000000000012}, {true, true, true, true}, {true, true, true, true}},
  {2.39, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.3600000000000012, -0.3600000000000012, 0.3600000000000012, 0.3600000000000012}, {true, true, true, true}, {true, true, true, true}},
  {2.4, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.40000000000000124, -0.40000000000000124, 0.40000000000000124, 0.40000000000000124}, {true, true, true, true}, {true, true, true, true}},
  {2.41, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.4400000000000013, -0.4400000000000013, 0.4400000000000013, 0.4400000000000013}, {true, true, true, true}, {true, true, true, true}},
  {2.42, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.4800000000000013, -0.4800000000000013, 0.4800000000000013, 0.4800000000000013}, {true, true, true, true}, {true, true, true, true}},
  {2.43, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.5200000000000014, -0.5200000000000014, 0.5200000000000014, 0.5200000000000014}, {true, true, true, true}, {true, true, true, true}},
  {2.44, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.5600000000000014, -0.5600000000000014, 0.5600000000000014, 0.5600000000000014}, {true, true, true, true}, {true, true, true, true}},
  {2.45, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.6000000000000014, -0.6000000000000014, 0.6000000000000014, 0.6000000000000014}, {true, true, true, true}, {true, true, true, true}},
  {2.46, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.6400000000000015, -0.6400000000000015, 0.6400000000000015, 0.6400000000000015}, {true, true, true, true}, {true, true, true, true}},
  {2.47, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.6800000000000015, -0.6800000000000015, 0.6800000000000015, 0.6800000000000015}, {true, true, true, true}, {true, true, true, true}},
  {2.48, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.7200000000000015, -0.7200000000000015, 0.7200000000000015, 0.7200000000000015}, {true, true, true, true}, {true, true, true, true}},
  {2.49, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.7600000000000016, -0.7600000000000016, 0.7600000000000016, 0.7600000000000016}, {true, true, true, true}, {true, true, true, true}},
  {2.5, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.8000000000000016, -0.8000000000000016, 0.8000000000000016, 0.8000000000000016}, {true, true, true, true}, {true, true, true, true}},
  {2.51, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.8400000000000016, -0.8400000000000016, 0.8400000000000016, 0.8400000000000016}, {true, true, true, true}, {true, true, true, true}},
  {2.52, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.8800000000000017, -0.8800000000000017, 0.8800000000000017, 0.8800000000000017}, {true, true, true, true}, {true, true, true, true}},
  {2.53, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.9200000000000017, -0.9200000000000017, 0.9200000000000017, 0.9200000000000017}, {true, true, true, true}, {true, true, true, true}},
  {2.54, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-0.9600000000000017, -0.9600000000000017, 0.9600000000000017, 0.9600000000000017}, {true, true, true, true}, {true, true, true, true}},
  {2.55, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-1.0000000000000018, -1.0000000000000018, 1.0000000000000018, 1.0000000000000018}, {true, true, true, true}, {true, true, true, true}},
  {2.56, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-1.0400000000000018, -1.0400000000000018, 1.0400000000000018, 1.0400000000000018}, {true, true, true, true}, {true, true, true, true}},
  {2.57, {-50, -48, 48, 50}, {0, 0, 0, 0}, {-1.0800000000000018, -1.0800000000000018, 1.0800000000000018, 1.0800000000000018}, {true, true, true, true}, {true, true, true, true}},
  {2.58, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.1200000000000019, 1.1200000000000019, 1.1200000000000019}, {true, true, true, true}, {true, true, true, true}},
  {2.59, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.160000000000002, 1.160000000000002, 1.160000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.6, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.200000000000002, 1.200000000000002, 1.200000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.61, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.240000000000002, 1.240000000000002, 1.240000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.62, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.280000000000002, 1.280000000000002, 1.280000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.63, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.320000000000002, 1.320000000000002, 1.320000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.64, {-50, -48, 48, 50}, {-60, 0, 0, 0}, {-0.3, -1.360000000000002, 1.360000000000002, 1.360000000000002}, {true, true, true, true}, {true, true, true, true}},
  {2.65, {-50, -48, 48, 50}, {-60, -55, 0, 0}, {-0.3, -0.4, 1.4000000000000021, 1.4000000000000021}, {true, true, true, true}, {true, true, true, true}},
  {2.66, {-50, -48, 48, 50}, {-60, -55, 0, 0}, {-0.3, -0.4, 1.4400000000000022, 1.4400000000000022}, {true, true, true, true}, {true, true, true, true}},
  {2.67, {-50, -48, 48, 50}, {-60, -55, 0, 0}, {-0.3, -0.4, 1.4800000000000022, 1.4800000000000022}, {true, true, true, true}, {true, true, true, true}},
  {2.68, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.5200000000000022, 1.5200000000000022}, {true, true, true, true}, {true, true, true, true}},
  {2.69, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.5600000000000023, 1.5600000000000023}, {true, true, true, true}, {true, true, true, true}},
  {2.7, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.6000000000000023, 1.6000000000000023}, {true, true, true, true}, {true, true, true, true}},
  {2.71, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.6400000000000023, 1.6400000000000023}, {true, true, true, true}, {true, true, true, true}},
  {2.72, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.6800000000000024, 1.6800000000000024}, {true, true, true, true}, {true, true, true, true}},
  {2.73, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.7200000000000024, 1.7200000000000024}, {true, true, true, true}, {true, true, true, true}},
  {2.74, {-50, -48, 48, 50}, {-50, -55, 0, 0}, {-0.3, -0.4, 1.7600000000000025, 1.7600000000000025}, {true, true, true, true}, {true, true, true, true}},
  {2.75, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 1.8000000000000025, 1.8000000000000025}, {true, true, true, true}, {true, true, true, true}},
  {2.76, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 1.8400000000000025, 1.8400000000000025}, {true, true, true, true}, {true, true, true, true}},
  {2.77, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 1.8800000000000026, 1.8800000000000026}, {true, true, true, true}, {true, true, true, true}},
  {2.78, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 1.9200000000000026, 1.9200000000000026}, {true, true, true, true}, {true, true, true, true}},
  {2.79, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 1.9600000000000026, 1.9600000000000026}, {true, true, true, true}, {true, true, true, true}},
  {2.8, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 2.0000000000000027, 2.0000000000000027}, {true, true, true, true}, {true, true, true, true}},
  {2.81, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 2.0400000000000027, 2.0400000000000027}, {true, true, true, true}, {true, true, true, true}},
  {2.82, {-50, -48, 48, 50}, {-50, -48, 0, 0}, {-0.3, -0.4, 2.0800000000000027, 2.0800000000000027}, {true, true, true, true}, {true, true, true, true}},
  {2.83, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.1200000000000028}, {true, true, true, true}, {true, true, true, true}},
  {2.84, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.160000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.85, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.200000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.86, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.240000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.87, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.280000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.88, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.320000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.89, {-50, -48, 48, 50}, {-50, -48, 55, 0}, {-0.3, -0.4, 0.5, 2.360000000000003}, {true, true, true, true}, {true, true, true, true}},
  {2.9, {-50, -48, 48, 50}, {-50, -48, 55, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.91, {-50, -48, 48, 50}, {-50, -48, 55, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.92, {-50, -48, 48, 50}, {-50, -48, 55, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.93, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.94, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.95, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.96, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.97, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.98, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {2.99, {-50, -48, 48, 50}, {-50, -48, 48, 70}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.01, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.02, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.03, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.04, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.05, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.06, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.07, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.08, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.09, {-50, -48, 48, 50}, {-50, -48, 48, 50}, {-0.3, -0.4, 0.5, 0.6}, {true, true, true, true}, {true, true, true, true}},
  {3.1, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.11, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.12, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.13, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.14, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.15, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.16, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.17, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.18, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.19, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.2, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.21, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.22, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.23, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.24, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.25, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.26, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.27, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.28, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.29, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.3, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.31, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.32, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.33, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.34, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.35, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.36, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.37, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.38, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.39, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.4, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.41, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.42, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.43, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.44, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.45, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.46, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.47, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.48, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.49, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.5, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.51, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.52, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.53, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.54, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.55, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.56, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.57, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.58, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
  {3.59, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {true, true, true, true}, {true, true, true, true}},
};
static const double golden_friction[4][2][2] = {
  {{0.9600000000000017, 1.0800000000000018}, {0.3, 0.3}},
  {{1.4800000000000022, 1.360000000000002}, {0.4, 0.4}},
  {{1.9600000000000026, 2.0800000000000027}, {0.5, 0.5}},
  {{2.480000000000003, 2.360000000000003}, {0.6, 0.6}},
};
static const double golden_overshoot[4][2] = {
  {1, 1.2},
  {1, 1.1458333333333333},
  {1, 1.1458333333333333},
  {0, 0},
};
static const tune_response_result golden_response = {
  .wheels = {
    {2, 2, 0.2650000000000001, 0.28000000000000025, 1.1, 1.2},
    {2, 2, 0.36500000000000005, 0.38, 1.0729166666666665, 1.1458333333333333},
    {2, 2, 0.5150000000000001, 0.5300000000000002, 1.0729166666666665, 1.1458333333333333},
    {2, 2, 0.615, 0.6299999999999999, 0, 0},
  },
  .spread_s = 0.34999999999999987, .spread_worst_s = 0.3799999999999999
};

static void relative(double got, double want) {
  if (want == 0)
    CHECK(got == 0);
  else
    CHECK_NEAR(got, want, fabs(want) * 1e-9);
}

static double decimal_round(double x, int digits) {
  char text[64];
  snprintf(text, sizeof(text), "%.*f", digits, x);
  return strtod(text, NULL);
}

static double round_even(double x) {
  double lo = floor(x), part = x - lo;
  return lo + (part > 0.5 || (part == 0.5 && fmod(lo, 2) != 0));
}

static size_t start_log(tune_log_entry *log, const double breakaway[4], const double kinetic[4],
                        const double target[4], const double *peak_rpm) {
  double moved_at[4] = {NAN, NAN, NAN, NAN};
  size_t n = 0;
  for (double t = 0; t < 0.5 + 0.8 + 0.5; t += 0.01) {
    CHECK(n < MAX_LOG);
    bool active = t >= 0.5 && t < 0.5 + 0.8;
    tune_log_entry e = {.t = decimal_round(t, 4)};
    for (size_t w = 0; w < 4; ++w) {
      e.rpm_valid[w] = e.iq_ref_valid[w] = true;
      e.target_rpm[w] = active ? target[w] : 0;
      if (!active)
        continue;
      double sign = target[w] > 0 ? 1 : -1;
      double ramp = (t - 0.5) * 4;
      if (isnan(moved_at[w]) && ramp >= breakaway[w])
        moved_at[w] = t;
      if (isnan(moved_at[w])) {
        e.iq_ref_a[w] = sign * ramp;
      } else {
        double dt = t - moved_at[w];
        double peak = peak_rpm ? peak_rpm[w] : fabs(target[w]);
        e.rpm[w] = sign * round_even(dt < 0.1 ? peak : fabs(target[w]));
        e.iq_ref_a[w] = sign * kinetic[w];
      }
    }
    log[n++] = e;
  }
  return n;
}

static size_t joined(tune_log_entry *out, const tune_log_entry *a, size_t na,
                      const tune_log_entry *b, size_t nb) {
  CHECK(na + nb <= MAX_LOG);
  for (size_t k = 0; k < na; ++k) {
    out[k] = a[k];
    out[k].t = decimal_round(a[k].t, 4);
  }
  double offset = out[na - 1].t + 0.01;
  for (size_t k = 0; k < nb; ++k) {
    out[na + k] = b[k];
    out[na + k].t = decimal_round(b[k].t + offset, 4);
  }
  return na + nb;
}

static const double TARGET[4] = {50, 48, -48, -50};
static const double REVERSE[4] = {-50, -48, 48, 50};
static const double KINETIC[4] = {0.3, 0.4, 0.5, 0.6};
static const double SAME_KINETIC[4] = {0.3, 0.3, 0.3, 0.3};

static void init_friction(tune_friction f[4], double storage[4][2][MAX_STARTS]) {
  for (size_t w = 0; w < 4; ++w) {
    f[w].breakaway = (tune_values){storage[w][0], MAX_STARTS, 0};
    f[w].kinetic = (tune_values){storage[w][1], MAX_STARTS, 0};
  }
}

static void test_breakaway_and_kinetic_per_wheel(void) {
  tune_log_entry a[MAX_LOG], b[MAX_LOG], log[MAX_LOG];
  size_t na = start_log(a, (double[]){1, 1.5, 2, 2.5}, KINETIC, TARGET, NULL);
  size_t nb = start_log(b, (double[]){1.1, 1.4, 2.1, 2.4}, KINETIC, REVERSE, NULL);
  size_t n = joined(log, a, na, b, nb);
  double storage[4][2][MAX_STARTS];
  tune_friction f[4];
  init_friction(f, storage);
  CHECK_EQ(tune_friction_samples(log, n, 40, 0.3, f), TUNE_OK);
  const double want[4] = {1.05, 1.45, 2.05, 2.45};
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(f[w].breakaway.count, 2);
    CHECK_NEAR((f[w].breakaway.data[0] + f[w].breakaway.data[1]) / 2, want[w], 0.06);
    CHECK_NEAR(f[w].kinetic.data[0], KINETIC[w], 0.0005);
  }
}

static void test_wheel_that_never_moves_gives_no_breakaway(void) {
  tune_log_entry log[MAX_LOG];
  size_t n = start_log(log, (double[]){1, 9, 1, 1}, SAME_KINETIC, TARGET, NULL);
  double storage[4][2][MAX_STARTS];
  tune_friction f[4];
  init_friction(f, storage);
  CHECK_EQ(tune_friction_samples(log, n, 40, 0.3, f), TUNE_OK);
  CHECK_EQ(f[1].breakaway.count, 0);
  CHECK_EQ(f[0].breakaway.count, 1);
  CHECK_EQ(f[1].kinetic.count, 0);
}

static void test_spread(void) {
  CHECK_NEAR(tune_spread((double[]){1, 1, 1}, 3), 0, 1e-9);
  CHECK_NEAR(tune_spread((double[]){1, 2, 3}, 3), 0.5, 1e-9);
  CHECK(tune_spread((double[]){1}, 1) == INFINITY);
  CHECK(tune_spread(NULL, 0) == INFINITY);
  CHECK(tune_spread((double[]){-2, -1}, 2) == INFINITY);
  relative(tune_spread((double[]){1, 2, 3, 4}, 4), 0.4);
}

static void test_overshoot_ratio(void) {
  tune_log_entry log[MAX_LOG];
  size_t n = start_log(log, (double[]){0.2, 0.2, 0.2, 0.2}, SAME_KINETIC, TARGET,
                       (double[]){150, 48, 48, 50});
  double storage[4][MAX_STARTS];
  tune_values over[4];
  for (size_t w = 0; w < 4; ++w)
    over[w] = (tune_values){storage[w], MAX_STARTS, 0};
  CHECK_EQ(tune_overshoot(log, n, 40, 0.6, over), TUNE_OK);
  CHECK_NEAR(over[0].data[0], 3, 1e-9);
  CHECK_NEAR(over[1].data[0], 1, 1e-9);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(over[w].count, 1);
    CHECK_NEAR(over[w].data[0], w == 0 ? 3 : 1, 1e-9);
  }
}

static tune_response_result simulated_response(double breakaway, const double *peak) {
  tune_log_entry log[MAX_LOG];
  double ba[4] = {breakaway, breakaway, breakaway, breakaway};
  size_t n = start_log(log, ba, SAME_KINETIC, TARGET, peak);
  tune_response_result r;
  CHECK_EQ(tune_response(log, n, &r), TUNE_OK);
  return r;
}

static double next_fraction(double frac, bool slow, bool over) {
  return tune_next_fraction(frac, slow, over, 0.15, 0.1, 0.2, 1);
}

static void test_judge_and_next_fraction(void) {
  tune_response_result fast_over = simulated_response(0.1, (double[]){150, 144, 144, 150});
  tune_judgement j = tune_judge(&fast_over, &LIMITS, 0);
  CHECK(!j.slow && j.over);
  CHECK_NEAR(next_fraction(0.8, j.slow, j.over), 0.65, 1e-12);
  tune_response_result slow_ok = simulated_response(2, NULL);
  j = tune_judge(&slow_ok, &LIMITS, 0);
  CHECK(j.slow && !j.over);
  CHECK_NEAR(next_fraction(0.8, true, false), 0.9, 1e-12);
  tune_response_result good = simulated_response(0.1, NULL);
  j = tune_judge(&good, &LIMITS, 0);
  CHECK(!j.slow && !j.over);
  CHECK(isnan(next_fraction(0.8, false, false)));
  CHECK(isnan(next_fraction(1, true, false)));
  CHECK_NEAR(next_fraction(0.8, true, true), 0.65, 1e-12);
  CHECK(isnan(next_fraction(0.2, false, true)));
  CHECK_NEAR(next_fraction(0.25, false, true), 0.2, 1e-12);
}

static void test_baseline_overshoot_is_tolerated(void) {
  tune_response_result resp = simulated_response(0.1, (double[]){80, 77, 77, 80});
  CHECK(tune_judge(&resp, &LIMITS, 0).over);
  CHECK(!tune_judge(&resp, &LIMITS, 1.6).over);
}

static tune_move summary(double vx, double vy, double direction) {
  return (tune_move){.twist = {vx, vy, 0}, .seen = true, .distance_mm = 160,
                     .direction_deg = direction, .max_sideways_mm = 2,
                     .heading_change_deg = 0.5, .commanded_mm = 160};
}

static tune_accept_result accept(const tune_move *moves, size_t n) {
  tune_move_row rows[8];
  CHECK(n <= ARRAY_LEN(rows));
  tune_accept_result result;
  CHECK_EQ(tune_accept_moves(moves, n, &LIMITS, rows, ARRAY_LEN(rows), &result), TUNE_OK);
  return result;
}

static void test_mount_yaw_removed_and_pass(void) {
  tune_move moves[] = {summary(0, 200, 120), summary(0, -200, -59),
                       summary(200, 0, 29), summary(-200, 0, 210)};
  tune_accept_result a = accept(moves, ARRAY_LEN(moves));
  CHECK(a.ok);
  CHECK_NEAR(a.mount_yaw_deg, 30, 0.5);
  CHECK(a.worst.direction_error_deg <= 1.1);
  CHECK_EQ(a.count, 4);
  CHECK_EQ(a.missing, 0);
  relative(a.worst.sideways_per_m, 12.5);
  relative(a.worst.heading_change_deg, 0.5);
  relative(a.worst.ratio_spread, 0);
}

static void test_fails_on_curve_heading_and_scale_spread(void) {
  tune_move base[] = {summary(0, 200, 90), summary(0, -200, -90),
                      summary(200, 0, 0), summary(-200, 0, 180)};
  tune_move changed[4];
  memcpy(changed, base, sizeof(base));
  changed[2].direction_deg = 10;
  CHECK(!accept(changed, 4).ok);
  memcpy(changed, base, sizeof(base));
  changed[0].heading_change_deg = 5;
  CHECK(!accept(changed, 4).ok);
  memcpy(changed, base, sizeof(base));
  changed[1].distance_mm = 140;
  CHECK(!accept(changed, 4).ok);
}

static void test_missing_moves(void) {
  tune_move moves[3] = {summary(0, 200, 90), summary(0, -200, -90), summary(200, 0, 0)};
  moves[0].seen = false;
  tune_accept_result a = accept(moves, 1);
  CHECK(!a.ok);
  CHECK_EQ(a.missing, 1);
  CHECK_EQ(a.count, 0);
  CHECK(strcmp(a.reason, "1 of 1 moves not seen still at both ends (or under 20 mm)") == 0);
  moves[0].seen = true;
  moves[1].seen = moves[2].seen = false;
  CHECK(!accept(moves, 3).ok);
  a = accept(moves, 1);
  CHECK(!a.ok);
  CHECK_EQ(a.missing, 0);
  moves[1].seen = true;
  moves[1].distance_mm = 20;
  a = accept(moves, 2);
  CHECK(!a.ok);
  CHECK_EQ(a.missing, 1);
}

static void test_golden_data(void) {
  tune_log_entry a[MAX_LOG], b[MAX_LOG], log[MAX_LOG];
  size_t na = start_log(a, (double[]){1, 1.5, 2, 2.5}, KINETIC, TARGET, NULL);
  size_t nb = start_log(b, (double[]){1.1, 1.4, 2.1, 2.4}, KINETIC, REVERSE, (double[]){60, 55, 55, 70});
  size_t n = joined(log, a, na, b, nb);
  CHECK_EQ(n, ARRAY_LEN(golden_log));
  for (size_t k = 0; k < n; ++k) {
    relative(log[k].t, golden_log[k].t);
    for (size_t w = 0; w < 4; ++w) {
      CHECK_EQ(log[k].target_rpm[w], golden_log[k].target_rpm[w]);
      CHECK_EQ(log[k].rpm[w], golden_log[k].rpm[w]);
      relative(log[k].iq_ref_a[w], golden_log[k].iq_ref_a[w]);
    }
  }
  for (size_t source = 0; source < 2; ++source) {
    const tune_log_entry *input = source ? log : golden_log;
    tune_segment segments[2];
    size_t count;
    CHECK_EQ(tune_segments(input, n, 40, segments, 2, &count), TUNE_OK);
    CHECK_EQ(count, 2);
    CHECK_EQ(segments[0].first, 50);
    CHECK_EQ(segments[0].end, 130);
    CHECK_EQ(segments[1].first, 230);
    CHECK_EQ(segments[1].end, 310);
    double storage[4][2][MAX_STARTS];
    tune_friction f[4];
    init_friction(f, storage);
    CHECK_EQ(tune_friction_samples(input, n, 40, 0.3, f), TUNE_OK);
    double over_storage[4][MAX_STARTS];
    tune_values over[4];
    for (size_t w = 0; w < 4; ++w)
      over[w] = (tune_values){over_storage[w], MAX_STARTS, 0};
    CHECK_EQ(tune_overshoot(input, n, 40, 0.6, over), TUNE_OK);
    tune_response_result r;
    CHECK_EQ(tune_response(input, n, &r), TUNE_OK);
    relative(r.spread_s, golden_response.spread_s);
    relative(r.spread_worst_s, golden_response.spread_worst_s);
    for (size_t w = 0; w < 4; ++w) {
      CHECK_EQ(f[w].breakaway.count, 2);
      CHECK_EQ(f[w].kinetic.count, 2);
      CHECK_EQ(over[w].count, 2);
      for (size_t k = 0; k < 2; ++k) {
        relative(f[w].breakaway.data[k], golden_friction[w][0][k]);
        relative(f[w].kinetic.data[k], golden_friction[w][1][k]);
        relative(over[w].data[k], golden_overshoot[w][k]);
      }
      const tune_wheel_response *want = &golden_response.wheels[w], *got = &r.wheels[w];
      CHECK_EQ(got->moved, want->moved);
      CHECK_EQ(got->starts, want->starts);
      relative(got->breakaway_s, want->breakaway_s);
      relative(got->breakaway_worst_s, want->breakaway_worst_s);
      relative(got->overshoot, want->overshoot);
      relative(got->overshoot_worst, want->overshoot_worst);
    }
  }
}

static void test_segments_and_missing_samples(void) {
  tune_log_entry log[8] = {0};
  for (size_t k = 0; k < ARRAY_LEN(log); ++k) {
    log[k].t = k * 0.1;
    for (size_t w = 0; w < 4; ++w) {
      log[k].rpm_valid[w] = log[k].iq_ref_valid[w] = true;
      log[k].iq_ref_a[w] = 0.4;
    }
  }
  for (size_t w = 0; w < 4; ++w) {
    log[0].target_rpm[w] = 0.9;
    log[1].target_rpm[w] = 1;
    log[2].target_rpm[w] = 39;
    log[4].target_rpm[w] = 1;
    log[5].target_rpm[w] = 50;
    log[6].target_rpm[w] = -50;
    log[7].target_rpm[w] = 50;
    log[4].rpm[w] = -10;
    log[5].rpm[w] = 50;
    log[6].rpm[w] = -50;
    log[7].rpm[w] = 50;
    log[5].iq_ref_a[w] = 8;
    log[5].iq_ref_valid[w] = false;
    log[6].rpm_valid[w] = false;
  }
  tune_segment seg[2];
  size_t count;
  CHECK_EQ(tune_segments(log, 8, 40, seg, 2, &count), TUNE_OK);
  CHECK_EQ(count, 1);
  CHECK_EQ(seg[0].first, 4);
  CHECK_EQ(seg[0].end, 8);
  CHECK_EQ(tune_segments(log, 8, 1, seg, 2, &count), TUNE_OK);
  CHECK_EQ(count, 2);
  CHECK_EQ(seg[0].first, 1);
  CHECK_EQ(seg[0].end, 3);
  CHECK_EQ(tune_segments(log, 8, 1, NULL, 0, &count), TUNE_CAPACITY);
  CHECK_EQ(count, 2);
  double storage[4][2][MAX_STARTS];
  tune_friction f[4];
  init_friction(f, storage);
  CHECK_EQ(tune_friction_samples(log, 8, 40, 0.3, f), TUNE_OK);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(f[w].breakaway.count, 1);
    relative(f[w].breakaway.data[0], 0.4);
    CHECK_EQ(f[w].kinetic.count, 1);
    relative(f[w].kinetic.data[0], 0.4);
  }
  tune_values over[4];
  double os[4][2];
  for (size_t w = 0; w < 4; ++w) {
    over[w] = (tune_values){os[w], 2, 0};
    for (size_t k = 4; k < 8; ++k)
      log[k].rpm_valid[w] = false;
  }
  CHECK_EQ(tune_overshoot(log, 8, 40, 0.6, over), TUNE_OK);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(over[w].count, 1);
    relative(over[w].data[0], 0);
  }
}

static void test_response_windows_and_missing_metrics(void) {
  tune_response_result r;
  CHECK_EQ(tune_response(NULL, 0, &r), TUNE_OK);
  CHECK(isnan(r.spread_s));
  CHECK(isnan(r.spread_worst_s));
  CHECK(tune_judge(&r, &LIMITS, 0).slow);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(r.wheels[w].starts, 0);
    CHECK_EQ(r.wheels[w].moved, 0);
    CHECK(isnan(r.wheels[w].breakaway_s));
    CHECK(isnan(r.wheels[w].breakaway_worst_s));
    CHECK(isnan(r.wheels[w].overshoot));
    CHECK(isnan(r.wheels[w].overshoot_worst));
  }
  tune_log_entry log[MAX_LOG];
  size_t n = start_log(log, (double[]){0.1, 0.1, 0.1, 0.1}, SAME_KINETIC, TARGET, NULL);
  for (size_t k = 0; k < n; ++k)
    for (size_t w = 0; w < 4; ++w)
      log[k].rpm_valid[w] = false;
  CHECK_EQ(tune_response(log, n, &r), TUNE_OK);
  CHECK(tune_judge(&r, &LIMITS, 0).slow);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(r.wheels[w].starts, 1);
    CHECK_EQ(r.wheels[w].moved, 0);
    CHECK(isnan(r.wheels[w].breakaway_s));
    relative(r.wheels[w].overshoot, 0);
  }
  /* A second segment inside the one-second window belongs to both starts. */
  tune_log_entry pulses[4] = {0};
  for (size_t k = 0; k < 4; ++k) {
    pulses[k].t = k * 0.25;
    for (size_t w = 0; w < 4; ++w) {
      pulses[k].target_rpm[w] = k == 1 ? 0 : 50;
      pulses[k].rpm[w] = k == 3 ? 20 : 0;
      pulses[k].rpm_valid[w] = true;
    }
  }
  CHECK_EQ(tune_response(pulses, 4, &r), TUNE_OK);
  for (size_t w = 0; w < 4; ++w) {
    CHECK_EQ(r.wheels[w].starts, 2);
    CHECK_EQ(r.wheels[w].moved, 2);
    relative(r.wheels[w].breakaway_s, 0.5);
    relative(r.wheels[w].breakaway_worst_s, 0.75);
  }
}

static void test_wrap_and_camera_rows(void) {
  relative(tune_wrap_deg(180), -180);
  relative(tune_wrap_deg(-180), -180);
  relative(tune_wrap_deg(540), -180);
  relative(tune_wrap_deg(-541), 179);
  relative(tune_wrap_deg(721), 1);
  tune_move moves[2] = {summary(160, 0, 179), summary(-160, 0, 1)};
  moves[0].commanded_mm = 0;
  moves[1].commanded_mm = NAN;
  moves[0].max_sideways_mm = 100;
  moves[0].heading_change_deg = 1.125;
  moves[1].heading_change_deg = -1.125;
  tune_move_row rows[2];
  tune_accept_result a;
  CHECK_EQ(tune_accept_moves(moves, 2, &LIMITS, rows, 2, &a), TUNE_OK);
  CHECK(a.ok);
  CHECK_NEAR(fabs(a.mount_yaw_deg), 180, 1e-9);
  CHECK_EQ(a.count, 2);
  CHECK_EQ(a.reason[0], '\0');
  for (size_t k = 0; k < 2; ++k) {
    relative(rows[k].twist[0], moves[k].twist[0]);
    relative(rows[k].twist[1], moves[k].twist[1]);
    relative(rows[k].twist[2], moves[k].twist[2]);
    relative(rows[k].distance_mm, 160);
    relative(rows[k].ratio, 1);
    relative(fabs(rows[k].direction_error_deg), 1);
    relative(fabs(rows[k].heading_change_deg), 1.12);
  }
  relative(a.worst.sideways_per_m, 625);
  relative(a.worst.ratio_spread, 0);
  CHECK_EQ(tune_accept_moves(moves, 2, &LIMITS, rows, 1, &a), TUNE_CAPACITY);
}

static void test_buffer_errors(void) {
  tune_log_entry log[MAX_LOG];
  size_t n = start_log(log, (double[]){0.1, 0.1, 0.1, 0.1}, SAME_KINETIC, TARGET, NULL);
  tune_values over[4] = {0};
  CHECK_EQ(tune_overshoot(log, n, 40, 0.6, over), TUNE_CAPACITY);
  tune_friction f[4] = {0};
  CHECK_EQ(tune_friction_samples(log, n, 40, 0.3, f), TUNE_CAPACITY);
  CHECK_EQ(tune_friction_samples(NULL, 0, 40, 0.3, f), TUNE_OK);
  CHECK_EQ(tune_overshoot(NULL, 0, 40, 0.6, over), TUNE_OK);
  CHECK_EQ(tune_response(NULL, 1, &(tune_response_result){0}), TUNE_INVALID);
  CHECK_EQ(tune_segments(NULL, 1, 40, NULL, 0, &(size_t){0}), TUNE_INVALID);
  CHECK_EQ(tune_accept_moves(NULL, 1, &LIMITS, NULL, 0, &(tune_accept_result){0}), TUNE_INVALID);
}

int main(void) {
  test_breakaway_and_kinetic_per_wheel();
  test_wheel_that_never_moves_gives_no_breakaway();
  test_spread();
  test_overshoot_ratio();
  test_judge_and_next_fraction();
  test_baseline_overshoot_is_tolerated();
  test_mount_yaw_removed_and_pass();
  test_fails_on_curve_heading_and_scale_spread();
  test_missing_moves();
  test_golden_data();
  test_segments_and_missing_samples();
  test_response_windows_and_missing_metrics();
  test_wrap_and_camera_rows();
  test_buffer_errors();
  puts("tune analysis: 14 test cases passed");
  return 0;
}
