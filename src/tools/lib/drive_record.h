/** @file drive_record.h
 * @brief Read all four speed-loop recorders and write decoded samples and summaries.
 */
#ifndef DRIVE_RECORD_H
#define DRIVE_RECORD_H
#include "drive_ops.h"
typedef struct {
  bool valid[4];
  char errors[4][256];
  drive_record_summary_t wheels[4];
} drive_records_t;
/** @brief Read each wheel independently; failed reads are preserved in the summary. */
int drive_records_read(ik_link_t *link, const char *path, double t0_monotonic_s, drive_records_t *out);
/** @brief Write a wheel-keyed summary (samples, trigger, peak RPM); failed reads as their error string. */
void drive_records_json(FILE *f, const drive_records_t *records);
#endif
