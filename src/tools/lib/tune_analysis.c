#include "tune_analysis.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static bool moving(const tune_log_entry *e) {
  for (size_t w = 0; w < TUNE_WHEELS; ++w)
    if (fabs(e->target_rpm[w]) >= 1)
      return true;
  return false;
}

static bool next_segment(const tune_log_entry *log, size_t n, double min_target,
                         size_t *cursor, tune_segment *seg) {
  while (*cursor < n) {
    while (*cursor < n && !moving(&log[*cursor]))
      ++*cursor;
    size_t a = *cursor;
    double peak = 0;
    while (*cursor < n && moving(&log[*cursor])) {
      for (size_t w = 0; w < TUNE_WHEELS; ++w)
        peak = fmax(peak, fabs(log[*cursor].target_rpm[w]));
      ++*cursor;
    }
    if (a < *cursor && peak >= min_target) {
      *seg = (tune_segment){a, *cursor};
      return true;
    }
  }
  return false;
}

static double target_peak(const tune_log_entry *log, tune_segment seg, size_t w) {
  double target = log[seg.first].target_rpm[w];
  for (size_t k = seg.first + 1; k < seg.end; ++k)
    if (fabs(log[k].target_rpm[w]) > fabs(target))
      target = log[k].target_rpm[w];
  return target;
}

static int compare_double(const void *a, const void *b) {
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

static double median(double *values, size_t n) {
  if (!n)
    return NAN;
  qsort(values, n, sizeof(*values), compare_double);
  return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

static bool valid_values(const tune_values *v) {
  return v->data || !v->capacity;
}

static int append(tune_values *v, double x) {
  if (v->count >= v->capacity)
    return TUNE_CAPACITY;
  v->data[v->count++] = x;
  return TUNE_OK;
}

int tune_segments(const tune_log_entry *log, size_t n, double min_target,
                  tune_segment *out, size_t capacity, size_t *count) {
  if ((!log && n) || (!out && capacity) || !count)
    return TUNE_INVALID;
  *count = 0;
  size_t cursor = 0;
  tune_segment seg;
  while (next_segment(log, n, min_target, &cursor, &seg)) {
    if (*count < capacity)
      out[*count] = seg;
    ++*count;
  }
  return *count > capacity ? TUNE_CAPACITY : TUNE_OK;
}

int tune_friction_samples(const tune_log_entry *log, size_t n, double min_target,
                          double track_tol, tune_friction out[TUNE_WHEELS]) {
  if ((!log && n) || !out || n > SIZE_MAX / sizeof(double))
    return TUNE_INVALID;
  for (size_t w = 0; w < TUNE_WHEELS; ++w) {
    if (!valid_values(&out[w].breakaway) || !valid_values(&out[w].kinetic))
      return TUNE_INVALID;
    out[w].breakaway.count = out[w].kinetic.count = 0;
  }
  if (!n)
    return TUNE_OK;
  double *track = malloc(n * sizeof(*track));
  if (!track)
    return TUNE_MEMORY;
  size_t cursor = 0;
  tune_segment seg;
  int status = TUNE_OK;
  while (next_segment(log, n, min_target, &cursor, &seg)) {
    double half = log[seg.first].t + (log[seg.end - 1].t - log[seg.first].t) / 2;
    for (size_t w = 0; w < TUNE_WHEELS; ++w) {
      double target = target_peak(log, seg, w);
      if (fabs(target) < min_target)
        continue;
      double sign = target > 0 ? 1 : -1, peak = 0;
      for (size_t k = seg.first; k < seg.end; ++k) {
        const tune_log_entry *e = &log[k];
        if (!e->rpm_valid[w] || !e->iq_ref_valid[w])
          continue;
        peak = fmax(peak, sign * e->iq_ref_a[w]);
        if (sign * e->rpm[w] >= 1) {
          status = append(&out[w].breakaway, peak);
          break;
        }
      }
      if (status)
        goto done;
      size_t used = 0;
      for (size_t k = seg.first; k < seg.end; ++k) {
        const tune_log_entry *e = &log[k];
        if (e->t >= half && e->rpm_valid[w] && e->iq_ref_valid[w] && e->target_rpm[w] &&
            fabs(e->rpm[w] - e->target_rpm[w]) <= track_tol * fabs(e->target_rpm[w]))
          track[used++] = sign * e->iq_ref_a[w];
      }
      if (used) {
        status = append(&out[w].kinetic, median(track, used));
        if (status)
          goto done;
      }
    }
  }
done:
  free(track);
  return status;
}

double tune_spread(const double *values, size_t n) {
  if (n < 2)
    return INFINITY;
  if (!values || n > SIZE_MAX / sizeof(double))
    return NAN;
  double *copy = malloc(n * sizeof(*copy));
  if (!copy)
    return NAN;
  memcpy(copy, values, n * sizeof(*copy));
  double m = median(copy, n), result = INFINITY;
  if (m > 0) {
    for (size_t k = 0; k < n; ++k)
      copy[k] = fabs(values[k] - m);
    result = median(copy, n) / m;
  }
  free(copy);
  return result;
}

int tune_overshoot(const tune_log_entry *log, size_t n, double min_target,
                   double window, tune_values out[TUNE_WHEELS]) {
  if ((!log && n) || !out)
    return TUNE_INVALID;
  for (size_t w = 0; w < TUNE_WHEELS; ++w) {
    if (!valid_values(&out[w]))
      return TUNE_INVALID;
    out[w].count = 0;
  }
  size_t cursor = 0;
  tune_segment seg;
  while (next_segment(log, n, min_target, &cursor, &seg)) {
    for (size_t w = 0; w < TUNE_WHEELS; ++w) {
      double target = target_peak(log, seg, w);
      if (fabs(target) < min_target)
        continue;
      double peak = 0, sign = target > 0 ? 1 : -1;
      bool found = false;
      for (size_t k = seg.first; k < seg.end; ++k) {
        if (log[k].t - log[seg.first].t <= window && log[k].rpm_valid[w]) {
          double rpm = log[k].rpm[w] * sign;
          if (!found || rpm > peak)
            peak = rpm;
          found = true;
        }
      }
      int status = append(&out[w], peak / fabs(target));
      if (status)
        return status;
    }
  }
  return TUNE_OK;
}

int tune_response(const tune_log_entry *log, size_t n, tune_response_result *out) {
  if ((!log && n) || !out || n > SIZE_MAX / (9 * sizeof(double)))
    return TUNE_INVALID;
  memset(out, 0, sizeof(*out));
  out->spread_s = out->spread_worst_s = NAN;
  for (size_t w = 0; w < TUNE_WHEELS; ++w) {
    out->wheels[w].breakaway_s = out->wheels[w].breakaway_worst_s = NAN;
    out->wheels[w].overshoot = out->wheels[w].overshoot_worst = NAN;
  }
  if (!n)
    return TUNE_OK;
  double *storage = malloc(9 * n * sizeof(*storage));
  if (!storage)
    return TUNE_MEMORY;
  tune_values over[TUNE_WHEELS], times[TUNE_WHEELS];
  for (size_t w = 0; w < TUNE_WHEELS; ++w) {
    over[w] = (tune_values){storage + w * n, n, 0};
    times[w] = (tune_values){storage + (4 + w) * n, n, 0};
  }
  int status = tune_overshoot(log, n, TUNE_MIN_TARGET_RPM, TUNE_OVERSHOOT_WINDOW_S, over);
  if (status) {
    free(storage);
    return status;
  }
  double *spreads = storage + 8 * n;
  size_t spread_count = 0;
  bool idle = true;
  for (size_t k = 0; k < n; ++k) {
    bool active = moving(&log[k]);
    if (idle && active) {
      double earliest = INFINITY, latest = -INFINITY;
      size_t moved = 0;
      for (size_t w = 0; w < TUNE_WHEELS; ++w) {
        double peak = 0, first = NAN;
        /* Python's breakaway window can extend past this segment's end. */
        for (size_t j = k; j < n; ++j) {
          const tune_log_entry *e = &log[j];
          if (e->t - log[k].t > 1.0)
            continue;
          peak = fmax(peak, fabs(e->target_rpm[w]));
          if (isnan(first) && e->rpm_valid[w] && fabs(e->rpm[w]) >= 20 &&
              e->rpm[w] * e->target_rpm[w] > 0)
            first = e->t - log[k].t;
        }
        if (peak < TUNE_MIN_TARGET_RPM)
          continue;
        ++out->wheels[w].starts;
        if (!isnan(first)) {
          times[w].data[times[w].count++] = first;
          ++moved;
          earliest = fmin(earliest, first);
          latest = fmax(latest, first);
        }
      }
      if (moved >= 2)
        spreads[spread_count++] = latest - earliest;
    }
    idle = !active;
  }
  for (size_t w = 0; w < TUNE_WHEELS; ++w) {
    tune_wheel_response *r = &out->wheels[w];
    r->moved = times[w].count;
    r->breakaway_s = median(times[w].data, times[w].count);
    if (times[w].count)
      r->breakaway_worst_s = times[w].data[times[w].count - 1];
    r->overshoot = median(over[w].data, over[w].count);
    if (over[w].count)
      r->overshoot_worst = over[w].data[over[w].count - 1];
  }
  out->spread_s = median(spreads, spread_count);
  if (spread_count)
    out->spread_worst_s = spreads[spread_count - 1];
  free(storage);
  return TUNE_OK;
}
