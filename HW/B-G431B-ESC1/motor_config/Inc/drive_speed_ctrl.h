#ifndef DRIVE_SPEED_CTRL_H
#define DRIVE_SPEED_CTRL_H

/* Speed loop with explicit startup (notes/ff-design.md, stage 1).
 *
 * One dsc_step() per speed-loop tick (1 kHz) replaces PI_Controller() +
 * drive_speed_iq_ref() when cfg.enable is set; with enable = 0 the caller keeps
 * the legacy path. Pure: no MCSDK globals, so the host tests and the chassis
 * simulator link it with the real pid_regulator.c.
 *
 * States: STOPPED (|ref| below the stop threshold: plain PI, no ff)
 *   -> STARTING (no PI: total current ramps from start_from toward start_cap
 *      along the command; integral untouched)
 *   -> RUNNING at the first raw Hall edge in the commanded direction, or with
 *      watch_ms set, once motion is confirmed: an edge to a new furthest
 *      position freezes the ramp for up to watch_ms, and only a second one
 *      under that constant current hands off (tyre and gear wind-up gives one
 *      edge per ~0.5-1 A and then stops; a free wheel keeps going); a watch
 *      that times out resumes the ramp. ff_run carries friction; the
 *      integral holds the rest of the hand-off current,
 *      never below 0, and decays by hold_decay per tick while positive-P is
 *      capped, until pcap_ms have passed and pcap_updates fresh Hall captures
 *      arrived; then plain PI from there)
 *   -> FAILED if no edge within start_hold_ms after reaching start_cap, or no
 *      fresh captures within pcap_fallback_ms of the hand-off; latched, output
 *      0, until the command returns to stop.
 * RUNNING with no commanded-direction edge for max(DSC_RESTICK_MIN_MS, two
 * edge periods at |ref|) has re-stuck: back to STARTING, the ramp resuming at
 * the last hand-off current; DSC_MAX_RESTICKS of
 * those in a row without qualified progress (edge or capture) is FAILED. A command sign change
 * runs plain PI until the wheel has slowed below DSC_REVERSE_RPM, then
 * STARTING; a start while already moving along the command goes straight to
 * RUNNING. Anti-windup: conditional integration against the combined
 * P + I + ff command and the active limit.
 *
 * Preload: each start's ramp begins at preload_pct % of the last confirmed
 * hand-off current in that direction (kept across dsc_reset), so wheels with
 * different breakaway currents reach theirs at about the same time.
 * A stop command clears the integral: a stuck wheel's held breakaway current
 * must not stay on once the command is zero.
 *
 * Units: current in counts (CURRENT_CONV_FACTOR per A), speed in RPM, time in
 * ticks (ms). */

#include <stdint.h>
#include "pid_regulator.h"

#define DSC_STOP_RPM 2      /* |ref| below this is a stop command */
#define DSC_REVERSE_RPM 45  /* standstill threshold (Hall rocking reads +-25-32) */
#define DSC_RESTICK_MIN_MS 100U
/* Below this |ref| the Hall edges come slower than the SDK Hall timeout
 * (150 ms, 1250/150 = 8 RPM, with 2x margin for the acceleration), so speed
 * captures may never arrive: the cap (and frozen integral) stays on, but the
 * capture deadline only counts time spent at or above it. That time adds up
 * across resticks until a capture arrives. */
#define DSC_CAP_MIN_RPM 20
/* Consecutive resticks without progress before FAILED, at any speed.
 * Progress = a new furthest raw displacement along the command, measured from
 * the first hand-off of that command (stick-slip gains ground each cycle,
 * rocking across one edge never passes its first position, coasting the old
 * way before a reversal leaves no deficit), a qualified edge, or a capture. */
#define DSC_MAX_RESTICKS 3U
/* Motion confirmation (watch_ms): an edge closer than this to the one that
 * froze the ramp is the rotor settling into the wind-up, not motion; after a
 * preload step the ramp waits this long before it rises (edges then only move
 * the furthest position). */
#define DSC_WATCH_MIN_MS 3U
#define DSC_SETTLE_MS 10U
enum { DSC_W_NONE, DSC_W_CONFIRM, DSC_W_SETTLE };
#define DSC_EDGE_PERIOD_MS_RPM 1250  /* 48 edges/rev: edge period [ms] = 1250 / RPM */

enum { DSC_STOPPED, DSC_STARTING, DSC_RUNNING, DSC_FAILED };
enum { DSC_R_NONE, DSC_R_ARMED, DSC_R_EDGE, DSC_R_STOP, DSC_R_REVERSE,
       DSC_R_NO_EDGE, DSC_R_NO_CAPTURE, DSC_R_INACTIVE, DSC_R_RESTICK,
       DSC_R_MOVING };

typedef struct {
  uint8_t enable;
  int16_t ff_run;           /* running friction, counts, along the command */
  int16_t ff_per_krpm;      /* + speed term, counts per 1000 RPM of |ref| */
  int16_t start_from;       /* startup ramp start, counts */
  int16_t start_cap;        /* startup ramp ceiling, counts */
  int16_t start_slope;      /* counts per tick */
  uint16_t start_hold_ms;   /* allowed at the ceiling before FAILED */
  int16_t pcap;             /* max positive-along-command P after hand-off, counts */
  uint16_t pcap_ms;         /* minimum cap duration */
  uint8_t pcap_updates;     /* fresh Hall captures needed to release the cap */
  uint16_t pcap_fallback_ms;/* FAILED if those captures have not arrived by then */
  int16_t hold_decay;       /* hand-off hold decay while capped, counts per tick */
  uint16_t watch_ms;        /* confirm motion before the hand-off; 0 = first edge */
  uint8_t preload_pct;      /* ramp start, % of the learned breakaway; 0 = off */
} dsc_cfg_t;

typedef struct {
  int16_t ref_rpm;          /* ramped speed reference */
  int16_t meas_rpm;         /* SDK average speed */
  uint16_t edges_pos;       /* cumulative qualified progress edges, + direction */
  uint16_t edges_neg;       /* cumulative qualified progress edges, - direction */
  uint16_t captures;        /* cumulative qualified Hall period captures */
  int16_t limit;            /* active |iq| limit, counts (thermal derate included) */
  uint8_t active;           /* motor running under drive control */
  uint16_t raw_pos;         /* cumulative raw valid Hall transitions, + direction */
  uint16_t raw_neg;         /* cumulative raw valid Hall transitions, - direction */
} dsc_in_t;

typedef struct {
  uint8_t state, reason;
  int8_t dir;               /* +1 / -1 command direction, 0 when stopped */
  uint16_t t;               /* ticks in the current state (saturating) */
  uint16_t at_cap;          /* STARTING: ticks at start_cap */
  uint16_t edges0, caps0;   /* raw commanded-direction edges and qualified
                               captures latched on entering the state */
  int16_t ramp;             /* startup ramp level, counts (magnitude) */
  int16_t start_iq;         /* command at the hand-off edge: apparent breakaway */
  uint8_t capped;           /* positive-P cap active (integral frozen meanwhile) */
  uint16_t edges_last;      /* commanded-direction edge count at the last edge */
  uint16_t since_edge;      /* ticks since that edge (saturating) */
  uint8_t resticks;         /* consecutive resticks without qualified progress */
  uint16_t caps_seen;       /* capture count when resticks was last cleared */
  uint16_t rp, rn;          /* raw edge counters at the last tick */
  int32_t along;            /* raw displacement along the command, edges (wrap-safe) */
  int32_t best;             /* furthest `along` since the first hand-off of the command */
  uint8_t anchored;         /* first hand-off of the command seen */
  uint8_t progressed;       /* best advanced since the last restick */
  int32_t peak;             /* STARTING: furthest `along` since entering it */
  uint8_t watching;         /* STARTING: DSC_W_*, ramp frozen while not NONE */
  uint16_t watch_t;         /* ticks in that watch */
  int16_t learned[2];       /* last confirmed hand-off |current|, [0] +, [1] -; kept by dsc_reset */
  uint16_t nocap;           /* capped RUNNING ticks with |ref| >= DSC_CAP_MIN_RPM since
                               the last fresh capture; kept across resticks */
  /* last tick, for the recorder */
  int16_t p, i, ff, out;
} dsc_state_t;

/* Hall reversal filter, used by both speed loops. A stuck wheel under high
 * current rocks across one Hall edge: transitions alternate +1/-1 a few ms
 * apart and the SDK reports the one-interval speed (+-200 RPM), which the
 * speed loop turns into +-6 A, which drives the next rock - a self-sustaining
 * limit cycle (FR stall test 2026-10-17: 60 reversals in 384 ms at +-6 A).
 * Until two consecutive transitions agree in direction, and the SDK speed has
 * the same sign, the speed fed to the loop is 0. The startup loop must not
 * take rocking as feedback either, so the filter also keeps qualified
 * counters for dsc_in_t: an edge is progress only when it leaves a one-edge
 * band around the furthest position reached (rocking across one edge adds
 * none), and a capture counts only while the speed is accepted. */
typedef struct {
  uint16_t pos, neg, caps;  /* raw counters at the last call */
  int8_t dir;               /* direction of the last transitions, 0 unknown */
  uint8_t run;              /* consecutive transitions in that direction (saturating) */
  int32_t net, hi, lo;      /* net position and its one-edge band, edges */
  uint16_t qpos, qneg, qcaps; /* qualified progress edges and captures */
  uint16_t since;           /* calls since the last transition (saturating) */
  uint16_t period;          /* calls between the last two transitions, 0 unknown */
} dsc_rev_t;
int16_t dsc_reversal_filter(dsc_rev_t *f, uint16_t edges_pos, uint16_t edges_neg,
                            uint16_t captures, int16_t rpm);
/* Low-speed speed from edge timing (VESC's Hall speed pattern): 60 deg
 * electrical / max(time since the last edge, the last edge interval), one
 * call per ms, so it falls as 1/t once edges stop and is live from the second
 * edge, where the SDK's capture speed still reads 0 for several edges (timer
 * prescaler stepping down from its start-up maximum). 0 until two consecutive
 * transitions agree in direction, like the filter. RPM, 1 ms resolution:
 * meant below DSC_EDGE_SPEED_MAX_RPM. */
#define DSC_EDGE_SPEED_MAX_RPM 100
int16_t dsc_edge_rpm(const dsc_rev_t *f);

/* New band origin at the current raw counters (edges and captures since the
 * last call are dropped, not counted): call whenever the startup loop enters
 * STARTING (start, reversal, restick) and at motor start, so the first edge of
 * the attempt always counts. Rocking stays harmless across rebases because
 * qualified captures still need two agreeing transitions. */
void dsc_rev_rebase(dsc_rev_t *f, uint16_t edges_pos, uint16_t edges_neg, uint16_t captures);

void dsc_reset(dsc_state_t *s);
int16_t dsc_step(const dsc_cfg_t *cfg, dsc_state_t *s, const dsc_in_t *in, PID_Handle_t *pid);

#endif
