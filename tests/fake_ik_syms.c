/* Host ELF carrying the ik_drive_* symbols of the Nucleo streaming IK image, so
 * ik_link.c can resolve addresses exactly as it does from the firmware ELF. */
#include <stdint.h>

#define SYM(name) uint32_t ik_drive_##name
SYM(abi); SYM(vx_mmps); SYM(vy_mmps); SYM(omega_mradps); SYM(enable); SYM(heartbeat); SYM(accel_rpmps);
SYM(raw_data_hi); SYM(raw_len); SYM(clear_seq); SYM(clear_done); SYM(fault); SYM(armed);
SYM(reply_lat_hist)[4][16]; SYM(reply_lat_max_us)[4];
uint32_t can_rx_frame_count;
SYM(link)[4 * 19]; SYM(ff_enable); SYM(ff_coulomb_xy_mn); SYM(ff_viscous_xy_mn_per_mps);
SYM(ff_coulomb_w_mnm); SYM(ff_viscous_w_mnm_per_radps); SYM(ff_a_per_nm_milli);
SYM(ff_scale_pct)[4]; SYM(ff_ma)[4];
SYM(raw_id); SYM(raw_data); SYM(raw_seq); SYM(state); SYM(raw_done); SYM(tx_count); SYM(tx_fail);
SYM(bad_input); SYM(telem_now); SYM(target_rpm)[4]; SYM(rpm)[4]; SYM(telem)[4 * 19]; SYM(capture)[5 + 3 * 512];

int main(void) { return 0; }
