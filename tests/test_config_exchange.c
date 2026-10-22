#define _POSIX_C_SOURCE 200809L
#include "check.h"
#include "ik_link.h"
#include "gen_esc_defaults.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

static uint32_t addr[IK_SYM_COUNT];
static void setup(int wheel, int status, int mismatch)
{
  char mem[8192];
  size_t n = (size_t)snprintf(mem, sizeof mem, "%x=%x,%x=0,%x=5,%x=6", addr[IK_ABI_SYM], IK_ABI,
                             addr[IK_STATE], addr[IK_RAW_SEQ], addr[IK_RAW_DONE]);
  for (int w = 0; w < 4; w++) for (int k = 0; k < IK_LINK_WORDS; k++)
    n += (size_t)snprintf(mem + n, sizeof mem - n, ",%x=%x", addr[IK_LINK] + (w * IK_LINK_WORDS + k) * 4,
                         k == 19 ? UINT32_MAX : k == 20 ? 200 : 0);
  setenv("FAKE_OCD_MEM", mem, 1);
  uint32_t row = addr[IK_LINK] + (uint32_t)wheel * IK_LINK_WORDS * 4U;
  /* Publish a literal reply when raw_seq is written. Only SWD memory is scripted. */
  snprintf(mem, sizeof mem, "%x>%x=%x,%x=%x,%x=%x,%x=%x,%x=%x,%x=%x,%x=%x",
    addr[IK_RAW_SEQ], row, 2, row + 19 * 4, mismatch == 3 ? UINT32_MAX : 0,
    row + 20 * 4, mismatch == 1 ? 202 : 201, row + 21 * 4, status,
    row + 22 * 4, mismatch == 2 ? MOTOR_CFG_SPEED_KP : MOTOR_CFG_HALL_SHIFT_DEG,
    row + 23 * 4, (uint32_t)-64, row + 24 * 4, (uint32_t)-63);
  setenv("FAKE_OCD_ON_WRITE", mem, 1);
}

static void exchanges(void)
{
  const int statuses[] = {0, MOTOR_R_NOT_IDLE, MOTOR_CFG_UNKNOWN_KEY, MOTOR_CFG_BAD_VALUE, MOTOR_R_BAD_VERSION};
  for (int w = 0; w < 4; w++) for (size_t i = 0; i < sizeof statuses / sizeof *statuses; i++) {
    setup(w, statuses[i], 0);
    ik_link_t link; CHECK_EQ(ik_link_open(&link, "FAKE", FAKE_ELF), 0);
    link.config_seq = 200;
    struct motor_config_reply reply;
    int rc = ik_config_exchange(&link, w, MOTOR_CONFIG_SET, MOTOR_CFG_HALL_SHIFT_DEG, -64, 0.15, &reply);
    CHECK_EQ(rc, statuses[i] ? -1 : 0); CHECK_EQ(reply.status, statuses[i]);
    CHECK_EQ(reply.seq, 201); CHECK_EQ(reply.key, MOTOR_CFG_HALL_SHIFT_DEG);
    CHECK_EQ(reply.stored, -64); CHECK_EQ(reply.effective, -63);
    if (rc) CHECK(strstr(link.err, ik_config_status_name(statuses[i])));
    uint32_t value;
    CHECK_EQ(ik_read(&link, IK_RAW_ID, &value), 0); CHECK_EQ(value, MOTOR_CAN_ID_CONFIG | IK_WHEEL_SELECT[w]);
    CHECK_EQ(ik_read(&link, IK_RAW_LEN, &value), 0); CHECK_EQ(value, 8);
    CHECK_EQ(ik_read(&link, IK_RAW_DATA, &value), 0); CHECK_EQ(value, 0x0002c911);
    CHECK_EQ(ik_read(&link, IK_RAW_DATA_HI, &value), 0); CHECK_EQ(value, 0x0000ffc0);
    ik_link_close(&link);
  }
}

static void matching_and_timeouts(void)
{
  for (int w = 0; w < 4; w++) for (int mismatch = 1; mismatch <= 4; mismatch++) {
    setup(mismatch == 4 ? (w + 1) % 4 : w, 0, mismatch);
    ik_link_t link; CHECK_EQ(ik_link_open(&link, "FAKE", FAKE_ELF), 0); link.config_seq = 200;
    struct motor_config_reply reply;
    CHECK_EQ(ik_config_exchange(&link, w, MOTOR_CONFIG_GET, MOTOR_CFG_HALL_SHIFT_DEG, 0, 0.03, &reply), -1);
    CHECK(strstr(link.err, "timeout")); CHECK_EQ(reply.seq, 0);
    ik_link_close(&link);
  }
  setup(0, 0, 0);
  ik_link_t link; CHECK_EQ(ik_link_open(&link, "FAKE", FAKE_ELF), 0);
  struct motor_config_reply reply;
  CHECK_EQ(ik_write(&link, IK_RAW_DONE, 0), 0);
  CHECK_EQ(ik_config_exchange(&link, 0, MOTOR_CONFIG_GET, 2, 0, 0.02, &reply), -1);
  CHECK(strstr(link.err, "not sent"));
  CHECK_EQ(ik_config_exchange(&link, -1, MOTOR_CONFIG_GET, 2, 0, 0.1, &reply), -1);
  CHECK_EQ(ik_config_exchange(&link, 0, MOTOR_CONFIG_SET, 2, 32768, 0.1, &reply), -1);
  CHECK_EQ(ik_config_exchange(&link, 0, MOTOR_CONFIG_GET, 2, 1, 0.1, &reply), -1);
  CHECK(strstr(link.err, "RESERVED_NONZERO"));
  CHECK_EQ(ik_write(&link, IK_STATE, 1), 0);
  CHECK_EQ(ik_config_exchange(&link, 0, MOTOR_CONFIG_GET, 2, 0, 0.1, &reply), -1);
  CHECK(strstr(link.err, "not stopped"));
  ik_link_close(&link);
}

static void current_mailbox(void)
{
  setup(0, 0, 0);
  ik_link_t link; CHECK_EQ(ik_link_open(&link, "FAKE", FAKE_ELF), 0);
  CHECK_EQ(ik_set_mode(&link, IK_MODE_CURRENT), 0);
  uint32_t value; CHECK_EQ(ik_read(&link, IK_MODE, &value), 0); CHECK_EQ(value, IK_MODE_CURRENT);
  int32_t currents[] = {-ESC_IQMAX_A * 1000 - 1, -300, 300, ESC_IQMAX_A * 1000 + 1};
  CHECK_EQ(ik_drive_current(&link, currents, true), 0);
  int expected[] = {-ESC_IQMAX_A * 1000, -300, 300, ESC_IQMAX_A * 1000};
  for (int w = 0; w < 4; w++) {
    CHECK_EQ(ocd_read32(link.ocd, link.addr[IK_CURRENT_MA] + w * 4, &value), 0);
    CHECK_EQ((int32_t)value, expected[w]);
  }
  CHECK_EQ(ik_read(&link, IK_ENABLE, &value), 0); CHECK_EQ(value, 1);
  CHECK_EQ(ik_write(&link, IK_STATE, 1), 0);
  CHECK_EQ(ik_set_mode(&link, IK_MODE_SPEED), -1);
  CHECK_EQ(ik_read(&link, IK_MODE, &value), 0); CHECK_EQ(value, IK_MODE_CURRENT);
  CHECK_EQ(ik_set_mode(&link, 2), -1);
  ik_link_close(&link);
}

int main(void)
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in a = {.sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
  bool available = fd >= 0 && bind(fd, (struct sockaddr *)&a, sizeof a) == 0;
  if (fd >= 0) close(fd);
  if (!available) { puts("SKIP: sandbox denies loopback sockets needed by fake OpenOCD RPC"); return 77; }
  char error[256]; CHECK_EQ(ik_resolve(FAKE_ELF, addr, error, sizeof error), 0);
  setenv("OPENOCD", FAKE_OCD, 1);
  exchanges(); matching_and_timeouts(); current_mailbox();
  puts("config_exchange: 3 test groups passed (all four wheels)");
  return 0;
}
