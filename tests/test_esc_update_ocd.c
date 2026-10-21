/* Extend the existing Tcl RPC fake with a scripted pair of read responses.
 * No updater or ESC execution is simulated. All other bytes use FAKE_OCD_MEM.
 * FAKE_OCD_READ_PAIR="hexaddr=first/second" affects two 32-bit reads only.
 * FAKE_OCD_READ_LOG records RPC requests for checking refusal before writes. */
#define _POSIX_C_SOURCE 200809L
#include <sys/socket.h>
static ssize_t scripted_recv(int fd, void *buf, size_t size, int flags);
#define recv scripted_recv
#define main fake_openocd_main
#include "fake_openocd.c"
#undef main
#undef recv

static ssize_t scripted_recv(int fd, void *buf, size_t size, int flags)
{
  ssize_t n = recv(fd, buf, size, flags);
  if (n <= 0) return n;
  const char *log = getenv("FAKE_OCD_READ_LOG");
  if (log) {
    FILE *f = fopen(log, "a");
    if (f) { fwrite(buf, 1, (size_t)n, f); fputc('\n', f); fclose(f); }
  }
  char command[4096];
  if ((size_t)n >= sizeof command) return n;
  memcpy(command, buf, (size_t)n); command[n] = 0;
  const char *pair = getenv("FAKE_OCD_READ_PAIR");
  unsigned long address, first, second, read_addr;
  unsigned width, count;
  static unsigned reads;
  if (pair && sscanf(pair, "%lx=%lx/%lx", &address, &first, &second) == 3 &&
      sscanf(command, "mem2array _m %u %lx %u;", &width, &read_addr, &count) == 3 &&
      width == 32 && count == 1 && address == read_addr) {
    store(address, reads++ ? second : first);
  }
  return n;
}

int main(int argc, char **argv) { return fake_openocd_main(argc, argv); }
