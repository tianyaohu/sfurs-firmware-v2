#ifndef TEST_FWU_CASES_H
#define TEST_FWU_CASES_H

#include "check.h"
#include <string.h>

struct fwu_test_case {
  const char *name;
  void (*run)(void);
};

/* CTest starts each scenario separately to preserve firmware power-up state. */
static int fwu_run_case(int argc, char **argv,
                        const struct fwu_test_case *cases, size_t count)
{
  CHECK(argc == 1 || argc == 2);
  const char *name = argc == 2 ? argv[1] : cases[0].name;
  for (size_t i = 0; i < count; ++i) {
    if (strcmp(name, cases[i].name) == 0) {
      cases[i].run();
      return 0;
    }
  }
  fprintf(stderr, "Unknown FWU case: %s\n", name);
  return 1;
}

#endif
