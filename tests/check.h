/* Minimal assertion macros shared by the host tests: a failed CHECK prints the
 * location and exits nonzero, which CTest reports as a failure. */
#ifndef TEST_CHECK_H
#define TEST_CHECK_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);        \
      exit(1);                                                     \
    }                                                              \
  } while (0)

#define CHECK_EQ(a, b) CHECK((long long)(a) == (long long)(b))

/* Floating-point comparison with an absolute tolerance. */
#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

#endif
