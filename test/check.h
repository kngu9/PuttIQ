#pragma once
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do { \
  ++g_checks; \
  if (!(cond)) { ++g_failures; \
    std::printf("FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
  ++g_checks; \
  double _d = std::fabs((double)(a) - (double)(b)); \
  if (_d > (tol)) { ++g_failures; \
    std::printf("FAIL %s:%d  CHECK_NEAR(%s=%g, %s=%g, tol=%g) diff=%g\n", \
      __FILE__, __LINE__, #a, (double)(a), #b, (double)(b), (double)(tol), _d); } \
} while (0)

#define RUN(fn) do { std::printf("-- %s\n", #fn); fn(); } while (0)

#define REPORT() do { \
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures); \
  return g_failures == 0 ? 0 : 1; \
} while (0)
