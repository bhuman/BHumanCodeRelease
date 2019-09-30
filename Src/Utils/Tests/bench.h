#pragma once

#include "gPrintf.h"
#include "bench/BenchTimer.h"

#define RUN_BENCH(TRIES,REP, ...) do { \
    Eigen::BenchTimer timer; \
    BENCH(timer, TRIES, REP, PROTECT(__VA_ARGS__)) \
    PRINTF("best: %.3fs, avg: %.3fs, worst: %.3fs \n", timer.best(), timer.total() / TRIES, timer.worst()); \
  } while(false)
