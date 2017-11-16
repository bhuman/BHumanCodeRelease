#include "Tools/Math/Random.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Optimization/DownhillSimplexOptimizer.h"
#include "Tools/Motion/LIP.h"

#include "bench.h"

#include "gtest/gtest.h"

#ifndef NDEBUG
#define RUNS 1000
#define TRIES 10
#else
#define RUNS 10000
#define TRIES 50
#endif

GTEST_TEST(DownhillSimplex, bench1)
{
  using Vector1f = Eigen::Matrix<float, 1, 1>;

  auto errorFunction = []
                       (const Vector1f& vals)
  {
    const LIP currentState(250.f);
    const LIP targetState(250.f);
    float timeToStep = 0.3f;
    float timeAfterStep = 0.3f;
    float footTrans = -50.f;
    float posWeight = 1;
    float velWeight = 50;
    const float currentZmp = vals(0);

    LIP forwardedState = currentState.predict(timeToStep, currentZmp);
    forwardedState.position += footTrans;
    forwardedState.update(timeAfterStep, 0.f);

    return posWeight * sqr(targetState.position - forwardedState.position) +
           velWeight * sqr(targetState.velocity - forwardedState.velocity);
  };

  std::array<Rangef, 1> ranges;
  ranges[0] = Rangef(-50.f, 50);
  auto optimizer = Optimizer::makeDownhillSimplexOptimizer(errorFunction, ranges);

  auto errorFunction2 = []
                        (const float& currentZmp)
  {
    const LIP currentState(250.f);
    const LIP targetState(250.f);
    float timeToStep = 0.3f;
    float timeAfterStep = 0.3f;
    float footTrans = -50.f;
    float posWeight = 1;
    float velWeight = 50;

    LIP forwardedState = currentState.predict(timeToStep, currentZmp);
    forwardedState.position += footTrans;
    forwardedState.update(timeAfterStep, 0.f);

    return posWeight * sqr(targetState.position - forwardedState.position) +
           velWeight * sqr(targetState.velocity - forwardedState.velocity);
  };

  const Rangef range(-50.f, 50);
  auto optimizer2 = Optimizer::makeDownhillSimplexOptimizer1(errorFunction2, range);

  Eigen::BenchTimer optTimer;
  Eigen::BenchTimer opt2Timer;

  for(int i = 0; i < TRIES; ++i)
  {
    Vector1f start, delta;
    start.x() = -1.f;
    delta.x() = 10.f;

    optimizer.initRandom(start);
    auto copy = optimizer.simplex;
    // waste some cycles to get the return type automatically
    auto res = optimizer.optimize(0.0001f, 1);
    optTimer.start();
    for(int j = 0; j < RUNS; ++j)
    {
      optimizer.simplex = copy;
      res = optimizer.optimize(0.0001f, 100);
    }
    optTimer.stop();

    optimizer2.simplex[0] = copy[0].values[0];
    optimizer2.simplex[1] = copy[1].values[0];

    auto copy2 = optimizer2.simplex;
    Optimizer::Result<float> res2;

    opt2Timer.start();
    for(int j = 0; j < RUNS; ++j)
    {
      optimizer2.simplex = copy2;
      res2 = optimizer2.optimize(0.0001f, 100);
    }
    opt2Timer.stop();

    if(res.best(0) == res2.best)
      EXPECT_TRUE(res.best(0) == res2.best);
  }

  PRINTF("generic optimizer overall - best: %.3fs, avg: %.3fs, worst: %.3fs \n", optTimer.best(), optTimer.total() / TRIES, optTimer.worst());
  PRINTF("generic optimizer per run - best: %.6fs, avg: %.6fs, worst: %.6fs \n", optTimer.best() / RUNS, optTimer.total() / (TRIES * RUNS), optTimer.worst() / RUNS);
  PRINTF("special optimizer overall - best: %.3fs, avg: %.3fs, worst: %.3fs \n", opt2Timer.best(), opt2Timer.total() / TRIES, opt2Timer.worst());
  PRINTF("special optimizer per run - best: %.6fs, avg: %.6fs, worst: %.6fs \n", opt2Timer.best() / RUNS, opt2Timer.total() / (TRIES * RUNS), opt2Timer.worst() / RUNS);
}

GTEST_TEST(DownhillSimplex, bench2)
{
  std::array<Rangef, 2> ranges;
  ranges[0] = Rangef(-50.f, 50);
  ranges[1] = Rangef(-50.f, 50);

  auto errorFunction = []
                       (const Vector2f& vals)
  {
    LIP currentState(250.f);
    LIP targetState(250.f);
    float timeToStep = 0.3f;
    float timeAfterStep = 0.3f;
    float footTrans = -50.f;
    float posWeight = 1;
    float velWeight = 50;

    const float currentZmp = vals(0);
    const float nextZmp = vals(1);

    LIP forwardedState = currentState.predict(timeToStep, currentZmp);
    forwardedState.position += footTrans;
    forwardedState.update(timeAfterStep, nextZmp);

    return posWeight * sqr(targetState.position - forwardedState.position) +
           velWeight * sqr(targetState.velocity - forwardedState.velocity);
  };
  auto optimizer = Optimizer::makeDownhillSimplexOptimizer(errorFunction, ranges);

  Eigen::BenchTimer optTimer;

  for(int i = 0; i < TRIES; ++i)
  {
    optimizer.initRandom(Vector2f(-1.f, 0.f));
    auto copy = optimizer.simplex;
    // waste some cycles to get the return type automatically
    auto res = optimizer.optimize(0.0001f, 1);
    optTimer.start();
    for(int j = 0; j < RUNS; ++j)
    {
      optimizer.simplex = copy;
      res = optimizer.optimize(0.0001f, 100);
    }
    optTimer.stop();
  }

  PRINTF("overall - best: %.3fs, avg: %.3fs, worst: %.3fs \n", optTimer.best(), optTimer.total() / TRIES, optTimer.worst());
  PRINTF("per run - best: %.6fs, avg: %.6fs, worst: %.6fs \n", optTimer.best() / RUNS, optTimer.total() / (TRIES * RUNS), optTimer.worst() / RUNS);
}

GTEST_TEST(DownhillSimplex, bench4)
{
  std::array<Rangef, 4> ranges;
  ranges[0] = Rangef(-50.f, 50);
  ranges[1] = Rangef(-50.f, 50);
  ranges[2] = Rangef(0, 1);
  ranges[3] = Rangef(0, 1);

  float timeToStepTarget = 0.3f;
  float timeAfterStepTarget = 0.3f;
  auto errorFunction = [timeToStepTarget, timeAfterStepTarget]
                       (const Vector4f& vals)
  {
    LIP currentState(250.f);
    LIP targetState(250.f);
    float footTrans = -50.f;
    float posWeight = 1;
    float velWeight = 50;
    float timeWeight = 500;

    const float currentZmp = vals(0);
    const float nextZmp = vals(1);
    const float timeToStep = vals(2);
    const float timeAfterStep = vals(3);

    LIP forwardedState = currentState.predict(timeToStep, currentZmp);
    forwardedState.position += footTrans;
    forwardedState.update(timeAfterStep, nextZmp);

    return posWeight * sqr(targetState.position - forwardedState.position) +
           velWeight * sqr(targetState.velocity - forwardedState.velocity) +
           timeWeight * sqr(timeToStepTarget - timeToStep) +
           timeWeight * sqr(timeAfterStepTarget - timeAfterStep);
  };
  auto optimizer = Optimizer::makeDownhillSimplexOptimizer(errorFunction, ranges);

  Eigen::BenchTimer optTimer;

  for(int i = 0; i < TRIES; ++i)
  {
    optimizer.initDelta(Vector4f(-1.f, 0.f, timeToStepTarget, timeAfterStepTarget),
                        Vector4f(10.f, 10.f, 0.1f, 0.1f));
    // waste some cycles to get the return type automatically
    auto res = optimizer.optimize(0.0001f, 1);
    optTimer.start();
    for(int j = 0; j < RUNS; ++j)
    {
      res = optimizer.optimize(0.0001f, 1000);
    }
    optTimer.stop();
  }

  PRINTF("overall - best: %.3fs, avg: %.3fs, worst: %.3fs \n", optTimer.best(), optTimer.total() / TRIES, optTimer.worst());
  PRINTF("per run - best: %.6fs, avg: %.6fs, worst: %.6fs \n", optTimer.best() / RUNS, optTimer.total() / (TRIES * RUNS), optTimer.worst() / RUNS);
}
