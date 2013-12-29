/**
* @file HillClimbing.h
*
* Implementation of simple generic hill climbing algorithm
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Common.h"
#include "Tools/Math/Random.h"
#include "Tools/Enum.h"

/**
* @class HillClimbing
*
* Simple optimization for N values of type T
* Reward has different type R
*/


template <typename T, typename R, int N>
class HillClimbing
{
public:
  ENUM(Mode, MINIMIZE, MAXIMIZE);
  Mode mode;

private:
  bool initialized;
  bool newHillDuringLastExecution;
  int maxUnchangedSteps;
  int runCount;
  int numOfRuns;
  int unchangedSteps;

  R bestReward;
  T bestValues[N];
  T currentValues[N];
  T minValues[N];
  T maxValues[N];
  T maxStepSize[N];

public:
  HillClimbing(): initialized(false), newHillDuringLastExecution(false)
  {}

  void init(T startValues[N], T minValues[N], T maxValues[N], T maxStepSize[N],
            R worstReward, Mode mode, int numOfRuns, int maxUnchangedSteps)
  {
    for(int i = 0; i < N; ++i)
    {
      bestValues[i] = startValues[i];
      currentValues[i] = startValues[i];
      this->minValues[i] = minValues[i];
      this->maxValues[i] = maxValues[i];
      this->maxStepSize[i] = maxStepSize[i];
    }
    this->mode = mode;
    bestReward = worstReward;
    initialized = true;
    this->maxUnchangedSteps = maxUnchangedSteps;
    this->numOfRuns = numOfRuns;
    newHillDuringLastExecution = false;
    runCount = 0;
    unchangedSteps = 0;
  }

  void setReward(R reward)
  {
    bool isBetter(false);
    if(mode == MINIMIZE)
      isBetter = reward < bestReward;
    else
      isBetter = reward > bestReward;
    if(isBetter)
    {
      bestReward = reward;
      unchangedSteps = 0;
      for(int i = 0; i < N; ++i)
        bestValues[i] = currentValues[i];
    }
    else
    {
      unchangedSteps++;
      if(unchangedSteps >= maxUnchangedSteps)
      {
        runCount++;
        unchangedSteps = 0;
//      maxUnchangedSteps/=2;
        for(int i = 0; i < N; ++i)
        {
          this->maxStepSize[i] /= 2;
        }
      }
    }
    newHillDuringLastExecution = isBetter;
  }

  void getNextParameterSet(T newValues[N])
  {
    for(int i = 0; i < N; ++i)
    {
      T delta = static_cast<T>(2.0f * (0.5f - randomFloat()) * maxStepSize[i]);
      newValues[i] = bestValues[i] + delta;
      if(newValues[i] > maxValues[i])
        newValues[i] = maxValues[i];
      else if(newValues[i] < minValues[i])
        newValues[i] = minValues[i];
      currentValues[i] = newValues[i];
    }
  }

  void getCurrentParameterSet(T currentValues[N])
  {
    for(int i = 0; i < N; ++i)
      currentValues[i] = this->currentValues[i];
  }

  void getBestParameterSetAndReward(T bestValues[N], R& reward)
  {
    for(int i = 0; i < N; ++i)
      bestValues[i] = this->bestValues[i];
    reward = bestReward;
  }

  bool isInitialized() const
  {
    return initialized;
  }

  bool hasReachedMinimum() const
  {
    T stepSum = 0;
    for(int i = 0; i < N; i++)
    {
      stepSum += this->maxStepSize[i];
    }

    return (stepSum == 0 || runCount >= numOfRuns);
  }

  bool wasNewHillDuringLastExecution() const
  { return newHillDuringLastExecution;}

  void uninitialize()
  { initialized = false;}

  int getRunCount()
  {
    return runCount;
  }


  int getUnchangedSteps()
  {
    return unchangedSteps;
  }


  void getMaxStepSize(T maxStepSize[N])
  {
    for(int i = 0; i < N; ++i)
    {
      maxStepSize[i] = this->maxStepSize[i];
    }
  }


};
