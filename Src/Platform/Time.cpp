#include "Time.h"

unsigned long long Time::base = 0;
unsigned long long Time::threadTimebase = 0;

#ifndef TARGET_ROBOT

bool Time::isInitialized = false;
bool Time::isTimeSimulated = false;
int Time::simulatedTime = 0;

void Time::initialize()
{
  simulatedTime = 100000 - getRealSystemTime();
  isTimeSimulated = false;
  isInitialized = true;
}

void Time::deinitialize()
{
  isInitialized = false;
}

void Time::setSimulatedTime(bool on)
{
  if(isTimeSimulated != on)
  {
    if(on)
      simulatedTime += getRealSystemTime();
    else
      simulatedTime -= getRealSystemTime();
    isTimeSimulated = on;
  }
}

void Time::addSimulatedTime(int dt)
{
  if(isTimeSimulated)
    simulatedTime += dt;
}

#endif

unsigned Time::getCurrentSystemTime()
{
#ifndef TARGET_ROBOT
  if(isInitialized)
  {
    if(isTimeSimulated)
      return simulatedTime;
    else
      return unsigned(getRealSystemTime() + simulatedTime);
  }
  else
#endif
    return getRealSystemTime();
}
