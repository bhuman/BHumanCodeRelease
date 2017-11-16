#include "Utils/bush/models/Power.h"

Power::Power(int value, bool batteryCharging)
  : batteryCharging(batteryCharging), value(value)
{
}

bool Power::isValid()
{
  return value >= 0 && value <= 100;
}
