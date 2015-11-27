#include "Utils/bush/models/Power.h"

Power::Power() : value(101)
{
}

Power::Power(int value) : value(value)
{
}

bool Power::isValid()
{
  return value >= 0 && value <= 100;
}
