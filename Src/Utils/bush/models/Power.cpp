#include "Utils/bush/models/Power.h"

Power::Power() : value(101)
{
}

Power::Power(unsigned char value) : value(value)
{
}

bool Power::isValid()
{
  return value >= 0 && value <= 100;
}

Power::operator int()
{
  return static_cast<int>(value);
}
