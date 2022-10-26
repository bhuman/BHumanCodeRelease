#include "Status.h"

bool Status::isPowerValid() const
{
  return batteryLevel >= 0 && batteryLevel <= 100;
}
