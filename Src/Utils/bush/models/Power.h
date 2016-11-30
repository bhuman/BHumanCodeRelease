#pragma once

class Power
{
public:
  bool batteryCharging = false;
  int value = 101;

  Power() = default;
  Power(int value, bool batteryCharging);

  bool isValid();
};
