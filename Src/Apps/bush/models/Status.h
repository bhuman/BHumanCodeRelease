#pragma once

class Status
{
public:
  bool batteryCharging = false;
  int batteryLevel = 101;
  int logs = 0;
  bool hasDump = false;

  Status() = default;

  bool isPowerValid() const;
};
