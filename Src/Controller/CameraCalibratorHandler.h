
#pragma once

#include "ImageViewAdapter.h"
#include "Tools/MessageQueue/MessageQueue.h"

class RobotConsole;

class CameraCalibratorHandler : public PointListener
{
private:
  RobotConsole* robotConsole;
public:
  CameraCalibratorHandler(RobotConsole* console) : robotConsole(console) {}

  void deliverPoint(const Vector2<int>& point);

  void setActive(std::string view);

  void setInactive(std::string view);
};
