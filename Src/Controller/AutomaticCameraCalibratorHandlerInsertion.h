/**
* @author Alexander Stöwing <stoewing@uni-bremen.de>, please dont let me explain this in detail
*/
#pragma once

#include "ImageViewAdapter.h"
#include "Tools/MessageQueue/MessageQueue.h"

class RobotConsole;

class AutomaticCameraCalibratorHandlerInsertion : public PointListener
{
private:
  RobotConsole* robotConsole;
public:
  AutomaticCameraCalibratorHandlerInsertion(RobotConsole* console) : robotConsole(console) {}

  //This method will not deliver a point it should delete a point
  void deliverPoint(const Vector2i& point, bool upper, bool deletionRequired);

  void setActive(std::string view);

  void setInactive(std::string view);
};
