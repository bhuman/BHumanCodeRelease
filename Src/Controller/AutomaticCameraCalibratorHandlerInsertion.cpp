#include "AutomaticCameraCalibratorHandlerInsertion.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Controller/ImageViewAdapter.h"
#include "Controller/RobotConsole.h"
#include "Tools/Debugging/Debugging.h"
#include <sstream>

void AutomaticCameraCalibratorHandlerInsertion::deliverPoint(const Vector2i& point, bool upper, bool deletionRequired)
{
  if(!deletionRequired)
  {
    std::stringstream line1;
    line1 << "set module:AutomaticCameraCalibrator:insertionCurrentCamera " << (upper ? "upper;" : "lower;");
    robotConsole->handleConsoleLine(line1.str());
    std::stringstream line2;
    line2 << "set module:AutomaticCameraCalibrator:insertionPoint x = ";
    line2 << point.x();
    line2 << "; y = ";
    line2 << point.y();
    line2 << ";";
    robotConsole->handleConsoleLine(line2.str());
    std::stringstream line3;
    line3 << "dr module:AutomaticCameraCalibrator:insertPoint";
    robotConsole->handleConsoleLine(line3.str());
  }
}

void AutomaticCameraCalibratorHandlerInsertion::setActive(std::string view)
{
  if(!ImageViewAdapter::addListener(this, view))
    OUTPUT(idText, text, "AutomaticCameraCalibratorInsertion is already active for \"" + view + "\"");
}

void AutomaticCameraCalibratorHandlerInsertion::setInactive(std::string view)
{
  ImageViewAdapter::removeListener(this, view);
}
