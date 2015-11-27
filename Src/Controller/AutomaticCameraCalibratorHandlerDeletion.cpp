#include "AutomaticCameraCalibratorHandlerDeletion.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Controller/ImageViewAdapter.h"
#include "Controller/RobotConsole.h"
#include "Tools/Debugging/Debugging.h"
#include <sstream>

void AutomaticCameraCalibratorHandlerDeletion::deliverPoint(const Vector2i& point, bool upper, bool deletionRequired)
{
  if(deletionRequired)
  {
    std::stringstream line1;
    line1 << "set module:AutomaticCameraCalibrator:deletionCurrentCamera " << (upper ? "upper;" : "lower;");
    robotConsole->handleConsoleLine(line1.str());
    std::stringstream line2;
    line2 << "set module:AutomaticCameraCalibrator:deletionPoint x = ";
    line2 << point.x();
    line2 << "; y = ";
    line2 << point.y();
    line2 << ";";
    robotConsole->handleConsoleLine(line2.str());
    std::stringstream line3;
    line3 << "dr module:AutomaticCameraCalibrator:deletePoint";
    robotConsole->handleConsoleLine(line3.str());
  }
}

void AutomaticCameraCalibratorHandlerDeletion::setActive(std::string view)
{
  if(!ImageViewAdapter::addListener(this, view))
    OUTPUT(idText, text, "AutomaticCameraCalibratorDeletion is already active for \"" + view + "\"");
}

void AutomaticCameraCalibratorHandlerDeletion::setInactive(std::string view)
{
  ImageViewAdapter::removeListener(this, view);
}
