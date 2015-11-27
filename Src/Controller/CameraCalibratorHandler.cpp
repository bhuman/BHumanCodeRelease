#include "CameraCalibratorHandler.h"
#include "Tools/Debugging/DebugRequest.h"
#include "Controller/ImageViewAdapter.h"
#include "Controller/RobotConsole.h"
#include "Tools/Debugging/Debugging.h"
#include <sstream>

void CameraCalibratorHandler::deliverPoint(const Vector2i& point, bool upper, bool deletionRequired)
{
  std::stringstream line1;
  line1 << "set module:CameraCalibrator:currentCamera " << (upper ? "upper;" : "lower;");
  robotConsole->handleConsoleLine(line1.str());
  std::stringstream line2;
  line2 << "set module:CameraCalibrator:point x = ";
  line2 << point.x();
  line2 << "; y = ";
  line2 << point.y();
  line2 << ";";
  robotConsole->handleConsoleLine(line2.str());
}

void CameraCalibratorHandler::setActive(std::string view)
{
  if(!ImageViewAdapter::addListener(this, view))
    OUTPUT(idText, text, "cameraCalibrator is already active for \"" + view + "\"");
}

void CameraCalibratorHandler::setInactive(std::string view)
{
  ImageViewAdapter::removeListener(this, view);
}
