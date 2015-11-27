#include "CameraResolution.h"
#include "Platform/SystemCall.h"

bool CameraResolutionRequest::setRequest(CameraResolution::Resolutions requestedResolution)
{
  if(SystemCall::getMode() != SystemCall::Mode::physicalRobot)
  {
    return false;
  }
  switch(requestedResolution)
  {
    case CameraResolution::defaultRes:
    case CameraResolution::upper640:
    case CameraResolution::lower640:
    case CameraResolution::both320:
      timestamp = SystemCall::getCurrentSystemTime();
      resolution = requestedResolution;
      return true;
    default:
      return false;
  }
}
