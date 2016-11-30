#include "CameraResolution.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"

bool CameraResolutionRequest::setRequest(CameraResolution::Resolutions requestedResolutionUpper, CameraResolution::Resolutions requestedResolutionLower)
{
  if(SystemCall::getMode() != SystemCall::Mode::physicalRobot)
  {
    return false;
  }
  switch(requestedResolutionUpper)
  {
    case CameraResolution::defaultRes:
    case CameraResolution::w320h240:
    case CameraResolution::w640h480:
    case CameraResolution::w1280h960:
      switch(requestedResolutionLower)
      {
        case CameraResolution::defaultRes:
        case CameraResolution::w320h240:
        case CameraResolution::w640h480:
        case CameraResolution::w1280h960:
          timestamp = Time::getCurrentSystemTime();
          resolutionUpper = requestedResolutionUpper;
          resolutionLower = requestedResolutionLower;
          return true;
      }
  }
  return false;
}
