#include "CameraResolution.h"

#include "CameraInfo.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"

void CameraResolution::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(resolution);
  STREAM(timestamp);
  STREAM_REGISTER_FINISH;
}

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

void CameraResolutionRequest::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(resolution, CameraResolution);
  STREAM(timestamp);
  STREAM_REGISTER_FINISH;
}