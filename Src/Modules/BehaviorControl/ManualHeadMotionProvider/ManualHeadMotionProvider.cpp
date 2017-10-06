#include "ManualHeadMotionProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Eigen.h"

MAKE_MODULE(ManualHeadMotionProvider, behaviorControl)

void ManualHeadMotionProvider::update(HeadMotionRequest& headMotionRequest)
{
  bool parametersChanged = xImg != currentX || yImg != currentY;
  if(parametersChanged && camera == theCameraInfo.camera)
  {
    currentX = xImg;
    currentY = yImg;

    Vector2f targetOnField;
    if(Transformation::imageToRobot(currentX, currentY, theCameraMatrix, theCameraInfo, targetOnField))
    {
      headMotionRequest.target.x() = targetOnField.x();
      headMotionRequest.target.y() = targetOnField.y();
      headMotionRequest.target.z() = 0;
      headMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;

      //Use the camera that the user is seeing right now.
      switch(camera)
      {
        case CameraInfo::lower:
          headMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera;
          break;
        case CameraInfo::upper:
          headMotionRequest.cameraControlMode = HeadMotionRequest::upperCamera;
          break;
        default:
          FAIL("Unknown camera.");
      }

      headMotionRequest.speed = 150_deg;
    }
  }
}
