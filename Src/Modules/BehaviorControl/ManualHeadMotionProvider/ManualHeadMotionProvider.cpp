#include "ManualHeadMotionProvider.h"
#include "Tools/Math/Transformation.h"
#include "Debugging/Debugging.h"
#include "Math/Eigen.h"

MAKE_MODULE(ManualHeadMotionProvider);

void ManualHeadMotionProvider::update(HeadMotionRequest& headMotionRequest)
{
  bool parametersChanged = xImg != currentX || yImg != currentY;
  if(parametersChanged )
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
      headMotionRequest.speed = 150_deg;
    }
  }
}
