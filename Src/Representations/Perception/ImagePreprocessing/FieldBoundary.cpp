/**
 * @author Alexis Tsogias
 */

#include "FieldBoundary.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"

void FieldBoundary::draw() const
{
  DEBUG_DRAWING("representation:FieldBoundary:image", "drawingOnImage")
    if(isValid && Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      Vector2i prev(0, getBoundaryY(0));
      for(const Vector2i& point : boundaryInImage)
      {
        DOT("representation:FieldBoundary:image", point.x(), point.y(), ColorRGBA::orange, ColorRGBA::orange);
        LINE("representation:FieldBoundary:image", prev.x(), prev.y(), point.x(), point.y(), 1, Drawings::solidPen, ColorRGBA::orange);
        prev = point;
      }
      Vector2i point(cameraInfo.width, getBoundaryY(cameraInfo.width));
      LINE("representation:FieldBoundary:image", prev.x(), prev.y(), point.x(), point.y(), 1, Drawings::solidPen, ColorRGBA::orange);
    }

  DEBUG_DRAWING("representation:FieldBoundary:field", "drawingOnField")
    if(isValid && Blackboard::getInstance().exists("CameraInfo")
       && Blackboard::getInstance().exists("CameraMatrix")
       && Blackboard::getInstance().exists("ImageCoordinateSystem"))
    {
      const CameraInfo& cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const CameraMatrix& cameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      const ImageCoordinateSystem& imageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);
      Vector2f prev, last;
      if(Transformation::imageToRobot(imageCoordinateSystem.toCorrected(Vector2i(0, getBoundaryY(0))), cameraMatrix, cameraInfo, prev)
         && Transformation::imageToRobot(imageCoordinateSystem.toCorrected(Vector2i(cameraInfo.width, getBoundaryY(cameraInfo.width))), cameraMatrix, cameraInfo, last))
      {
        for(const Vector2f& point : boundaryOnField)
        {
          LARGE_DOT("representation:FieldBoundary:field", point.x(), point.y(), ColorRGBA::orange, ColorRGBA::orange);
          LINE("representation:FieldBoundary:field", prev.x(), prev.y(), point.x(), point.y(), 20, Drawings::solidPen, ColorRGBA::orange);
          prev = point;
        }
        LINE("representation:FieldBoundary:field", prev.x(), prev.y(), last.x(), last.y(), 20, Drawings::solidPen, ColorRGBA::orange);
      }
    }
}

int FieldBoundary::getBoundaryY(int x) const
{
  ASSERT(boundaryInImage.size() >= 2);

  const Vector2i* left = &boundaryInImage.front();
  const Vector2i* right = &boundaryInImage.back();

  if(x < left->x())
    right = &(*(boundaryInImage.begin() + 1));
  else if(x > right->x())
    left = &(*(boundaryInImage.end() - 2));
  else
  {
    for(const Vector2i& point : boundaryInImage)
    {
      if(point.x() == x)
        return point.y();
      else if(point.x() < x && point.x() > left->x())
        left = &point;
      else if(point.x() > x && point.x() < right->x())
        right = &point;
    }
  }

  float m = static_cast<float>(right->y() - left->y()) / static_cast<float>(right->x() - left->x());

  return static_cast<int>(static_cast<float>(x - left->x()) * m) + left->y();
}
