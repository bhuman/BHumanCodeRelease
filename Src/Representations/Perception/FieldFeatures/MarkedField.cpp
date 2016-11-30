/**
 * @file MarkedField.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "MarkedField.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"

void MarkedField::draw() const //TODO
{
  FieldFeature::draw();

}

const Pose2f MarkedField::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  return Pose2f(0.f, 0.f, 0.f);
}
