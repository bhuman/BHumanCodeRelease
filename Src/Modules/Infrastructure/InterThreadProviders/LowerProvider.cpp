/**
 * @file LowerProvider.cpp
 *
 * This file implements a module that provides representations from the lower
 * camera thread for the current frame.
 * @author Thomas Röfer
 */

#include "LowerProvider.h"

MAKE_MODULE(LowerProvider, infrastructure)

void LowerProvider::update(OtherFieldBoundary& theOtherFieldBoundary)
{
  static_cast<FieldBoundary&>(theOtherFieldBoundary) = theLowerFieldBoundary;
}

void LowerProvider::update(OtherObstaclesPerceptorData& theOtherObstaclesPerceptorData)
{
  static_cast<ObstaclesPerceptorData&>(theOtherObstaclesPerceptorData) = theLowerObstaclesPerceptorData;
  theOtherObstaclesPerceptorData.imageCoordinateSystem.cameraInfo = theLowerObstaclesPerceptorData.cameraInfo;
}
