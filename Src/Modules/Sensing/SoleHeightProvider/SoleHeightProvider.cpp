/*
 * @file SoleHeightProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "SoleHeightProvider.h"
#include <cmath>

MAKE_MODULE(SoleHeightProvider);

void SoleHeightProvider::update(SoleHeightDifference& theSoleHeightDifference)
{
  const Pose3f currentLeftSole3D = theTorsoMatrix * theRobotModel.soleLeft;
  const Pose3f currentRightSole3D = theTorsoMatrix * theRobotModel.soleRight;
  const Pose2f currentSoleLeft(currentLeftSole3D.rotation.getZAngle(), currentLeftSole3D.translation.head<2>());
  const Pose2f currentSoleRight(currentRightSole3D.rotation.getZAngle(), currentRightSole3D.translation.head<2>());

  const Vector3f left1 = currentLeftSole3D * Vector3f(theRobotDimensions.soleToFrontEdgeLength, -theRobotDimensions.soleToInnerEdgeLength, 0.f);
  const Vector3f left2 = currentLeftSole3D * Vector3f(theRobotDimensions.soleToFrontEdgeLength, theRobotDimensions.soleToOuterEdgeLength, 0.f);
  const Vector3f left3 = currentLeftSole3D * Vector3f(-theRobotDimensions.soleToBackEdgeLength, -theRobotDimensions.soleToInnerEdgeLength, 0.f);
  const Vector3f left4 = currentLeftSole3D * Vector3f(-theRobotDimensions.soleToBackEdgeLength, theRobotDimensions.soleToOuterEdgeLength, 0.f);
  const std::vector<float> leftSoleHeights = { left1.z(), left2.z(), left3.z(), left4.z() };
  const float leftSoleMinHeight = *std::min_element(leftSoleHeights.begin(), leftSoleHeights.end());

  const Vector3f right1 = currentRightSole3D * Vector3f(theRobotDimensions.soleToFrontEdgeLength, -theRobotDimensions.soleToInnerEdgeLength, 0.f);
  const Vector3f right2 = currentRightSole3D * Vector3f(theRobotDimensions.soleToFrontEdgeLength, theRobotDimensions.soleToOuterEdgeLength, 0.f);
  const Vector3f right3 = currentRightSole3D * Vector3f(-theRobotDimensions.soleToBackEdgeLength, -theRobotDimensions.soleToInnerEdgeLength, 0.f);
  const Vector3f right4 = currentRightSole3D * Vector3f(-theRobotDimensions.soleToBackEdgeLength, theRobotDimensions.soleToOuterEdgeLength, 0.f);
  const std::vector<float> rightSoleHeights = { right1.z(), right2.z(), right3.z(), right4.z() };
  const float rightSoleMinHeight = *std::min_element(rightSoleHeights.begin(), rightSoleHeights.end());

  theSoleHeightDifference.difference = leftSoleMinHeight - rightSoleMinHeight;
}
