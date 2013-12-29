

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/ObstacleClusters.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(ObstacleClusterer)
REQUIRES(ObstacleWheel)
REQUIRES(RobotPose)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
REQUIRES(FieldDimensions)
DEFINES_PARAMETER(float, fieldSizeThreshold, 100);
DEFINES_PARAMETER(Vector2<>, robotRotationDeviation, Vector2<>(0.02f, 0.06f)) /**< Deviation of the rotation of the robot's torso (used for estimating covariances) */
DEFINES_PARAMETER(float, robotRadius, 100.0f); /**< All obstacles will be moved away from the robot by this value. */
DEFINES_PARAMETER(float, mergeDistance, 150.0f);
PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleClusters)
END_MODULE

class ObstacleClusterer : public ObstacleClustererBase
{
private:
  struct Cluster
  {
    std::list<Vector2<> > spots;
    Vector2<> centerOfMass;
  };

  void update(ObstacleClusters& clusters);

  /**Copy & paste from ExpRobotLocator */
  Matrix2x2<> getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const;

  /**Determine if a point in absolute field coordinates is inside the field*/
  bool isInsideField(const Vector2<>& pointInWorld);
  void extractCluster(std::list<Vector2<> >& spots, Cluster& cluster);
  void extractFitting(std::list<Vector2<> >& spots, Cluster& cluster, Vector2<> reference);
  void calculateCenterOfMass(Cluster& cluster);
};
