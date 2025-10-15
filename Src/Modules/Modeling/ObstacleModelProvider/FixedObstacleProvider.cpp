/**
 * @file FixedObstacleProvider.cpp
 *
 * Implementation of a module that places fixed obstacles into the obstacle model.
 *
 * @author Moritz Oppermann
 */

#include "FixedObstacleProvider.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(FixedObstacleProvider);

void FixedObstacleProvider::update(ObstacleModel& obstacleModel)
{
  obstacleModel.obstacles.clear();
  for (auto& obstacleLocation : fixedObstacleLocations)
  {
    Obstacle ob = Obstacle();
    ob.covariance = cov;
    ob.center = theRobotPose.inverse() * obstacleLocation;
    ob.setLeftRight(theRobotDimensions.robotDepth);
    ob.type = Obstacle::Type::opponent;
    obstacleModel.obstacles.emplace_back(ob);
  }
}
