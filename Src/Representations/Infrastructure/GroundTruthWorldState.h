/**
* @file GroundTruthWorldModel.h
*
* The file declares a class that contains the state of the world in the current simulation szene.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
* @class GroundTruthWorldState
* A class that contains the state of the world in the current simulation szene.
*/
STREAMABLE(GroundTruthWorldState,
{
public:
  STREAMABLE(GroundTruthRobot,
  {
  public:,
    (int) number,
    (Pose2D) pose,
    ;
  }),

  (std::vector<GroundTruthRobot>) blueRobots,
  (std::vector<GroundTruthRobot>) redRobots,
  (std::vector<Vector2<>>) balls,
  (Pose2D) ownPose,
});
