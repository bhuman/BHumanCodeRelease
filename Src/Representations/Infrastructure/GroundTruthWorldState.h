/**
 * @file GroundTruthWorldModel.h
 *
 * The file declares a struct that contains the state of the world in the current simulation szene.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct GroundTruthWorldState
 * A struct that contains the state of the world in the current simulation szene.
 */
STREAMABLE(GroundTruthWorldState,
{
  STREAMABLE(GroundTruthPlayer,
  {,
    (int) number,
    (Pose2f) pose,
    (bool) upright,
  }),

  (std::vector<GroundTruthPlayer>) bluePlayers,
  (std::vector<GroundTruthPlayer>) redPlayers,
  (std::vector<Vector2f>) balls,
  (Pose2f) ownPose,
});
