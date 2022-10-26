/**
 * @file GroundTruthWorldState.h
 *
 * The file declares a struct that contains the state of the world in the current simulation szene.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include <vector>

/**
 * @struct GroundTruthWorldState
 * A struct that contains the state of the world in the current simulation szene.
 */
STREAMABLE(GroundTruthWorldState,
{
  STREAMABLE(GroundTruthBall,
  {,
    (Vector3f) position,
    (Vector3f) velocity,
  });
  STREAMABLE(GroundTruthPlayer,
  {,
    (int) number,
    (Pose2f) pose,
    (bool) upright,
  }),

  (std::vector<GroundTruthPlayer>) ownTeamPlayers,
  (std::vector<GroundTruthPlayer>) opponentTeamPlayers,
  (std::vector<GroundTruthBall>) balls,
  (Pose2f) ownPose,
});
