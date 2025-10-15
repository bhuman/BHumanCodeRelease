/**
 * @file GroundTruthWorldState.h
 *
 * The file declares a struct that contains a single robot's state of the world in the current simulation scene.
 * All coordinates are given in millimeters in absolute field coordinates, using B-Human's default field coordinate system.
 * Robot perspective and occlusion do not matter here, all objects on the field are included in this struct, with their exact positions and orientations.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include <vector>

/**
 * @struct GroundTruthWorldState
 * A struct that contains the state of the world in the current simulation scene.
 */
STREAMABLE(GroundTruthWorldState,
{
  STREAMABLE(GroundTruthBall,
  {,
    (Vector3f) position,   /** 3D position of a ball's center */
    (Vector3f) velocity,   /** 3D velocity of a ball */
  });
  STREAMABLE(GroundTruthPlayer,
  {,
    (int) number,          /** The player number of a robot */
    (Pose2f) pose,         /** The position and orientation of a robot */
    (bool) upright,        /** Flag that indicates whether a robot is currently standing (true) or not (false). */
  }),

  (std::vector<GroundTruthPlayer>) ownTeamPlayers,      /** A list of all players of the own team, excluding the robot itself (see ownPose) */
  (std::vector<GroundTruthPlayer>) opponentTeamPlayers, /** A list of all players of the opponent team */
  (std::vector<GroundTruthBall>) balls,                 /** A list of all balls in the current scene (usually just one) */
  (Pose2f) ownPose,                                     /** Position and rotation of the (simulated) robot using this representation */
});
