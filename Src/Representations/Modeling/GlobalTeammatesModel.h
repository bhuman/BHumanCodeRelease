/**
 * @file GlobalTeammatesModel.h
 *
 * Declaration of a representation that stores the estimated states
 * of the teammates.
 * The representation is a result of the combination of receveived position information
 * and own observations of teammates.
 *
 * @author Tim Laue
 */

#pragma once

#include <vector>
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct GlobalTeammatesModel
 * A model that combines all available information about the positions of the teammates
 */
STREAMABLE(GlobalTeammatesModel,
{
  /**
   * @struct TeammateEstimate
   * Information that we have about a teammate.
   * Currently only the estimated position and the player number.
   */
   STREAMABLE(TeammateEstimate,
   {
     Vector2f getFuturePosition(unsigned time) const, /**< The estimated position of a teammate in global field coordinates in <time> many milliseconds, 0 for pose.translation */

     (Pose2f) pose,              /**< The estimated pose of a teammate in global field coordinates, rotation is currently unreliable */
     (int)(-1) playerNumber,     /**< The number on the robot's jersey */
     (Vector2f)(Vector2f::Zero()) relativeWalkTarget, /**< The target position the robot is walking to (in robot relative coordinates). Based on communicated BehaviorStatus. */
     (float)(-1.f) speed,        /**< The absolute speed in mm/s. Copied from communicated BehaviorStatus. */
   });

   /** Draw the content of this representation */
   void draw() const;
   /** Verify the data consistency */
   void verify() const,

  /** List of teammate estimates. The list only contains players that have sent at least one message in the current half
   *  and that are currently not penalized. Thus, the length of the list is variable.
   *  The list is ordered by ascending player numbers.
   *  The robot that executes this code is not in this list.
   */
  (std::vector<TeammateEstimate>) teammates,
});
