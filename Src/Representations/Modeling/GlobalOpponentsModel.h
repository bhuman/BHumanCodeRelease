/**
 * @file GlobalOpponentsModel.h
 *
 * Declaration of a representation that stores the estimated states
 * of the opponent robots.
 *
 * @author Tim Laue
 */

#pragma once

#include <vector>
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct GlobalOpponentsModel
 *
 * Contains a list of all currently tracked opponent robots on the pitch.
 */
STREAMABLE(GlobalOpponentsModel,
{
  /**
   * @struct OpponentEstimate
   * Information that we have about an opponent robot.
   * Currently only the estimated position.
   */
   STREAMABLE(OpponentEstimate,
   {,
     (Vector2f) position,        /**< The estimated position of an opponent robot in absolute field coordinates  */
     (Vector2f) left,            /**< Left point of an opponent (in absolute field coordinates) */
     (Vector2f) right,           /**< Right point of an opponent (in absolute field coordinates) */
     (bool)     unidentified,    /**< Indicates that the jersey has not been seen yet -> UFO (Unidentified Football-playing Object) */
   });

   /** Draw the content of this representation */
   void draw() const;
   /** Verify the data consistency */
   void verify() const,

  (std::vector<OpponentEstimate>) opponents, /**< List of opponent estimates */
  (int) numOfPenalizedOpponents,             /**< Number of opponents that are currently off the pitch */
  (int) numOfUnknownOpponents,               /**< Number of opponents about we do not seem to have any information (should be 0 in the best case) */
});
