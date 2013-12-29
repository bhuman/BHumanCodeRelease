/**
 * @file CombinedWorldModel.h
 *
 * Declaration of a representation that represents a combined world model
 *
 * @author Katharina Gillmann
 */

#pragma once

#include <vector>
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Pose2D.h"

/**
* @class GaussianDistribution
* a gaussian distribution consisting of a mean and a covariance.
*/
STREAMABLE(GaussianPositionDistribution,
{
public:
  GaussianPositionDistribution(const Vector2<>& robotPosition, const Matrix2x2<>& covariance),

  (Vector2<>) robotPosition, /**< Position (mean) of the detected robot, mean is the center point in both cases (ultrasonic and vision) */
  (Matrix2x2<>) covariance, /**< covariance of the mesasurement **/
});

/**
* @class CombinedWorldModel
* a combined world model.
*/
STREAMABLE(CombinedWorldModel,
{
public:
  void draw() const,

  (std::vector<Pose2D>) positionsOwnTeam,   /**< poses of own robots */
  (std::vector<GaussianPositionDistribution>) positionsOpponentTeam, /**< positions of opponent robots */
  (BallState) ballState,                    /**< position and velocity of the ball in field coordinates. (Do not trust comments of BallState here) */
  (bool) ballIsValid,                       /**< ball state is valid, if true */

  (BallState) ballStateOthers,              /**< position and velocity of the ball as seen by teammates only */
  (bool) ballIsValidOthers,                 /**< if calculated ball state is valid as seen by teammates only */
  (float) ballStateOthersMaxSideConfidence, /**< Maximum SideConfidence of the involved robots */

  (Vector2<>) expectedEndPosition,          /**< expected end position of the ball. */
});
