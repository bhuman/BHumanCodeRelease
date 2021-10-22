/**
 * @file LookAroundAfterPenaltyCard.cpp
 *
 * This file implements the behavior of a robot after it was unpenalized to make it look around.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/Math/Random.h"

CARD(LookAroundAfterPenaltyCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookLeftAndRight),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  DEFINES_PARAMETERS(
  {,
    (int)(500) minDuration, /**< This card will be active for at least this duration after a robot has been unpenalized. */
    (int)(3000) maxDuration, /**< This card will be left if the robot has been unpenalized for more than this duration. */
  }),
});

class LookAroundAfterPenaltyCard : public LookAroundAfterPenaltyCardBase
{
  bool preconditions() const override
  {
    return theExtendedGameInfo.timeSinceLastPenaltyEnded == 0;
  }

  bool postconditions() const override
  {
    return theExtendedGameInfo.timeSinceLastPenaltyEnded >= (theRobotPose.quality != RobotPose::poor ? minDuration : maxDuration);
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::lookAroundAfterPenalty);
    theLookLeftAndRightSkill(startLeft);
    theStandSkill();
  }

  void reset() override
  {
    startLeft = theTeamBallModel.isValid ? (theTeamBallModel.position.y() > 0.f) : Random::bernoulli();
  }

  bool startLeft; /**< Whether the robot should look to the left side first. */
};

MAKE_CARD(LookAroundAfterPenaltyCard);
