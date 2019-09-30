/**
 * @file OwnSideModelProvider.h
 * The file declares a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"

MODULE(OwnSideModelProvider,
{,
  REQUIRES(CognitionStateChanges),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(Odometer),
  REQUIRES(StaticInitialPose),
  USES(TeamBehaviorStatus),
  PROVIDES(OwnSideModel),
  LOADS_PARAMETERS(
  {,
    (float) distanceUncertaintyOffset, /**< Estimated odometry and localization base error (mm). */
    (float) distanceUncertaintyFactor, /**< Estimated odometry error as a factor of the distance walked. */
    (float) largestXInInitial, /**< The largest x coordinate of a robot in initial (mm). */
    (float) awayFromLineDistance, /**< The distance the robot has to be before or behind a certain line (mm). */
    (int) minPenaltyTime, /**< The minimum time a robot must be penalized to actually believe it (ms). */
    (int) minPenaltyTimeIP, /**< The minimum time a robot must be penalized for Illegal Positioning to actually believe it (ms). */
  }),
});

/**
 * @class OwnSideModelProvider
 * A module that determines whetehr the robot cannot have left its own
 * side since the last kick-off.
 */
class OwnSideModelProvider : public OwnSideModelProviderBase
{
private:
  float distanceWalkedAtKnownPosition; /**< The robot walked this far at its last known position. */
  float largestXPossibleAtKnownPosition; /**< The largest x coordinate possible at its last known position. */
  bool manuallyPlaced; /**< Was the robot manually placed in the set state? */
  unsigned timeWhenPenalized; /**< When was the robot penalized. */
  unsigned timeWhenPenaltyEnded; /**< When the previous penalty ended (i.e. the robot was unpenalized). */
  bool receivedGameControllerPacket; /**< Was a GameController packet already received? */

  void update(OwnSideModel& ownSideModel) override;

public:
  OwnSideModelProvider();
};
