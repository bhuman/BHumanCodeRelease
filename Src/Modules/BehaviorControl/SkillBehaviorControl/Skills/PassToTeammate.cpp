/**
 * @file PassToTeammate.cpp
 *
 * This file defines an implementation of the PassToTeammate skill, that chooses a suitable target position, kick type and time of execution for passing the ball to a specific player while avoiding obstacles.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Tools/Modeling/BallPhysics.h"
// CABSL must be included last:
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(PassToTeammateImpl,
{,
  IMPLEMENTS(PassToTeammate),
  CALLS(GoToBallAndKick),
  CALLS(KickAtGoal),
  CALLS(LookActive),
  CALLS(PassTarget),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(BallSpecification),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(ExpectedGoals),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(KickInfo),
  REQUIRES(MotionInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(PassEvaluation),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (std::vector<KickInfo::KickType>) allowedKicks, /**< The list of available kicks for passing */
    (bool) doSpecialKickChecks, /**< Makes kicks unavailable for certain situations (should be disabled when only a few kicks are allowed) */
    (float) minKickLengthForLong, /** < Distance to pass target must be greater than this for using the forwardFastLong during own goal kicks */
    (float) ratingThreshold, /**< During kick-off and free kicks wait with kicking the ball if the teammate's pass position rating is worse */
    (float) passAheadDistance, /**< Shift the pass target by this distance in the direction of the goal */
    (Angle) minAngleOffset, /**< Kick angle must deviate more than this from the next obstacle range in the sector wheel. This is half the minimum angular range of a free sector that the ball can be kicked through */
    (Angle) maxAngleOffset, /**< Kick angle must not deviate more than this from the original angle to the target. This is half the angular range in which free sectors are considered */
    (Angle) maxAngleOffsetBehind, /**< maxAngleOffset when the ball is passed on the own goal side (instead of the opponent's goal side) of the pass target (from the perspective of the ball) */
    (Angle) maxPrecisionOffset, /** Precision range for kick angle may not be more than +- this offset */
    (Vector2f) waitingPositionOffset, /**< Distance of waiting position to the kick position offset to the ball */
    (float) targetOffsetToFieldBorder, /**< Pass target's minimum distance to field border, to prevent ball from rolling out of its boundary */
    (int) timeLeftToAdjust, /**< Time left until the game state ends below which the robot will choose the final pass target and adjust itself accordingly behind the ball before executing the pass */
    (int) minTimeWaiting, /**< Stand still for a minimum of this many seconds */
    (int) maxTimeWaiting, /**< Stand still for a maximum of this many seconds */
    (float) kickRangeOffset, /**< Minimum and maximum range of the forwardFastPass is extended by this */
    (float) averageKickExecutionTime, /**< Time it takes to perform most kick motions (from the KickInfo) */
    (Vector2f) averageKickBallOffset, /**< Position behind the ball needed to perform most kick motions (from the KickInfo) */
    (float) ignoreObstaclesThreshold, /**< Obstacle avoidance will be disabled when this close to the waiting position */
    (float) ignoreDynamicObstaclesThreshold, /**< Obstacle avoidance will be disabled when this close to the waiting position */
    (bool) lookAhead, /**< Estimate the teammate's position into the future */
    (float) maxLookAheadTime, /**< When lookAhead, estimate the teammate's position a maximum of this many milliseconds into the future */
    (Rangef) interceptionDistanceThreshold, /** < Thresholds to check whether or not the receiver is close enough to the kick target to intercept the ball in time */
    (Rangef) targetDistanceScale, /** < Length thresholds for a linear interpolation of the pass distance to the above parameter */
  }),
});

class PassToTeammateImpl : public PassToTeammateImplBase
{
  const GlobalTeammatesModel::TeammateEstimate* teammate = nullptr;
  Vector2f teammatePosition = Vector2f::Zero();
  bool isAngleFree = false;
  bool isTargetFree = false;
  Angle kickAngle = 0_deg;
  float kickLength = 0.f;
  Vector2f kickTarget = Vector2f::Zero();
  Rangea precisionRange = Rangea(0_deg, 0_deg);
  KickInfo::KickType lastKickType = KickInfo::numOfKickTypes;
  int lastPassTarget = -1;
  Vector2f lastKickTarget = Vector2f::Zero();
  const Vector2f opponentGoal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
  const Vector2i bottomLeftCorner = Vector2i(theFieldDimensions.xPosOwnGroundLine + targetOffsetToFieldBorder,
                                             theFieldDimensions.yPosRightSideline + targetOffsetToFieldBorder);
  const Vector2i topRightCorner = Vector2i(theFieldDimensions.xPosOpponentGroundLine - targetOffsetToFieldBorder,
                                           theFieldDimensions.yPosLeftSideline - targetOffsetToFieldBorder);

  void preProcess() override {}
  void preProcess(const PassToTeammate&) override
  {
    DECLARE_DEBUG_DRAWING("skill:PassToTeammate:target", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:PassToTeammate:evaluation", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:PassToTeammate:precision", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:PassToTeammate:wheel", "drawingOnField");
  }

  option(PassToTeammate)
  {
    // Find the player to which the ball should be passed
    for(const auto& t : theGlobalTeammatesModel.teammates)
    {
      if(t.playerNumber == p.playerNumber)
      {
        teammate = &t;
        break;
      }
    }
    if(!teammate)
    {
      p.setState("invalidPassTarget");
      theKickAtGoalSkill();
      reset();
      return;
    }

    common_transition
    {
      // In regular playing, always kick the ball immediately (without waiting)
      if(!isFreeKick() && !isKickOff())
        goto kick;
      // When a free kick started this frame, reset member variables and take the transitions from the initial state
      if(theGameState.state != theExtendedGameState.stateLastFrame)
      {
        reset();
        if(canWait(minTimeWaiting + timeLeftToAdjust))
          goto walk;
        else
          goto adjust;
      }
    }

    // When the skill started this frame, reset member variables and decide whether to wait or adjust before executing the kick
    initial_state(initial)
    {
      transition
      {
        reset();
        if(canWait(minTimeWaiting + timeLeftToAdjust))
          goto walk;
        else
          goto adjust;
      }
    }

    // Walk to the waiting position while calculating the kick target once
    state(walk)
    {
      transition
      {
        // When the waiting position is reached, wait behind the ball, if there's time to stand, otherwise adjust before executing the kick
        if(theWalkToPointSkill.isDone() &&
           canWait(minTimeWaiting + timeLeftToAdjust))
          goto wait;
        if(!canWait(timeLeftToAdjust))
          goto adjust;
      }
      action
      {
        // Calculate the kick target and kick type in the first frame of this state and when the passed-to player changed
        // TODO: Adjust the kick target continously if it doesn't change the planned path to the waiting position
        if(state_time == 0 || !keepLastKickTarget())
        {
          updateKickTarget();
          lastKickType = selectKickType();
        }
        else
          updateKickParameters();
        const Pose2f targetPose = calculateWaitingPose();
        theWalkToPointSkill({ .target = targetPose,
                              .reduceWalkingSpeed = false,
                              .rough = targetPose.translation.squaredNorm() <= sqr(ignoreObstaclesThreshold),
                              .disableObstacleAvoidance = targetPose.translation.squaredNorm() <= sqr(ignoreDynamicObstaclesThreshold),
                              .disableAvoidFieldBorder = true});
        theLookActiveSkill({.withBall = true});
      }
    }

    // Stand at the waiting position behind the ball as long as there's is still enough time left and the pass situation could improve
    state(wait)
    {
      transition
      {
        updateTargetRating();
        if(!canWait(timeLeftToAdjust) ||
           state_time > maxTimeWaiting ||
           (state_time > minTimeWaiting &&
            (!keepLastKickTarget() ||
             (isAngleFree && isTargetFree))))
          goto adjust;
      }
      action
      {
        theStandSkill();
        theLookActiveSkill({.withBall = true});
        lastKickType = KickInfo::numOfKickTypes;
      }
    }

    // Calculate the kick target once, adjust to the new waiting position and keep walking, then execute the kick
    state(adjust)
    {
      transition
      {
        // When the ideal orientation is reached, execute the kick
        if(theWalkToPointSkill.isDone())
          goto kick;
      }
      action
      {
        // Calculate the kick target and kick type in the first frame of this state and when the passed-to player changed
        if(state_time == 0 || !keepLastKickTarget())
        {
          updateKickTarget();
          lastKickType = selectKickType();
        }
        else
          updateKickParameters();
        const Pose2f targetPose = calculateWaitingPose();
        theWalkToPointSkill({.target = targetPose,
                             .reduceWalkingSpeed = false,
                             .rough = targetPose.translation.squaredNorm() <= sqr(ignoreObstaclesThreshold),
                             .disableObstacleAvoidance = targetPose.translation.squaredNorm() <= sqr(ignoreDynamicObstaclesThreshold),
                             .disableStanding = true,
                             .disableAvoidFieldBorder = true});
        theLookActiveSkill({.withBall = true});
      }
    }

    // Execute the kick immediately
    state(kick)
    {
      action
      {
        // Keep the last kick target during a set play
        if(!isFreeKick() && !isKickOff())
          updateKickTarget();
        else
          updateKickParameters();
        // Set the most suitable kick and its parameters, then execute it
        auto kickType = selectKickType();
        const auto alignPrecisely = (kickType == KickInfo::walkForwardsRightLong ||
                                     kickType == KickInfo::walkForwardsLeftLong ||
                                     kickType == KickInfo::walkTurnRightFootToLeft ||
                                     kickType == KickInfo::walkTurnLeftFootToRight) ?
                                    KickPrecision::precise :
                                    KickPrecision::notPrecise;
        const bool preStepAllowed = !(kickType == KickInfo::walkForwardsLeft ||
                                      kickType == KickInfo::walkForwardsRight);
        theGoToBallAndKickSkill({.targetDirection = kickAngle,
                                 .kickType = kickType,
                                 .alignPrecisely = alignPrecisely,
                                 .length = kickLength,
                                 .preStepAllowed = preStepAllowed,
                                 .turnKickAllowed = preStepAllowed,
                                 .directionPrecision = precisionRange});
        thePassTargetSkill({.passTarget = p.playerNumber,
                            .ballTarget = theRobotPose.inversePose * kickTarget});
        lastKickType = kickType;
      }
    }

    lastPassTarget = teammate->playerNumber;
    lastKickTarget = kickTarget;
    draw();
  }

  // Check whether or not there is still enough time left to wait during any restart in play (free kick, corner kick, goal kick, kick-in, kick-off)
  bool canWait(const int timeLeft) const
  {
    // TODO: Add interpolated threshold for time left in game and state instead of hard threshold
    return (isFreeKick() || isKickOff()) &&
           theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds) < -timeLeft &&
           theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds) < -timeLeft * 12.f; // TODO: Improve this to gradually decrease the waiting time with time left in the game
  };

  // Modify the kick target when the receiver changed from last frame or there was no last kick target
  bool keepLastKickTarget() const
  {
    return lastPassTarget == teammate->playerNumber &&
           lastKickTarget != Vector2f::Zero();
  }

  // Calculate the target pose in robot-relative coordinates for waiting behind the ball to minimize adjustment steps when executing the kick later on
  Pose2f calculateWaitingPose() const
  {
    // Use parameters from the kick info to find the preferred waiting position behind the ball
    const Angle kickRotationOffset = lastKickType != KickInfo::numOfKickTypes ?
                                     theKickInfo[lastKickType].rotationOffset :
                                     0_deg;
    const Vector2f kickBallOffset = waitingPositionOffset + (lastKickType != KickInfo::numOfKickTypes ?
                                                             theKickInfo[lastKickType].ballOffset :
                                                             Vector2f::Zero());
    return Pose2f(kickAngle, theFieldBall.endPositionRelative).rotate(kickRotationOffset).translate(kickBallOffset);
  }

  // Calculates a kick target as close to the teammate's position as possible while reducing the likelihood of obstacles blocking the ball on the way
  void updateKickTarget()
  {
    const Vector2f& ballPositionOnField = theFieldBall.endPositionOnField;
    // Calculates the angular sectors of the known opponents from the obstacle model
    const std::list<SectorWheel::Sector>& sectors = calculateObstacleSectors(ballPositionOnField);

    teammatePosition = teammate->pose.translation;
    if(lookAhead)
    {
      // Approximate the time the pass will take by estimating the time to reach kick position, kick execution time and ball rolling time
      const Angle estimatedKickAngle = Angle::normalize((teammatePosition - ballPositionOnField).angle() - theRobotPose.rotation);
      const Pose2f estimatedKickPose = Pose2f(estimatedKickAngle, ballPositionOnField).translate(averageKickBallOffset);
      const Vector2f estimatedBallVelocity(BallPhysics::velocityForDistance((teammatePosition - ballPositionOnField).norm(), theBallSpecification.friction), 0.f);
      const float estimatedBallRollingTime = BallPhysics::computeTimeUntilBallStops(estimatedBallVelocity, theBallSpecification.friction) * 1000.f;
      const float timeToReachKickPosition = KickSelection::calcTTRP(estimatedKickPose, ballPositionOnField, theWalkingEngineOutput.maxSpeed);
      const unsigned int estimatedPassExecutionTime = static_cast<unsigned int>(clip(timeToReachKickPosition + averageKickExecutionTime + estimatedBallRollingTime, 0.f, maxLookAheadTime));
      teammatePosition = teammate->getFuturePosition(estimatedPassExecutionTime);
    }

    // Determine whether the ball should be ideally passed to the left (=counterclockwise) or right (=clockwise) side of the pass target (from the perspective of the ball), so that the ball ends up between the receiver and the opponent's goal
    const Geometry::Line ballGoalLine(ballPositionOnField, opponentGoal - ballPositionOnField);
    const float distToLine = Geometry::getDistanceToLineSigned(ballGoalLine, teammatePosition);
    const bool passToLeftSide = distToLine > 0.f;
    const Vector2f targetPosition = (teammatePosition + (opponentGoal - teammatePosition).normalized(passAheadDistance));
    // TODO: Large rotation of target around ball will place the modified target away from the target to goal line, so the range should be extended accordingly
    float targetDistance = (targetPosition - ballPositionOnField).norm();
    Angle targetAngle = (targetPosition - ballPositionOnField).angle();

    // During kick-off, the target angle will not be modified because obstacles can be ignored (there can not legally be any opponents in the own half)
    isAngleFree = isKickOff();
    // When the initial target angle is blocked, it is shifted to a free sector (not blocked by obstacles) in the direction of the opponent's goal first (ideally placing it in front of the receiver)
    if(!isAngleFree)
    {
      Angle firstSideAngle = targetAngle;
      Angle secondSideAngle = targetAngle;
      // Calculate the clostest angles in both directions and pick the one with be best combined target rating. Prefer first angle, because a pass to the goal side of the teammate is more "natural".
      const bool isFirstSideValid = findNextFreeAngle(sectors, firstSideAngle, precisionRange, passToLeftSide, maxAngleOffset);
      const bool isSecondSideValid = findNextFreeAngle(sectors, secondSideAngle, precisionRange, !passToLeftSide, maxAngleOffsetBehind);
      isAngleFree |= isFirstSideValid || isSecondSideValid;
      if(isFirstSideValid && isSecondSideValid)
      {
        // When both sides of the receiver can be passed to, evaluate the resulting target positions to determine which side would be better
        const float firstSideRating = getAngleRating(firstSideAngle, true, ballPositionOnField, targetAngle, targetDistance);
        const float secondSideRating = getAngleRating(secondSideAngle, false, ballPositionOnField, targetAngle, targetDistance);
        if(firstSideRating >= secondSideRating)
          targetAngle = firstSideAngle;
        else
          targetAngle = secondSideAngle;
      }
      else if(isFirstSideValid)
        targetAngle = firstSideAngle;
      else if(isSecondSideValid)
        targetAngle = secondSideAngle;
    }

    kickTarget = ballPositionOnField + Vector2f::polar(targetDistance, targetAngle);
    // TODO: Clipping the angle inside the field plus offset could lead to the target angle being blocked by an obstacle
    Geometry::clipPointInsideRectangle(bottomLeftCorner, topRightCorner, kickTarget);
    updateTargetRating();
    updateKickParameters();
  }

  // Calculates the rating thresholds of the target position for the decision to wait behind the ball or kick it
  void updateTargetRating()
  {
    float interpolatedRatingThreshold = ratingThreshold;
    const int timeInState = theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted);
    const int timeInStateLeft = theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
    // Linear interpolation of state progress to required pass rating from maximum value to 0.f
    if(isFreeKick() || isKickOff())
    {
      const int totalStateTime = std::max(timeInState - timeInStateLeft, 1);
      const float stateProgress = static_cast<float>(timeInState) / static_cast<float>(totalStateTime);
      interpolatedRatingThreshold *= (1.f - clip(stateProgress, 0.f, 1.f));
    }
    isTargetFree = thePassEvaluation.getRating(kickTarget) > std::max(interpolatedRatingThreshold, 0.01f);
  }

  void updateKickParameters()
  {
    kickAngle = Angle::normalize((kickTarget - theFieldBall.endPositionOnField).angle() - theRobotPose.rotation);
    kickLength = (kickTarget - theFieldBall.endPositionOnField).norm();
  }

  // Calculates the angular sectors of the known opponents from the obstacle model
  std::list<SectorWheel::Sector> calculateObstacleSectors(const Vector2f& ballPositionOnField) const
  {
    SectorWheel sectorWheel;
    sectorWheel.begin(ballPositionOnField);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      if(obstacle.isTeammate() || obstacle.isGoalpost())
        continue;
      const Vector2f obstacleOnField = theRobotPose * obstacle.center;
      if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGroundLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
        continue;
      const float obstacleWidth = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
      const float obstacleDistance = std::sqrt(std::max((obstacleOnField - ballPositionOnField).squaredNorm() - sqr(obstacleWidth / 2.f), 1.f));
      if(obstacleDistance < theBallSpecification.radius)
        continue;
      const float obstacleRadius = std::atan(obstacleWidth / (2.f * obstacleDistance));
      const Angle obstacleDirection = (obstacleOnField - ballPositionOnField).angle();
      sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection - obstacleRadius), Angle::normalize(obstacleDirection + obstacleRadius)), obstacleDistance, SectorWheel::Sector::obstacle);
    }
    return sectorWheel.finish();
  }

  // Calculates the rating for a target position based on the rating functions for passing and shooting at the opponent's goal
  float getAngleRating(const Angle candidateAngle, const bool isIdealSide, const Vector2f basePosition, const Angle targetAngle, const float targetDistance) const
  {
    const Vector2f targetPosition = basePosition + Vector2f::polar(targetDistance, candidateAngle);
    // Rating of diviation of new angle to the original angle
    const Angle maxAngleDeviation = isIdealSide ? maxAngleOffset : maxAngleOffsetBehind;
    const float offsetRating = mapToRange(targetAngle.diffAbs(candidateAngle), 0_deg, maxAngleDeviation, isIdealSide ? Angle(1.f) : Angle(0.6f), 0_deg);
    const float positionRating = thePassEvaluation.getRating(targetPosition);
    const float goalRating = theExpectedGoals.getRating(targetPosition);
    const float combinedRating = offsetRating * positionRating * goalRating;
    COMPLEX_DRAWING("skill:PassToTeammate:evaluation")
    {
      const Vector2f& ballPositionOnField = theFieldBall.endPositionOnField;
      LINE("skill:PassToTeammate:evaluation", ballPositionOnField.x(), ballPositionOnField.y(), targetPosition.x(), targetPosition.y(), 20, Drawings::PenStyle::solidPen, ColorRGBA::violet);
      DRAW_TEXT("skill:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y(), 200, ColorRGBA::violet, "p: " << positionRating);
      DRAW_TEXT("skill:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 250, 200, ColorRGBA::violet, "g: " << goalRating);
      DRAW_TEXT("skill:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 500, 200, ColorRGBA::violet, "o: " << offsetRating);
      DRAW_TEXT("skill:PassToTeammate:evaluation", targetPosition.x(), targetPosition.y() - 750, 200, ColorRGBA::violet, "c: " << combinedRating);
    }
    return combinedRating;
  }

  // Sets the closest target angle and precision range by reference, if a free angle (not blocked by obstacles) could be found in the sector wheel with the given parameters
  bool findNextFreeAngle(const std::list<SectorWheel::Sector>& sectors, Angle& targetAngle, Rangea& precisionRange, const bool ccw, const Angle maxAngleDeviation)
  {
    for(auto it = sectors.begin(); it != sectors.end(); ++it)
    {
      if(it->angleRange.isInside(targetAngle))
      {
        // Correct the initial angle if it is too close to the edge of the angle range (too close to the obstacle to successfully pass by)
        const bool tooCloseMinBorder = targetAngle.diffAbs(it->angleRange.min) < minAngleOffset;
        const bool tooCloseMaxBorder = targetAngle.diffAbs(it->angleRange.max) < minAngleOffset;
        Angle correctedAngle = targetAngle;
        if(tooCloseMinBorder || tooCloseMaxBorder)
          correctedAngle = getNextAngleInRange(it->angleRange, tooCloseMinBorder);
        const bool skipOriginalSector = ccw ? tooCloseMaxBorder : tooCloseMinBorder;
        if(!skipOriginalSector &&
           it->type == SectorWheel::Sector::free &&
           setAnglePrecisionRange(targetAngle, precisionRange, correctedAngle, maxAngleDeviation, it->angleRange))
          return true;
        auto origIt = it;
        if(ccw)
        {
          do
          {
            ++it;
            if(it == sectors.end())
              it = sectors.begin();
            if(it->type == SectorWheel::Sector::free &&
               setAnglePrecisionRange(targetAngle, precisionRange, getNextAngleInRange(it->angleRange, ccw), maxAngleDeviation, it->angleRange))
              return true;
          }
          while(it != origIt);
        }
        else
        {
          do
          {
            if(it == sectors.begin())
              it = sectors.end();
            --it;
            if(it->type == SectorWheel::Sector::free &&
               setAnglePrecisionRange(targetAngle, precisionRange, getNextAngleInRange(it->angleRange, ccw), maxAngleDeviation, it->angleRange))
              return true;
          }
          while(it != origIt);
        }
        break;
      }
    }
    return false;
  }

  // Calculates the angle inside a range that is closest in the (counter)clockwise direction. If the angle range is too small, its center is used.
  Angle getNextAngleInRange(const Rangea angleRange, const bool ccw) const
  {
    const Angle halfSizeOfRange = angleRange.getSize() / 2.f;
    const Angle angleOffset = std::min(minAngleOffset, halfSizeOfRange);
    return ccw ?
           angleRange.min + angleOffset :
           angleRange.max - angleOffset;
  }

  // Sets the target angle, when it is valid (i.e. within the given parameters)
  bool setAnglePrecisionRange(Angle& targetAngle, Rangea& precisionRange, const Angle currentAngle, const Angle maxAngleDeviation, const Rangea& angleRange) const
  {
    /** No final kick angle will be set, when one of the following conditions is met:
        - the free angle range is too small
        - the angle is too far from the original angle
        - the angle is outside of the free angle range
     */
    const Angle halfSizeOfRange = angleRange.getSize() / 2.f;
    if(halfSizeOfRange < minAngleOffset ||
       targetAngle.diffAbs(currentAngle) > maxAngleDeviation ||
       !angleRange.isInside(currentAngle))
      return false;
    targetAngle = currentAngle;

    // TODO: Refactor this code block to be much simpler
    precisionRange = angleRange;
    Rangea borderToMinMax(precisionRange.min + maxPrecisionOffset,
                          precisionRange.max - maxPrecisionOffset);
    if(precisionRange.max < precisionRange.min)
    {
      if(targetAngle > precisionRange.min)
        borderToMinMax.max = 180_deg;
      else
        borderToMinMax.min = -180_deg;
    }
    borderToMinMax = Rangea(std::min(targetAngle, borderToMinMax.min),
                            std::max(targetAngle, borderToMinMax.max));
    precisionRange.max = std::max(0.f, borderToMinMax.limit(targetAngle + maxPrecisionOffset) - targetAngle);
    precisionRange.min = std::min(0.f, borderToMinMax.limit(targetAngle - maxPrecisionOffset) - targetAngle);
    return true;
  }

  // Selects the kick type that would move the ball most quickly and accurately to the pass target based on the kick's range and the time to reach its required pose behind the ball
  KickInfo::KickType selectKickType() const
  {
    KickInfo::KickType bestKick = KickInfo::numOfKickTypes;
    float bestRangeDeficiency = std::numeric_limits<float>::max();
    float bestTTRP;
    const auto kicks = calcAvailableKicks();
    for(const KickInfo::KickType kickType : kicks)
    {
      const KickInfo::Kick& kick = theKickInfo[kickType];
      const float rangeDeficiency = kick.range.isInside(kickLength) ? 0.f :
                                    std::max(0.f, (kickLength < kick.range.min ?
                                                   (kick.range.min - kickLength) :
                                                   (kickLength - kick.range.max)) - (kickType == lastKickType ? 200.f : 0.f));
      if(rangeDeficiency < bestRangeDeficiency)
      {
        bestRangeDeficiency = rangeDeficiency;
        bestTTRP = std::numeric_limits<float>::max();
      }
      else if(rangeDeficiency > bestRangeDeficiency)
        continue;
      const Pose2f kickPose = Pose2f(kickAngle, theFieldBall.endPositionRelative).rotate(kick.rotationOffset).translate(kick.ballOffset);
      const float ttrp = KickSelection::calcTTRP(kickPose, theFieldBall.endPositionRelative, theWalkingEngineOutput.maxSpeed) - (kickType == lastKickType ? 500.f : 0.f);
      if(ttrp < bestTTRP)
      {
        bestKick = kickType;
        bestTTRP = ttrp;
      }
    }
    return bestKick;
  }

  // Filters the allowed kicks based on the current situation to improve the overall accuracy of passing
  std::vector<KickInfo::KickType> calcAvailableKicks() const
  {
    std::vector<KickInfo::KickType> kicks;
    kicks.reserve(KickInfo::numOfKickTypes);
    // Check for each allowed kick whether or not it should be made available in the given situation
    for(const KickInfo::KickType kickType : allowedKicks)
    {
      const KickInfo::Kick& kick = theKickInfo[kickType];
      // Makes some kicks unavailable or even mandatory for certain situations
      if(doSpecialKickChecks)
      {
        switch(kickType)
        {
          case KickInfo::forwardFastRightLong:
          case KickInfo::forwardFastLeftLong:
          {
            // Skip this kick if it requires standing on a weak leg of the robot
            if(kick.motion == MotionPhase::kick &&
               theDamageConfigurationBody.sides[kick.mirror ? Legs::right : Legs::left].weakLeg)
              continue;
            // This kick requires standing still and should only be used for goal kicks
            if(isGoalKick() && kickLength >= minKickLengthForLong - (kickType == lastKickType ? 400.f : 0.f))
              return {KickInfo::forwardFastRightLong, KickInfo::forwardFastLeftLong};
            continue;
          }
          case KickInfo::forwardFastRight:
          case KickInfo::forwardFastLeft:
          {
            // Skip this kick if it requires standing on a weak leg of the robot
            if(kick.motion == MotionPhase::kick &&
               theDamageConfigurationBody.sides[kick.mirror ? Legs::right : Legs::left].weakLeg)
              continue;
            // This kick requires standing still and should only be used and preferred during any restart in play (free kick, corner kick, goal kick, kick-in) except for kick-offs
            if(!isFreeKick() && !isKickOff())
              continue;
            // This kick has a high chance of the ball rolling further than planned and therefore should only be used for kicking directly at the receiver
            if(canTeammateInterceptBall())
              return {KickInfo::forwardFastRight, KickInfo::forwardFastLeft};
            continue;
          }
          case KickInfo::forwardFastRightPass:
          case KickInfo::forwardFastLeftPass:
          {
            // Skip this kick if it requires standing on a weak leg of the robot
            if(kick.motion == MotionPhase::kick &&
               theDamageConfigurationBody.sides[kick.mirror ? Legs::right : Legs::left].weakLeg)
              continue;
            // This kick requires standing still and should only be used and preferred during any restart in play (free kick, corner kick, goal kick, kick-in) except for kick-offs
            if(!isFreeKick() && !isKickOff())
              continue;
            const float kickRangeExtension = kickRangeOffset + (kickType == lastKickType ? 200.f : 0.f);
            // This kick should only be used for distances close to the minimum or maximum of it's range
            const Rangef minRange(kick.range.min - kickRangeExtension, kick.range.min + kickRangeExtension);
            const Rangef maxRange(kick.range.max - kickRangeExtension, kick.range.max + kickRangeExtension);
            if(minRange.isInside(kickLength) ||
               maxRange.isInside(kickLength))
              return {KickInfo::forwardFastRightPass, KickInfo::forwardFastLeftPass};
            continue;
          }
          case KickInfo::walkForwardsRightLong:
          case KickInfo::walkForwardsLeftLong:
          {
            // This kick should be substituted for the alternative version for kick-offs to prevent the ball from leaving the field
            if(isKickOff())
              continue;
          }
          default:
            break;
        }
      }
      kicks.push_back(kickType);
    }
    return kicks;
  }

  // Estimate whether or not the receiver is close enough to the kick target to intercept the ball in time (usually by walking sidewards)
  bool canTeammateInterceptBall() const
  {
    const Vector2f& ballPosition = theFieldBall.endPositionOnField;
    const Geometry::Line passTrajectory(ballPosition, kickTarget - ballPosition);
    const float interceptionDistance = Geometry::getDistanceToLine(passTrajectory, teammatePosition);
    const float maxDistanceThreshold = mapToRange(kickLength, targetDistanceScale.min, targetDistanceScale.max, interceptionDistanceThreshold.min, interceptionDistanceThreshold.max);
    // Note that no hysteresis is required here (until proven otherwise), because this will only be calculated twice (in the 'walk' and 'adjust' states) during a free kick
    return interceptionDistance <= maxDistanceThreshold;
  }

  bool isKickOff() const { return theGameState.isKickOff() && theGameState.isForOwnTeam(); }
  bool isFreeKick() const { return theGameState.isFreeKick() && theGameState.isForOwnTeam(); }
  bool isGoalKick() const { return theGameState.isGoalKick() && theGameState.isForOwnTeam(); }

  void draw()
  {
    COMPLEX_DRAWING("skill:PassToTeammate:target")
    {
      const Vector2f& ballPositionOnField = theFieldBall.endPositionOnField;
      CROSS("skill:PassToTeammate:target", teammatePosition.x(), teammatePosition.y(), 100, 20, Drawings::solidPen, isTargetFree ? ColorRGBA::blue : ColorRGBA::red);
      ARROW("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), kickTarget.x(), kickTarget.y(), 20, Drawings::PenStyle::solidPen, isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
    }
    COMPLEX_DRAWING("skill:PassToTeammate:precision")
    {
      const Vector2f& ballPositionOnField = theFieldBall.endPositionOnField;
      const Vector2f ballToTarget = kickTarget - ballPositionOnField;
      const Vector2f ballToTeammate = teammatePosition - ballPositionOnField;
      const Vector2f precisionLeft = ballPositionOnField + ballToTarget.rotated(precisionRange.min);
      const Vector2f precisionRight = ballPositionOnField + ballToTarget.rotated(precisionRange.max);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), precisionLeft.x(), precisionLeft.y(), 20, Drawings::PenStyle::dashedPen, isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), precisionRight.x(), precisionRight.y(), 20, Drawings::PenStyle::dashedPen, isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
      const Vector2f maxAngleLeft = ballPositionOnField + ballToTeammate.rotated(maxAngleOffset);
      const Vector2f maxAngleRight = ballPositionOnField + ballToTeammate.rotated(-maxAngleOffset);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleLeft.x(), maxAngleLeft.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::red);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleRight.x(), maxAngleRight.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::red);
      const Vector2f maxAngleLeftBehind = ballPositionOnField + ballToTeammate.rotated(maxAngleOffsetBehind);
      const Vector2f maxAngleRightBehind = ballPositionOnField + ballToTeammate.rotated(-maxAngleOffsetBehind);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleLeftBehind.x(), maxAngleLeftBehind.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::orange);
      LINE("skill:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleRightBehind.x(), maxAngleRightBehind.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::orange);
    }
    COMPLEX_DRAWING("skill:PassToTeammate:wheel")
    {
      const Vector2f& ballPositionOnField = theFieldBall.endPositionOnField;
      DRAW_SECTOR_WHEEL("skill:PassToTeammate:wheel", calculateObstacleSectors(ballPositionOnField), ballPositionOnField);
    }
  }

  void reset()
  {
    kickAngle = 0_deg;
    kickLength = 0.f;
    kickTarget = Vector2f::Zero();
    precisionRange = Rangea(0_deg, 0_deg);
    lastKickType = KickInfo::numOfKickTypes;
    lastPassTarget = -1;
    lastKickTarget = Vector2f::Zero();
  }
};

MAKE_SKILL_IMPLEMENTATION(PassToTeammateImpl);
