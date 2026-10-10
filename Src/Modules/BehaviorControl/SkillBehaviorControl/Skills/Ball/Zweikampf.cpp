/**
 * @file Zweikampf.cpp
 *
 * This file implements a card that duels with an opponent.
 *
 * The general procedure is as follows:
 * - Determine some kick angles based on the last kick direction
 * - For each direction, calculate the field ratings for different kick ranges (TODO only calculate ranges that are actually used for the current direction to safe computation)
 * - For each allowed kick type, determine the best field rating and apply some more ratings and hysteresis, such as:
 *   - avoid kicks that push us closer to an obstacle
 *   - worse rating for kicks with a different direction
 *   - bonus for the same kick type for goals
 * - All kicks are sorted based on their priority: goal > steal (forwardSteal kick, that is our V-Form kick) > pass > everything else
 * - Best kick is selected
 * - Afterwards calculate some precision range, to make the execution easier
 *
 * Future TODOs:
 * - If the behavior says, we want to pass, then ignore possible goal shots and soften the forwardSteal condition.
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"
#include "Framework/ModuleGraphRunner.h"
#include "Platform/SystemCall.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include "MathBase/Approx.h"
#include <cmath>

namespace Zweikampf
{
  STREAMABLE(SearchParameters,
  {,
    (int) numOfAnglesNearBestDuelPose, /**< Number of direction sample for both sides around the last calculated kick direction.*/
    (Angle) rangeOfBestDuelPose, /**< Range for the sampling around the last calculated kick direction.*/
    (int) numOfOverallSearch, /**< Number of direction samples outside of the rangeOfBestDuelPose.*/
    (Angle) overallSearchRange, /**< Search range for additional kick directions.*/
    (Angle) bonusForStealBallDirectionAdjustmentSmall, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
    (Angle) bonusForStealBallDirectionAdjustmentBig, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
    (int) moreSearchAfterDoingNothing, /**< After this much time passed without an InWalkKick was possible, some more goal shot angles are added.*/
    (Angle) goalSectorWidth, /**< A goal kick must have at least this much width in the goal sector.*/
    (Rangea) minMaxAngleAngleRange, /**< The allowed robot rotation adjustment for the kick direction is interpolated between this range.*/
    (Angle) goalShotBufferAngle, /**< If possible use this buffer to the goal posts as buffer zone. */
  });

  STREAMABLE(DuelRatings,
  {,
    // rating modifiers
    (float) ratingMinMaxDifference, /**< Add this much malus, the more the robot must rotate for the kick direction. */
    (float) ratingBallLandsInOwnHalf, /**< Add this malus, if the ball would land in the own half. */
    (float) ratingPoseBlocked, /**< Add this malus, if the kick pose is blocked by an obstacle. */
    (float) ratingPassingToOpponent, /**< Add this malus the closer the ball is passed towards an opponent. */

    (float) ratingSameKick, /**< Add this bonus, if the checked kick is the same as the last best kick. */
    (float) ratingSameKickAngle, /**< Add this bonus, if the checked kick is the same as the last best kick and the direction is similar. */
    (float) ratingStealBall, /**< removes ratingBallLandsInOwnHalf and adds some bonus too. */
    (float) ratingStealBallKick, /**< Add this bonus for steal ball kicks.*/
    (float) ratingStealBallKickBetterSide, /**< Add this bonus for the forwardSteal, to get the better side.*/
    (float) ratingGoalShot, /**< Add this bonus if the kick can reach the goal.*/
    (float) ratingGoalShotBuffer, /**< Add this bonus if there is some buffer range to the goal post or obstacles. */
  });

  STREAMABLE(DuelTimings,
  {,
    (int) maxTimeDoingNothing, /**< After this much time passed without an InWalkKick was possible, the Zweikampf deactivates itself.*/
    (int) inactiveAfterDoingNothing, /**< After the Zweikampf deactivated itself because of doing nothing, this much time must pass before going active again.*/
    (int) sidewardRestrictionTime, /**< After the sideward kick was restricted once, so much time must have passed after no restriction is needed anymore, before the restriction is actually disabled again.*/
    (int) noKickMinTime, /**< If the last planned kick was not the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
    (int) noKickStealMinTime, /**< If the last planned kick was the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
    (int) ignoreSkillRequestTime, /**< Ignore the pass skill request for this long, if we decided we can not execute it. */
  });

  STREAMABLE(ObstacleHandling,
  {,
    (Vector2f) leftFootEdge, /**< Left foot edge.*/
    (Vector2f) rightFootEdge, /**< Right foot edge.*/
    (Vector2f) leftFootKickEdge, /**< Left foot edge after the InWalkKick.*/
    (Vector2f) rightFootKickEdge, /**< Right foot edge after the InWalkKick.*/
    (float) maxObstacleDistanceForWalkStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
    (float) maxObstacleDistanceForSideStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
    (float) obstacleDistanceForBlockingPose, /**< Calculate where the obstacle could be, when we execute the kick. This is used to decide, if one kick is worse than the mirror one. */
    (float) noKickBlockingPose, /**< distance to ball, when no kick was calculated. */
    (float) obstacleMinDistanceForBackPass, /**< The duel obstacle must be this far away to allow for a back pass from the skill request. */
    (Rangef) obstacleShiftRange, /**< Obstacles, that are this near the ball, are shifted behind the ball.*/
    (float) maxObstacleDistanceToBallForRiskyKicks,
    (float) maxObstacleDistanceToForceForwardSteal,
    (float) obstacleDistanceThreshold, /**< Towards obstacles, we want to keep a safe range to prevent passing to an opponent. Scale  */
    (float) goalSectorShiftFactor, /**< For goal shots, shift direction by this factor times sector width towards center. */
  });

  STREAMABLE(StealBallParameters,
  {,
    (Angle) stealBallTypeTooMuchRotationMinValue, /**< StealBall kick types are allowed to have higher needed rotation. */
    (Angle) bonusForStealBallMinMaxAngle,
    (Rangea) stealBallRange, /**< Field directions get a bonus, if they are in this range.*/ // TODO After Simulation Evaluation set max angle to 90 deg
    (Angle) stealBallMinRobotRotation, /**< The own rotation must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
    (Angle) stealBallMinPositionRotation, /**< The ball position angle on the field must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
    (Rangea) forwardStealBallOpponentPositionNormal, /**< The obstacle must have this angle relative to the ball, to allow the forwardSteal InWalkKick.*/
    (Angle) forwardStealBallOpponentPositionMalus, /**< The obstacle must have this angle relative to the ball, to allow the forwardSteal InWalkKick, if the last planned InWalkKick was no forwardSteal.*/
  });

  STREAMABLE(LastKickHysteresis,
  {,
    (float) kickLengthHysteresis, /**< Max kick length hysteresis */
    (Angle) kickDirectionHysteresis, /**< Interpolate max kick length hysteresis only if the direction is similar to previous kick. */
  });

  STREAMABLE(DistanceParameters,
  {,
    (KickInfo::KickType) mainKick,
    (int) numberOfSubRanges,
    (std::vector<float>) additionalRanges,
  });

  STREAMABLE(StealKickReplacementParameter,
  {,
    (Angle) staticShift, /**< Static direction shift for steal kick replacement. */
    (Rangef) distanceToFieldBorderRange, /**< Reduce kick strength based on field border distance. */
    (Rangef) interpolateDirectionGoalDistance, /**< Interpolate replacement kick direction towards opponent goal, closer to the goal. */
  });
  /**
   * The different types of kicks. Higher ups, like goal shots, are always better than lower types, even if the rating is worse.
   * That way, a goal kick will always be done as long as it is possible.
   */
  ENUM(TargetType,
  {,
    goalShot,
    goalDribbleShot,
    stealBall,
    pass,
    other,
  });

  struct DuelPose
  {
    float rating = 0.f; /**< Rating of the kick. Small == good, high == bad. */
    Angle kickAngle = 0_deg; /**< Kick direction. */
    Pose2f pose; /**< Kick pose. */
    bool noKick = true; /**< Is a kick requested? */
    bool preStepAllowed = true; /**< Is a pre step allowed for the kick? */
    KickInfo::KickType kickType = KickInfo::rlKickDynamic; /**< Type of the kick. */
    Rangea precision = Rangea(0_deg, 0_deg); /**< Precision range of the kick direction. */
    float range = -1.f; /**< Kick range. Used to calculate the kick power. */
    TargetType type = TargetType::other; /**< Type of the situation, in which the kick is used. */
    Vector2f lastFieldEndPoint; /**< Ball end position after the kick. */
  };

  /**
   * Local copy of theSkillRequest
   */
  struct SkillRequestPose
  {
    Pose2f target;
    Rangea passRangeOne;
    Rangea passRangeTwo;
    int passTarget = -1;
    int ignoredPassTarget = -1;
    SkillRequest::Type skill = SkillRequest::none;
  };

  /**
   * Mapping of every kick range(that should be checked) to a rating.
   */
  struct RatingMap
  {
    std::optional<float> rating; /**< Rating of the range. */
    float range = 0.f; /**< The kick range. */
    Vector2f fieldEndPoint; /**< Global field coordinate. */
    bool isTeammatePass = false; /**< Is the field coordinate considered as a pass? */
    bool isTypeSteal = false; /**< Is the field coordinate considered as a steal of the ball? */

    float ratingFunc(const FieldRating& theFieldRating, const int passTarget, const Vector2f& useBallPosition,
                     const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal);
  };

  /**
   * Calculate the rating of a given kick
   *
   * @param theFieldRating the rating functions
   * @param passTarget The pass teammate from the behavior. -1 if none is planned.
   * @param useBallPosition The ball in field coordinates
   * @param isTypeSteal Is the kick a forwardSteal kick? In that case, ignore some ratings
   * @param direction Direction and length of the kick
   * @param rangeForGoal The range to reach the goal
   */
  float RatingMap::ratingFunc(const FieldRating& theFieldRating, const int passTarget, const Vector2f& useBallPosition,
                              const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal)
  {
    if(this->rating.has_value())
      return this->rating.value();

    this->fieldEndPoint = useBallPosition + direction * std::min(range, rangeForGoal);
    // normal rating
    bool isTeammatePass = false;
    PotentialValue teammate;
    PotentialValue pvBallNear;

    // get most ratings, with and without obstacle potential
    PotentialValue ratingField = theFieldRating.potentialFieldOnly(this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
    if(!isTypeSteal)
    {
      theFieldRating.getObstaclePotential(ratingField, this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
      theFieldRating.duelBallNearPotential(pvBallNear, this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
    }
    theFieldRating.potentialOverall(teammate, this->fieldEndPoint.x(), this->fieldEndPoint.y(), isTeammatePass, false, passTarget);
    theFieldRating.removeBallNearFromTeammatePotential(teammate, pvBallNear);
    ratingField += teammate;
    ratingField += pvBallNear; // add ball near potential after the teammate's potential

    this->rating = ratingField.value;
    this->isTeammatePass = isTeammatePass;

    return this->rating.value();
  }

  /**
   * The struct that contains the information of the ratings on the field.
   * It is used to calculate the ratings on demand, to reduce computation time.
   */
  struct RatingMapVector
  {
    std::vector<RatingMap> ratingMap;
    const bool isTypeSteal;
    const Vector2f direction;
    const float rangeForGoal;
    const int passTarget;
    const Vector2f useBallPosition;

    RatingMapVector(const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal, const int passTarget, const Vector2f& useBallPosition) :
      isTypeSteal(isTypeSteal), direction(direction), rangeForGoal(rangeForGoal), passTarget(passTarget), useBallPosition(useBallPosition) {}

    float ratingFunc(const FieldRating& theFieldRating, RatingMap& ratingMap);
  };

  /**
   * Interface to get the ratings
   */
  float RatingMapVector::ratingFunc(const FieldRating& theFieldRating, RatingMap& ratingMap)
  {
    return ratingMap.ratingFunc(
             theFieldRating, this->passTarget, this->useBallPosition, this->isTypeSteal, this->direction, this->rangeForGoal);
  }

  /**
   * Definition of a sector with an obstacle in it. Used for the sector wheel.
   */
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace Zweikampf;

option((SkillBehaviorControl) Zweikampf,
       load((SearchParameters) searchParameters, /**< Parameters to decide which kick angles are checked. */
            (DuelRatings) duelRatings, /**< All rating modifiers. */
            (DuelTimings) duelTimings, /**< All time stamp parameters. */
            (ObstacleHandling) obstacleHandling, /**< All parameters to handle obstacles. */
            (StealBallParameters) stealBallParameters, /**< All parameters to handle kick angles near obstacles. */
            (LastKickHysteresis) lastKickHysteresis, /**< Hysteresis parameters too make sure the last decision is kept. */
            (DistanceParameters) distanceParameters, /**< Parameters to generate list of ranges to check. */
            (StealKickReplacementParameter) stealKickReplacementParameter, /**< Parameters for replacing the steal kick. */

            (Angle) smallerKickAnglePreference, /**< Interpolate ratingLargeKickAngle to 100% over this angle range. */
            (Angle) forwardStealPreferenceRange, /**< If the kick angle for the forward steal is bigger or smaller than this value, we can be sure to use the better side. */
            (float) sectorWheelObstacleBallRadiusFactor, /**< Increase obstacle with by this factor times the ball radius.*/
            (Rangea) allowPassAngle, /**< Pass request from the skill request are allowed in this field angle range. */
            (Angle) passOrientation, /**< Pass request from the skill request are allowed if the robot is orientated to the target already good enough. */
            (float) fieldBorderSafeDistance, /**< InWalkKick have some uncertainty, so the safe distance to the field border intersection is increased by up to this value.*/

            (Angle) sidewardsRotationCorrectionBonus, /**< If last planned kick was the sidewards kick, then add a bonus to the allowed rotation. */

            (Rangef) ballPositionInterpolationRange, /**< If the ball is close, interpolate the perceived and intercepted position based on this ball distance ranges. */

            // look active
            (float) lookActiveMinBallDistance, /**< If the ball is at least this far away, use lookActive with withBall = true. */

            (bool) replaceForwardWithLongGoalShot, /**< If kicking at the goal, replace this forward kick with the long version. */

            (std::vector<KickInfo::KickType>) allowedKicks, /**< The allowed InWalkKicks */
            (Rangef) minKickRange), /**< Use min kick range based on field position. */
       vars((DuelPose) theDuelPose, /**< All information about the "to be executed" kick */
            (Angle)(0_deg) lastKickAngle, /**< Last kick angle with odometry update */
            (Angle)(0_deg) lastRobotPoseRotation, /**< Last RobotPose rotation */
            (SkillRequestPose) theSkillRequestPose, /**< All information about the current behavior request */
            (unsigned)(0) timeSinceDoingNothing, /**< timestamp since no kick was calculated */
            (unsigned)(0) lastForwardSteal, /**< timestamp since last forwardSteal was the best kick */
            (unsigned)(0) lastSidewardRestrictionTimestamp, /**< Last timestamp, when sidewards kick was restricted */
            (unsigned)(0) ignoreSkillRequestTimestamp, /**< Time stamp when skill request started to be ignored */
            (std::array<unsigned, TargetType::numOfTargetTypes>)({0, 0, 0, 0, 0}) kickForcedUpTime, /**< Timestamps of the execution of the different kick types. */
            (std::vector<float>) checkKickDistancesForFR, /**< All kick ranges, that shall be checked */
            (bool)(false) forcedInactive, /**< Deactivate the Zweikampf? */
            (bool)(false) forceForwardSteal, /**< Was the forwardSteal force because the obstacle is actually close to the ball? */
            (bool)(false) shouldDribble, /**< Zweikampf can not handle kicks right now. Instead dribble for a moment. */
            (unsigned)(0) lastDribbleRequest)) /**< Should we first dribble to get in goal range? */
{
  const Vector2f leftGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the opponent's goal.
  const Vector2f rightGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the opponent's goal.
  const Vector2f leftGoalPostOwn(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the own goal.
  const Vector2f rightGoalPostOwn(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the own goal.
  const Obstacle opponentLeftGoalPost(Matrix2f::Zero(), leftGoalPost, leftGoalPost + Vector2f(0.f, theFieldDimensions.goalPostRadius), leftGoalPost - Vector2f(0.f, theFieldDimensions.goalPostRadius), 0, Obstacle::Type::unknown);
  const Obstacle opponentRightGoalPost(Matrix2f::Zero(), rightGoalPost, rightGoalPost + Vector2f(0.f, theFieldDimensions.goalPostRadius), rightGoalPost - Vector2f(0.f, theFieldDimensions.goalPostRadius), 0, Obstacle::Type::unknown);

  const Vector2f penaltyBoxBackRight(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea); // Back right corner of own penalty box.
  const Vector2f penaltyBoxFrontLeft(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); // Front left corner of own penalty box.
  const Vector2f opponentHalfBackRight(0.f, theFieldDimensions.yPosRightTouchline); // Back right corner of opponent half.
  const Vector2f opponentHalfFrontLeft(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline); // Front left corner of opponent half.
  const Geometry::Rect opponentHalf(opponentHalfBackRight, opponentHalfFrontLeft); // Rectangle of opponent half.
  const Geometry::Rect opponentGoalArea(Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea - 100.f), Vector2f(theFieldDimensions.xPosOpponentGoalLine + 200.f, theFieldDimensions.yPosLeftGoalArea + 100.f)); // Rectangle of opponent goal area.
  const Geometry::Rect penaltyRect(penaltyBoxBackRight, penaltyBoxFrontLeft); // Rectangle of own penalty area.
  const Geometry::Rect playingField(-opponentHalfFrontLeft, opponentHalfFrontLeft);
  const Rangea boundaryCheck(-180_deg, 180_deg); // Min and max angles.
  const Rangef ballClipX(theFieldDimensions.xPosOwnGoalLine + 10.f, theFieldDimensions.xPosOpponentGoalLine - 10.f);
  const Rangef ballClipY(theFieldDimensions.yPosRightTouchline + 10.f, theFieldDimensions.yPosLeftTouchline - 10.f);
  const float baseHighestDistance = sqr(theFieldDimensions.xPosOpponentGoal * 2.f); // Just a large number as an upper bound

  // Variables written and read by different lambdas
  bool interpolateBallVelocity = true;
  Obstacle duelObstacle; // Obstacle, with which we are fighting for the ball
  Obstacle duelObstacleMovedNearBall; // duelObstacle, but shifted based on the ball position
  float distanceToClosestObstacle; // distance to the duelObstacle
  std::list<SectorWheel::Sector> kickAngles; // SectorWheel
  std::vector<Rangea> goalAngleWithBuffer; // Goal sectors with more buffer to the goal posts and other obstacles
  Vector2f useBallPositionInField = Vector2f(0.f, 0.f); // Clipped Ballposition in field coordinates
  Vector2f useBallPositionRelative = Vector2f(0.f, 0.f); // Ballposition in relative coordinates
  bool soonCloseRangeDuel = false; // Opponent is dangerous near to the ball. Start using risky kicks

  /**
   * Update the kick angle based on the robot pose change compared to the previous execution frame
   */
  const auto updateDuelPose = [&]
  {
    // Calculate kickAngle based in the planned end point
    lastKickAngle = Angle::normalize(theDuelPose.kickAngle - theRobotPose.rotation + lastRobotPoseRotation);
    lastRobotPoseRotation = theRobotPose.rotation;
    theDuelPose.kickAngle = (theDuelPose.lastFieldEndPoint - useBallPositionInField).angle() - theRobotPose.rotation;
    if(std::isnan(theDuelPose.kickAngle))
      theDuelPose.kickAngle = 0_deg;
  };

  /**
   * The behavior plans a pass to a teammate. Set up some variables
   */
  const auto updatePassEndPosition = [&]
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = nullptr;
    for(const auto& t : theGlobalTeammatesModel.teammates)
    {
      if(t.playerNumber == theSkillRequestPose.passTarget)
        teammate = &t;
    }
    if(!teammate)
      theSkillRequestPose.passTarget = -1;
    else
    {
      theSkillRequestPose.target = teammate->pose;

      // In case the kick angles are restricted, e.g. for back passes, pre compute kick angles ranges which are allowed for only passes
      const Angle relativeAngle = Angle::normalize((theSkillRequestPose.target.translation - useBallPositionInField).angle() - theRobotPose.rotation);
      const Rangea range(Angle::normalize(relativeAngle - passOrientation), Angle::normalize(relativeAngle + passOrientation));
      if(range.min > range.max)
      {
        theSkillRequestPose.passRangeOne = Rangea(range.min, 180_deg);
        theSkillRequestPose.passRangeTwo = Rangea(-180_deg, range.max);
      }
      else
      {
        theSkillRequestPose.passRangeOne = range;
        theSkillRequestPose.passRangeTwo = range;
      }
    }
  };

  // TODO Refactor Method, updatePassEndPosition() is kinda useless and code is hard to read, because it is unnecessary complicated
  // If skill request changes and it is a pass, we just check if the target exists to verify a possible target
  // If dribbling, take the target direction as a base for the search later in the Zweikampf
  // In general: Just update the last direction based on the odometry
  const auto updateSkillRequest = [&]
  {
    if((theFrameInfo.getTimeSince(ignoreSkillRequestTimestamp) > duelTimings.ignoreSkillRequestTime || theSkillRequest.skill != SkillRequest::pass || // Ignore bad to be executed passes
        (theSkillRequestPose.ignoredPassTarget != -1 && theSkillRequestPose.ignoredPassTarget != theSkillRequest.passTarget)) && // pass target changed
       (theSkillRequest.skill != theSkillRequestPose.skill ||
        theSkillRequest.passTarget != theSkillRequestPose.passTarget))
    {
      theSkillRequestPose.passTarget = theSkillRequest.skill == SkillRequest::pass ? theSkillRequest.passTarget : -1;
      theSkillRequestPose.skill = theSkillRequest.skill;
      theSkillRequestPose.target = theSkillRequest.target;
      switch(theSkillRequest.skill)
      {
        case SkillRequest::pass:
        {
          for(const auto& t : theGlobalTeammatesModel.teammates)
          {
            if(t.playerNumber == theSkillRequestPose.passTarget)
            {
              updatePassEndPosition();
              goto teammateFound;
            }
          }
          theSkillRequestPose.passTarget = -1; // Nothing more needed here. Just use the last duelPose as a base
        teammateFound:
          break;
        }
        case SkillRequest::dribble:
          theDuelPose.kickAngle = theSkillRequestPose.target.rotation - theRobotPose.rotation;

        default:
          break;
      }
    }
    else if(theSkillRequestPose.skill == SkillRequest::pass)
      updatePassEndPosition();

    if(theSkillRequestPose.passTarget == -1 && theSkillRequestPose.skill == SkillRequest::pass)
      theSkillRequestPose.skill = SkillRequest::shoot;

    updateDuelPose();
  };

  /**
   * Obstacles that are close to the ball shall be changed and moved slightly away.
   * Otherwise an obstacle can be in front of the ball by a few mm. This is an error in the perception which is corrected here.
   */
  const auto shiftObstacleBackward = [&](Vector2f& obstacleCenter, Vector2f& duelObstacleMovedNearBall)
  {
    // Currently, obstacles that are really close to the ball, but not in front of it, are perceived as being in front of the ball by a few cm.
    // We need to shift them backwards behind the ball, otherwise the sector wheel for the duel calculation will just ignore this obstacle.
    // This would just result in a forward kick -> ball bounces off the opponent and we just walk on top of the opponent feet and fall over.
    const float obstacleDistanceToBall = (obstacleCenter - useBallPositionRelative).norm();
    if(theFieldBall.timeSinceBallWasSeen < 1500.f && obstacleDistanceToBall <= std::abs(obstacleHandling.obstacleShiftRange.min) && obstacleCenter.x() < useBallPositionRelative.x() + obstacleHandling.obstacleShiftRange.max && obstacleCenter.x() > theFieldBall.positionRelative.x() + obstacleHandling.obstacleShiftRange.min)
    {
      const Angle rotationAngle = (Vector2f(theBallSpecification.radius, obstacleCenter.y() - useBallPositionRelative.y())).angle();
      obstacleCenter = Vector2f(std::max(std::abs(obstacleCenter.y() - useBallPositionRelative.y()), theBallSpecification.radius), 0.f).rotated(rotationAngle) + useBallPositionRelative;
    }
    if(duelObstacleMovedNearBall != Vector2f(1.f, 0.f) && (theRobotPose * obstacleCenter - useBallPositionInField).x() > 0.f)
      duelObstacleMovedNearBall = (duelObstacleMovedNearBall - useBallPositionRelative).normalized(obstacleHandling.obstacleDistanceForBlockingPose) + useBallPositionRelative;
  };

  /**
   * Find the obstacle which is our dueling obstacle. Important for some decisions for the forwardSteal kick.
   */
  const auto calculateClosestObstacle = [&](const ObstacleModel& model, Obstacle& nearestObstacle, Obstacle& duelObstacleMovedNearBall)
  {
    distanceToClosestObstacle = baseHighestDistance;
    duelObstacle.center = Vector2f(baseHighestDistance, 0.f);

    for(const Obstacle& omo : model.obstacles)
    {
      const float sqrNorm = (omo.center - theFieldBall.positionRelative).squaredNorm();
      if(sqrNorm < distanceToClosestObstacle)
      {
        distanceToClosestObstacle = sqrNorm;
        nearestObstacle = omo;
      }
    }
    distanceToClosestObstacle = std::sqrt(distanceToClosestObstacle);
    duelObstacleMovedNearBall = nearestObstacle;
    shiftObstacleBackward(nearestObstacle.center, duelObstacleMovedNearBall.center);
  };

  /**
   * Decide whether we should ignore the request of the behavior
   */
  const auto handleIgnoreSkillRequest = [&](const Vector2f& opponentOnField)
  {
    if(theSkillRequestPose.skill == SkillRequest::pass)
    {
      const Angle fieldAngle = (theSkillRequestPose.target.translation - useBallPositionInField).angle();
      if(!allowPassAngle.isInside(fieldAngle) &&
         (opponentOnField - useBallPositionInField).squaredNorm() < sqr(obstacleHandling.obstacleMinDistanceForBackPass) &&
         std::abs(fieldAngle - theRobotPose.rotation) > passOrientation) // the robot must be aligned good enough for the pass already
      {
        ignoreSkillRequestTimestamp = theFrameInfo.time;
        theSkillRequestPose.skill = SkillRequest::shoot;
        theSkillRequestPose.ignoredPassTarget = theSkillRequestPose.passTarget;
        theSkillRequestPose.passTarget = -1;
      }
    }
  };

  /**
   * Calculate the obstacle and goal sectors
   */
  const auto calculateSectorWheel = [&]
  {
    Vector2f dummyOB(1.f, 0.f);

    const auto calcObstacleInSector = [&](const Obstacle& obstacle, ObstacleSector& dummyObstacleSector)
    {
      Vector2f useObstacleCenter = obstacle.center;
      bool filterSector = false;
      shiftObstacleBackward(useObstacleCenter, dummyOB);
      filterSector = useObstacleCenter != obstacle.center;

      if(!filterSector && useObstacleCenter == duelObstacle.center)
      {
        Vector2f disToBall = useObstacleCenter - useBallPositionRelative;
        const float normSquared = disToBall.squaredNorm();
        const float ownDistanceToBallSquared = useBallPositionRelative.squaredNorm();
        if(normSquared > sqr(300.f))
          useObstacleCenter = useBallPositionRelative + disToBall / (std::sqrt(normSquared)) * std::max(300.f, std::sqrt(normSquared - ownDistanceToBallSquared));
      }

      // clip obstacle behind the ball
      const Vector2f obstacleOnField = theRobotPose * useObstacleCenter;
      if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
        return false;

      const float widthScale = 1.f - Rangef::ZeroOneRange().limit((theFrameInfo.getTimeSince(obstacle.lastSeen) - 1000) / 4000.f);
      const float width = widthScale * ((obstacle.left - obstacle.right).norm() + sectorWheelObstacleBallRadiusFactor * theBallSpecification.radius);
      const float distance = (obstacleOnField - useBallPositionInField).norm();

      // TODO make it less ugly
      // In case the obstacle is "inside" the ball, the sector needs to be filtered. otherwise the possible kick angles change heavily every frame...
      const float radius = std::atan(width / (2.f * distance));
      const Angle direction = (obstacleOnField - useBallPositionInField).angle();
      Rangea obstacleSectorRange = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
      if(filterSector &&
         ((std::abs(obstacleSectorRange.min) > 90_deg && std::abs(obstacleSectorRange.max) < 90_deg) ||
          (std::abs(obstacleSectorRange.max) > 90_deg && std::abs(obstacleSectorRange.min) < 90_deg)))
      {
        const Rangea clippedRange(-90_deg, 90_deg);
        obstacleSectorRange.min = clippedRange.limit(obstacleSectorRange.min) ;
        obstacleSectorRange.max = clippedRange.limit(obstacleSectorRange.max);
      }
      obstacleSectorRange.min -= theRobotPose.rotation;
      obstacleSectorRange.max -= theRobotPose.rotation;
      obstacleSectorRange.max = Angle::normalize(obstacleSectorRange.max);
      obstacleSectorRange.min = Angle::normalize(obstacleSectorRange.min);
      dummyObstacleSector.distance = distance;
      dummyObstacleSector.sector = obstacleSectorRange;
      dummyObstacleSector.x = obstacleOnField.x();
      return true;
    };

    std::vector<ObstacleSector> obstacleSectors;
    ObstacleSector dummyObstacleSector;
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
      if(calcObstacleInSector(obstacle, dummyObstacleSector))
        obstacleSectors.push_back(dummyObstacleSector);
    }

    Obstacle opponentLeftGoalPostInRobot = opponentLeftGoalPost;
    opponentLeftGoalPostInRobot.center = theRobotPose.inverse() * opponentLeftGoalPostInRobot.center;
    Obstacle opponentRightGoalPostInRobot = opponentRightGoalPost;
    opponentRightGoalPostInRobot.center = theRobotPose.inverse() * opponentRightGoalPostInRobot.center;

    if(calcObstacleInSector(opponentLeftGoalPostInRobot, dummyObstacleSector))
      obstacleSectors.push_back(dummyObstacleSector);
    if(calcObstacleInSector(opponentRightGoalPostInRobot, dummyObstacleSector))
      obstacleSectors.push_back(dummyObstacleSector);

    SectorWheel drawWheel;
    SectorWheel wheel;
    std::list<SectorWheel::Sector> sectors;
    std::list<SectorWheel::Sector> kickAnglesPre;

    wheel.begin(useBallPositionInField);
    const float minBallGoalPostOffsetSquared = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
    const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (leftGoalPost - useBallPositionInField).norm()));
    const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (rightGoalPost - useBallPositionInField).norm()));
    const Angle angleToLeftPost = Angle::normalize((leftGoalPost - useBallPositionInField).angle() - leftAngleOffset - theRobotPose.rotation - 2_deg);
    const Angle angleToRightPost = Angle::normalize((rightGoalPost - useBallPositionInField).angle() + rightAngleOffset - theRobotPose.rotation + 2_deg);

    if(angleToLeftPost > angleToRightPost)
      wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

    for(const ObstacleSector& obstacleSector : obstacleSectors)
      wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
    kickAnglesPre = wheel.finish();

    const auto shiftObstacleRanges = [](auto& itOuter, auto& closestSector)
    {
      const float distanceDiff = std::abs(itOuter->distance - closestSector->distance);
      const float ratio = mapToRange(distanceDiff, 200.f, 500.f, 1.f, 0.f);
      const Angle shift = ratio * (itOuter->angleRange.getSize());
      if(itOuter->angleRange.min < closestSector->angleRange.min)
      {
        closestSector->angleRange.min = Angle::normalize(itOuter->angleRange.max - shift);
        // floating point errors -> need to clip to max
        itOuter->angleRange.max = std::max(itOuter->angleRange.min, Angle(Angle::normalize(itOuter->angleRange.max - shift)));
      }
      else
      {
        closestSector->angleRange.max = Angle::normalize(itOuter->angleRange.min + shift);
        // floating point errors -> need to clip to min
        itOuter->angleRange.min = std::min(itOuter->angleRange.min, Angle(Angle::normalize(itOuter->angleRange.min + shift)));
      }
    };

    /**
     * Obstacles close to each other shall become smaller if further away, and obstacles closer shall include the sizes of their neighbors
     */
    const auto searchObstacleWall = [](const std::vector<float>& distances,
                                       const auto& itCurrent,
                                       const auto& itNext,
                                       const auto& itNextAfter,
                                       const auto& end) -> bool
    {
      if(itNext == end || itNextAfter == end || itNextAfter->type != SectorWheel::Sector::obstacle || itNext->type != SectorWheel::Sector::obstacle)
        return false;

      float ratio = mapToRange(std::abs(itNextAfter->distance - itCurrent->distance), 200.f, 500.f, 1.f, 0.f);
      for(const float distance : distances)
        ratio = std::max(ratio, mapToRange(std::abs(itNextAfter->distance - distance), 200.f, 500.f, 1.f, 0.f));
      return ratio > 0.f;
    };

    // Combine obstacle sectors, that are too close together
    for(auto it = kickAnglesPre.begin(); it != kickAnglesPre.end(); it++)
    {
      if(it->type == SectorWheel::Sector::obstacle)
      {
        std::list<SectorWheel::Sector>::iterator itNext;
        std::list<SectorWheel::Sector>::iterator itNextAfter = std::next(it, 1);
        if(itNextAfter == kickAnglesPre.end() || itNextAfter->type != SectorWheel::Sector::obstacle)
          continue;
        std::list<SectorWheel::Sector>::iterator itLast = itNextAfter;
        std::list<SectorWheel::Sector>::iterator itSmallestDistance = it->distance < itNextAfter->distance ? it : itNextAfter;
        std::vector<float> distances = { itNextAfter->distance };
        bool continueSearch = true;
        while(continueSearch)
        {
          itNext = itNextAfter;
          itNextAfter = std::next(itNext, 1);
          continueSearch = searchObstacleWall(distances, it, itNext, itNextAfter, kickAnglesPre.end());
          if(continueSearch)
          {
            distances.push_back(itNextAfter->distance);
            itSmallestDistance = itSmallestDistance->distance < itNextAfter->distance ? itSmallestDistance : itNextAfter;
            itLast = itNextAfter;
          }
        }
        if(itLast != kickAnglesPre.end() && itLast->angleRange.min < itLast->angleRange.max)
        {
          for(auto itCurrent = std::next(it, 1); itCurrent != itLast; itCurrent++)
          {
            ASSERT(itCurrent != kickAnglesPre.end());
            if(itCurrent != itSmallestDistance)
              itCurrent->angleRange.min = itCurrent->angleRange.max;
          }
        }

        if(it != itSmallestDistance)
          shiftObstacleRanges(it, itSmallestDistance);
        if(itLast != itSmallestDistance)
          shiftObstacleRanges(itLast, itSmallestDistance);

        // skip all sectors in between, as those now have a width of 0
        ASSERT(itLast != kickAnglesPre.begin());
        if(std::prev(itLast, 1) != it)
          it = std::prev(itLast, 1);
      }
    }

    wheel = SectorWheel();
    wheel.begin(useBallPositionInField);
    if(angleToLeftPost > angleToRightPost)
      wheel.addSector(Rangea(angleToRightPost, angleToLeftPost), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);

    for(const auto& sector : kickAnglesPre)
    {
      if(sector.type == SectorWheel::Sector::obstacle && sector.angleRange.min != sector.angleRange.max)
        wheel.addSector(sector.angleRange, sector.distance, SectorWheel::Sector::obstacle);
    }
    kickAngles = wheel.finish();

    for(const auto& sector : kickAngles)
    {
      if(sector.type == SectorWheel::Sector::goal)
      {
        const Angle min = Angle::normalize(sector.angleRange.min + searchParameters.goalShotBufferAngle);
        const Angle max = Angle::normalize(sector.angleRange.max - searchParameters.goalShotBufferAngle);
        goalAngleWithBuffer.emplace_back(min, max);
      }
    }

    COMPLEX_DRAWING("option:Zweikampf:wheel")
    {
      drawWheel.begin(useBallPositionInField);
      for(const SectorWheel::Sector& sector : kickAngles)
        drawWheel.addSector(Rangea(Angle::normalize(sector.angleRange.min + theRobotPose.rotation), Angle::normalize(sector.angleRange.max + theRobotPose.rotation)), sector.distance, sector.type);
      sectors = drawWheel.finish();
      DRAW_SECTOR_WHEEL("option:Zweikampf:wheel", sectors, useBallPositionInField);
    }
  };

  /**
   * Calculate the rating of a given kick, based on theFieldRating and some other constraints
   * Also filter some kicks out before calculating the ratings, to reduce computation time
   */
  const auto getDuelRating = [&](const KickInfo::KickType kickType, const Angle kickAngle, DuelPose& duelPose,
                                 const Angle minMaxAngle, const bool isGoalAngle, const float maxKickRange, const float distanceToOutOfFieldSquared,
                                 const Rangea& stealBallBonusMin, const Rangea& stealBallBonusMax, const Angle angleFromBallToOpponent,
                                 RatingMapVector& ratingMapVector, const float rangeForGoal,
                                 const Angle angleFromGoalToBall, const TargetType highestTargetType,
                                 const float dribbleRange) -> bool
  {
    Angle rotationOffset(theKickInfo[kickType].rotationOffset);
    Vector2f ballOffset(theKickInfo[kickType].ballOffset);
    Rangef useKickRange = theKickInfo[kickType].range;

    DuelPose pose;
    // 8.4.6. Calculate kick pose
    pose.noKick = false;
    pose.kickType = kickType;
    pose.kickAngle = kickAngle;
    pose.pose = (Pose2f(kickAngle + theRobotPose.rotation, useBallPositionInField) - theRobotPose)
    .rotate(rotationOffset)
    .translate(ballOffset);
    pose.pose.rotation.normalize();

    // 8.4.4. kick range is too long and would hit an opponent
    if(useKickRange.min > maxKickRange)
      return false;

    const float rangeHysteresis = !isGoalAngle || theDuelPose.kickType != kickType || theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight ? 0.f : lastKickHysteresis.kickLengthHysteresis * (1.f - Rangef::ZeroOneRange().limit(std::abs(Angle::normalize(theDuelPose.kickAngle - kickAngle)) / lastKickHysteresis.kickDirectionHysteresis));

    // 8.4.7. The robot needs to turn too much
    // Add range minmax, to prevent robot doing stupid kick angles when chasing opponent
    // needs to be in relative rotation, but based on world rotation
    // steal ball -> 90_deg to -90_deg -> 90_deg to 180_deg and -180_deg to -90_deg
    // not steal ball -> calculate based on minMaxAngle and create both ranges that way
    // TODO precompute to save computation time
    Rangea maxAllowedRotationRangeOne(-minMaxAngle, minMaxAngle);
    Rangea maxAllowedRotationRangeTwo = maxAllowedRotationRangeOne;
    const bool isStealSituation = stealBallBonusMin != stealBallBonusMax;
    bool checkForKickAngleInstead = false;
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
    {
      Angle maxAllowedRotation = std::max(minMaxAngle, stealBallParameters.stealBallTypeTooMuchRotationMinValue);
      maxAllowedRotation += sidewardsRotationCorrectionBonus;
      maxAllowedRotationRangeOne = Rangea(-maxAllowedRotation, maxAllowedRotation);
      maxAllowedRotationRangeTwo = maxAllowedRotationRangeOne;
    }
    else if(isStealSituation)
    {
      checkForKickAngleInstead = true;
      const Angle max = Angle::normalize((theRobotPose.rotation < 0 ? -stealBallParameters.stealBallRange.min : 110_deg) - theRobotPose.rotation);
      const Angle min = Angle::normalize((theRobotPose.rotation > 0 ? stealBallParameters.stealBallRange.min : -110_deg) - theRobotPose.rotation);
      if(max < min)
      {
        maxAllowedRotationRangeOne = Rangea(std::max(-180_deg, min), std::min(180_deg, max));
        maxAllowedRotationRangeTwo = Rangea(std::min(180_deg, min), std::min(180_deg, max));
      }
      else
      {
        maxAllowedRotationRangeOne = Rangea(min, max);
        maxAllowedRotationRangeTwo = maxAllowedRotationRangeOne;
      }
    }
    const Angle useRotationOfInterest = checkForKickAngleInstead ? kickAngle : pose.pose.rotation;
    bool tooMuchRotationForNonePass = false;
    if(!maxAllowedRotationRangeOne.isInside(useRotationOfInterest) && !maxAllowedRotationRangeTwo.isInside(useRotationOfInterest) &&  // too much rotation needed
       !(isGoalAngle && rangeForGoal < dribbleRange + useKickRange.max + ((theDuelPose.type == TargetType::goalShot || theDuelPose.type == TargetType::goalDribbleShot) ? rangeHysteresis : 0.f)) &&// we risk losing the ball to score a goal
       !(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)) // forwardSteal can always be done
    {
      // If the kick angle will be a pass, then it is allowed
      if(theSkillRequestPose.passTarget != -1 &&
         (theSkillRequestPose.passRangeOne.isInside(kickAngle) || theSkillRequestPose.passRangeTwo.isInside(kickAngle)))
        tooMuchRotationForNonePass = true;
      else
        return false;
    }

    // 8.4.8. for the forward steal, we are standing on the wrong side
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
    {
      Rangea positionRange(angleFromGoalToBall - 90_deg, angleFromGoalToBall + 90_deg);
      if(!positionRange.isInside((useBallPositionInField - (theRobotPose + pose.pose.translation).translation).angle())) // calculate in field coordinates. this is wrong
        return false;
    }

    // 8.4.3. Get PotentialField rating, so we know the kick range
    size_t bestIndex = static_cast<size_t>(-1);
    float bestRating = std::numeric_limits<float>::max();
    const bool useStrongForwardKick = Geometry::isPointInsideRectangle(opponentGoalArea, useBallPositionInField) ||
                                      isStealSituation;
    const float useFieldBorderSafeDistance = isStealSituation ? 0.f : fieldBorderSafeDistance;
    const float useMinKickRange = kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight ? 1.f : mapToRange(useBallPositionInField.x(), theFieldDimensions.centerCircleRadius, theFieldDimensions.xPosOpponentPenaltyArea, std::min(minKickRange.max, theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyArea * 0.5f), minKickRange.min);

    for(size_t index = 0; index < ratingMapVector.ratingMap.size(); index++)
    {
      if(sqr(ratingMapVector.ratingMap[index].range + useFieldBorderSafeDistance) > distanceToOutOfFieldSquared ||
         ratingMapVector.ratingMap[index].range > useKickRange.max + rangeHysteresis) // kick range is too long
        break;
      if(ratingMapVector.ratingMap[index].range < useMinKickRange)  // skip to weak kicks
        continue;
      if(
        ratingMapVector.ratingMap[index].range >= useKickRange.min - rangeHysteresis && // kick range executable checks
        ratingMapVector.ratingFunc(theFieldRating, ratingMapVector.ratingMap[index]) < bestRating + (useStrongForwardKick ? 0.1f : 0.f)) // a "<=" would result in using always a longer kick than a shorter one. TODO eval what is better
      {
        bestIndex = index;
        bestRating = (ratingMapVector.ratingFunc(theFieldRating, ratingMapVector.ratingMap[index]) + (pose.type == TargetType::stealBall || (isGoalAngle && useKickRange.max + (theDuelPose.type == TargetType::goalShot ? rangeHysteresis : 0.f) >= rangeForGoal) ? 0.01f : 0.f)); // + 0.01f makes sure, the strongest kick is used for stealing the ball
      }
    }
    if(bestIndex == static_cast<size_t>(-1))
      return false;
    if(tooMuchRotationForNonePass && !ratingMapVector.ratingMap[bestIndex].isTeammatePass) // the kick angle is not allowed for none passes
      return false;
    pose.range = ratingMapVector.ratingMap[bestIndex].range;
    pose.rating += bestRating;
    pose.type = ratingMapVector.ratingMap[bestIndex].isTeammatePass ? TargetType::pass : pose.type;
    pose.lastFieldEndPoint = ratingMapVector.ratingMap[bestIndex].fieldEndPoint;

    // 8.4.9. get the end point and check, if we can score a goal

    Vector2f intersectionPoint(0.f, 0.f);
    if(isGoalAngle && // checks, if kick distance is long enough to reach the goal
       rangeForGoal < dribbleRange + useKickRange.max + (theDuelPose.type == TargetType::goalShot ? rangeHysteresis : 0.f))
    {
      pose.rating += duelRatings.ratingGoalShot;
      // kick angles with some buffer zone to goal posts and obstacles are better
      Angle smallestAngleDiff = std::numeric_limits<float>::max();
      for(const Rangea& goalSector : goalAngleWithBuffer)
        smallestAngleDiff = std::min(smallestAngleDiff, Angle(std::abs(pose.kickAngle - goalSector.limit(pose.kickAngle))));
      pose.rating += duelRatings.ratingGoalShotBuffer * mapToRange(static_cast<float>(smallestAngleDiff), 0.f, static_cast<float>(searchParameters.goalShotBufferAngle), 1.f, 0.f);
      pose.range = useKickRange.max;
      pose.type = rangeForGoal > useKickRange.max + (theDuelPose.type == TargetType::goalShot ? rangeHysteresis : 0.f) ? TargetType::goalDribbleShot : TargetType::goalShot;
    }

    // rlKickStrong is only used for goal shots
    if(pose.type != TargetType::goalShot && kickType == KickInfo::rlKickStrong)
      return false;

    if((highestTargetType == TargetType::goalShot || highestTargetType == TargetType::goalDribbleShot) && pose.type > highestTargetType)
      return false;

    // 8.4.10. Own penalty area checks
    // 8.4.10.1. we would kick the ball into our own penalty area, which we do not want
    if(!Geometry::isPointInsideRectangle(penaltyRect, useBallPositionInField) && Geometry::isPointInsideRectangle(penaltyRect, pose.lastFieldEndPoint))
      return false;

    // 8.4.10.3. we do no want to accidentally kick the ball into a good position for the opponent
    if(kickType != KickInfo::walkForwardStealBallLeft && kickType != KickInfo::walkForwardStealBallRight && pose.lastFieldEndPoint.x() < theFieldDimensions.xPosOwnPenaltyMark && useBallPositionInField.x() > pose.lastFieldEndPoint.x())
      return false;

    if(!isGoalAngle && maxKickRange != std::numeric_limits<float>::max())
      pose.rating += mapToRange(pose.range, maxKickRange - obstacleHandling.obstacleDistanceThreshold, maxKickRange, 0.f, duelRatings.ratingPassingToOpponent);

    // 8.4.7.2 worse rating for too much needed rotation
    // This is checked here, because after 8.4.7 some values needed to be updated before this calculation
    pose.rating += std::max(0.f, std::abs(pose.pose.rotation) - minMaxAngle) / minMaxAngle * duelRatings.ratingMinMaxDifference * (pose.type == TargetType::pass || pose.type == TargetType::stealBall || pose.type == TargetType::goalShot ? 0.3f : 1.f); // too much rotation needed, but worth it for a goal shot

    // 8.4.11. ball will land in own half as a back kick
    if(pose.lastFieldEndPoint.x() < useBallPositionInField.x() && pose.lastFieldEndPoint.x() < 0.f)
    {
      pose.rating += duelRatings.ratingBallLandsInOwnHalf;
      pose.range = pose.type == TargetType::other ? useKickRange.min : pose.range; // the rating will be off, but the ball is only kicked for a small distance
    }

    // 8.4.12 for the forward steal, make sure the better side is chosen, also the ball shall not come too close to the touchline
    if(kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight)
    {
      pose.rating += duelRatings.ratingStealBallKick;
      pose.type = TargetType::stealBall;
      float outOfFieldMalus = std::min(0.f, std::abs(std::abs(pose.lastFieldEndPoint.y()) - (theFieldDimensions.yPosLeftTouchline - 500.f)) / 500.f);
      if(outOfFieldMalus == 0) // ball does not come close, add a bonus for the kick, that has a high success chance
      {
        if((angleFromBallToOpponent > forwardStealPreferenceRange && kickType == KickInfo::walkForwardStealBallLeft) ||
           (angleFromBallToOpponent < -forwardStealPreferenceRange && kickType == KickInfo::walkForwardStealBallRight))
          pose.rating += duelRatings.ratingStealBallKickBetterSide;
      }
      else
        pose.rating -= duelRatings.ratingStealBallKickBetterSide * outOfFieldMalus;
    }

    if(pose.type > highestTargetType)
      return false;

    /////////////////////////////
    // 8.4.13. Fall Prevention //
    // //////////////////////////
    // an obstacle would prevent that the robot can reach this kick pose. To prevent walking on the opponents feet or not reaching the kick pose in time,
    // the kicks rating gets worsens in case an obstacle is only on one side
    switch(kickType)
    {
      // Check first to prevent fall through below
      case KickInfo::walkForwardStealBallLeft:
      case KickInfo::walkForwardStealBallRight:
      case KickInfo::rlKickDynamic:
      case KickInfo::rlKickDynamicOld:
      case KickInfo::rlKickStrong:
        break;
      default:
        FAIL("Unknown rl kick");
    }

    // 8.4.15. Bonus for same kick, but smaller rotation
    if(!theDuelPose.noKick &&
       ((theDuelPose.kickType == kickType) && theDuelPose.type == pose.type))
    {
      pose.rating += duelRatings.ratingSameKick;
      pose.rating += duelRatings.ratingSameKickAngle * (1.f - std::min((std::abs(Angle::normalize(theDuelPose.kickAngle - kickAngle))) / smallerKickAnglePreference, 1.f));
    }

    duelPose = pose;

    return true;
  };

  /**
   * Draw the ratings of the best types
   */
  const auto drawRating = [&](std::vector<DuelPose>& duelPoses)
  {
    Rangef minMaxDrawRatings(-1.f, 1.f); // Reset
    for(DuelPose& pose : duelPoses)
    {
      minMaxDrawRatings.min = std::min(pose.rating, minMaxDrawRatings.min);
      minMaxDrawRatings.max = std::max(pose.rating, minMaxDrawRatings.max);
    }
    std::vector<float> ranges;
    DuelPose* lastUniqueKick = nullptr;
    const float minShift = (minMaxDrawRatings.min + minMaxDrawRatings.max) / 2.f;
    const float ratingRange = (minMaxDrawRatings.max - minMaxDrawRatings.min) / 2.f;

    for(DuelPose& pose : duelPoses)
    {
      // do not over draw better ratings
      if(!lastUniqueKick || lastUniqueKick->kickAngle != pose.kickAngle)
      {
        lastUniqueKick = &pose;
        ranges.clear();
      }
      if(lastUniqueKick->rating >= pose.rating || !(std::find(ranges.begin(), ranges.end(), pose.lastFieldEndPoint.squaredNorm()) != ranges.end()))
      {
        ranges.emplace_back(pose.lastFieldEndPoint.squaredNorm());
        lastUniqueKick = &pose;

        ColorRGBA color(0, 0, 0, 0);

        const float useRating = (pose.rating - minShift) / ratingRange;
        if(useRating >= 0.f)
        {
          color.r = static_cast<unsigned char>(useRating * 255);
          color.b = static_cast<unsigned char>((1.f - useRating) * 255);
          color.g = 0;
          color.a = 100;
        }
        else
        {
          color.r = 0;
          color.b = static_cast<unsigned char>(useRating * -1.f * 255);
          color.g = static_cast<unsigned char>((1.f - useRating * -1.f) * 255);
          color.a = 100;
        }

        FILLED_RECTANGLE("option:Zweikampf:kicks",
                         pose.lastFieldEndPoint.x() - 30.f, pose.lastFieldEndPoint.y() - 30.f,
                         pose.lastFieldEndPoint.x() + 30.f, pose.lastFieldEndPoint.y() + 30.f,
                         1, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, color);
      }
    }
  };

  /**
   * Main function that calculates
   * - all kick angles that shall be checked
   * - get sector wheel
   * - init some variables
   * - get rating of all kicks in the different kick angles
   * - get best kick
   * - calculate precision range
   * - if no kick is possible, set up dribble
   */
  const auto calculateDuel = [&]
  {
    // Hack for replaying logs
    if(timeSinceDoingNothing > theFrameInfo.time)
      timeSinceDoingNothing = 0;
    // First frame for Zweikampf. Ensure doing nothing is handled correctly
    if(timeSinceDoingNothing == 0)
      timeSinceDoingNothing = theFrameInfo.time - duelTimings.noKickMinTime - 1;
    // 1. update some variables
    if(theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.inactiveAfterDoingNothing)
      timeSinceDoingNothing = theFrameInfo.time;

    DuelPose lastDuelPose = theDuelPose;
    std::vector<Angle> directionPossibilities;
    Angle minMaxAngle;
    std::vector<std::vector<DuelPose>> duelPoses;
    for(std::size_t i = 0; i < TargetType::numOfTargetTypes; i++)
      duelPoses.emplace_back();

    const Vector2f opponentOnField = theRobotPose * duelObstacle.center;
    soonCloseRangeDuel = (duelObstacle.center - useBallPositionRelative).squaredNorm() < sqr(obstacleHandling.maxObstacleDistanceToBallForRiskyKicks);

    handleIgnoreSkillRequest(opponentOnField);

    // 2. Calculate distance metric between us and the opponent to the ball
    // fast simulation scenes need in theory a 50 to 100 mm shift
    const float opponentAndSelfDistanceToBallDiff = (opponentOnField - useBallPositionInField).norm() - std::max(0.f, useBallPositionRelative.norm() - (theRobotDimensions.footLength + theBallSpecification.radius)) // distance to ball for duelObstacle and self
    - ((ModuleGraphRunner::getInstance().getProvider("RobotPose") == "LogDataProvider" || ModuleGraphRunner::getInstance().getProvider("RobotPose") == "SimulatedWorldModelProvider") && SystemCall::getMode() == SystemCall::simulatedRobot ? 100.f : 0.f); // fast simulation scene robot shift

    // 3. Calculate variables, which disable and enable specific kicks or behavior
    // 3.1. Calculate variables, which disable and enable the side kick
    // get angles to the goal posts of own goal
    const Angle angleToLeftPost = Angle::normalize((leftGoalPostOwn - useBallPositionInField).angle() - theRobotPose.rotation - 180_deg); // TODO WHY -180deg? WTF
    const Angle angleToRightPost = Angle::normalize((rightGoalPostOwn - useBallPositionInField).angle() - theRobotPose.rotation + 180_deg);

    // --------------- can the sidewards outer kick be used? -------------- //
    const Angle angleToBall = (useBallPositionInField - theRobotPose.translation).angle();
    const Angle angleFromGoalToRobot = (theRobotPose.translation - Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f)).angle();
    const Angle angleFromGoalToBall = (useBallPositionInField - Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f)).angle();

    const Vector2f rotatedVectorRobotToBall = (useBallPositionInField.rotated(-angleFromGoalToBall) - theRobotPose.translation.rotated(-angleFromGoalToBall));
    bool mustStandBetweenBallAndGoal = (rotatedVectorRobotToBall.x() > 0.f && std::abs(angleFromGoalToRobot - angleToBall) < 90_deg) ||
    (Geometry::isPointInsideRectangle(penaltyRect, theRobotPose.translation) && std::abs((useBallPositionInField - theRobotPose.translation).angle()) < 90_deg);

    if(mustStandBetweenBallAndGoal)
      lastSidewardRestrictionTimestamp = theFrameInfo.time;
    else
      mustStandBetweenBallAndGoal |= theFrameInfo.getTimeSince(lastSidewardRestrictionTimestamp) < duelTimings.sidewardRestrictionTime;

    // side kick must use the following kick direction
    const Rangea forbiddingKickAngle = !mustStandBetweenBallAndGoal ? Rangea(-180_deg, 180_deg) : Rangea(angleToLeftPost, angleToRightPost < angleToLeftPost ? 179.9_deg : angleToRightPost);
    const Range forbiddingKickAngleExtra = !mustStandBetweenBallAndGoal ? Rangea(-180_deg, 180_deg) : (angleToRightPost < angleToLeftPost ? Rangea(-179_deg, angleToRightPost) : forbiddingKickAngle);

    const Angle angleFromBallToOpponent = (opponentOnField - useBallPositionInField).angle();

    // 3.2. Calculate variables, which disable and enable the forward steal kick
    // ------------- get the angle cone for the allowed kick directions of the forwardSteal ------------- //
    // TODO how do I make this less ugly :sob:
    const Angle forbiddingKickAngleMiddle = (Angle::normalize(Angle(std::max(angleToLeftPost, angleToRightPost)) + theRobotPose.rotation) + Angle::normalize(Angle(std::min(angleToLeftPost, angleToRightPost)) + theRobotPose.rotation)) / 2.f;
    const float forwardStealDirectionInterpolation = std::min(1.f, std::max(useBallPositionInField.x() - theFieldDimensions.xPosOwnGoalArea, 0.f) / (theFieldDimensions.xPosOwnGoalLine / 2.f - theFieldDimensions.xPosOwnGoalArea));

    const auto calculateForwardStealObstacleAngleRange = [&](const float directionFactor, Rangea& bestForwardStealRange,
                                                             Rangea& leftForwardStealRange, Rangea& rightForwardStealRange)
    {
      const Angle bestForwardStealAngle = directionFactor * 90_deg + (1.f - directionFactor) * (90_deg + forbiddingKickAngleMiddle);
      const Angle bestForwardStealAngleMirrored = Angle::normalize(bestForwardStealAngle - 180_deg);
      bestForwardStealRange = Rangea(std::min(bestForwardStealAngleMirrored, bestForwardStealAngle), std::max(bestForwardStealAngleMirrored, bestForwardStealAngle));
      leftForwardStealRange = Rangea(bestForwardStealRange.max - 5_deg, bestForwardStealRange.max + 5_deg);
      rightForwardStealRange = Rangea(bestForwardStealRange.min - 5_deg, bestForwardStealRange.min + 5_deg);

      // Angle cone where the duel obstacle must be relative to the ball, to allow the forwardSteal
      const bool lastDuelKickWasForwardSteal = theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight;
      const float forwardStealConeRatio = Rangef::ZeroOneRange().limit((opponentAndSelfDistanceToBallDiff + theBallSpecification.radius) / 100.f);
      const Angle useForwardStealBallOpponentPosition = !lastDuelKickWasForwardSteal ? stealBallParameters.forwardStealBallOpponentPositionMalus : // Normal Case
      Angle(forwardStealConeRatio * stealBallParameters.forwardStealBallOpponentPositionNormal.min + (1.f - forwardStealConeRatio) * stealBallParameters.forwardStealBallOpponentPositionNormal.max); // Use wider cone
      return Rangea(std::min(Angle((bestForwardStealRange.max + bestForwardStealRange.min) / 2.f - useForwardStealBallOpponentPosition), 0_deg), std::max(Angle((bestForwardStealRange.max + bestForwardStealRange.min) / 2.f + useForwardStealBallOpponentPosition), 0_deg));
    };

    Rangea leftForwardStealRange;
    Rangea rightForwardStealRange;
    Rangea bestForwardStealRange;
    Rangea leftAngleRangeBallToOpponent;
    const Rangea leftAngleRangeBallToOpponentNormal = calculateForwardStealObstacleAngleRange(1.f, bestForwardStealRange, leftForwardStealRange, rightForwardStealRange);
    if(forwardStealDirectionInterpolation != 1.f) // save computation time, if factor is equal to 1
    {
      // otherwise simple increase the range an obstacle needs to stand to allow the forward steal
      leftAngleRangeBallToOpponent = calculateForwardStealObstacleAngleRange(forwardStealDirectionInterpolation, bestForwardStealRange, leftForwardStealRange, rightForwardStealRange);
      leftAngleRangeBallToOpponent.min = std::min(leftAngleRangeBallToOpponent.min, leftAngleRangeBallToOpponentNormal.min);
      leftAngleRangeBallToOpponent.max = std::max(leftAngleRangeBallToOpponent.max, leftAngleRangeBallToOpponentNormal.max);
    }
    else
      leftAngleRangeBallToOpponent = leftAngleRangeBallToOpponentNormal;

    // 3.3. Calculate if the special behavior can be active, when we are standing orthogonal on the side to the opponent.
    // ----------- bonus for stealing the ball from the opponent, when we do not stand behind the ball / between ball and goal ----------- //
    bool bonusForStealBall = std::abs(theRobotPose.rotation) > stealBallParameters.stealBallMinRobotRotation && std::abs((useBallPositionInField - theRobotPose.translation).angle()) > stealBallParameters.stealBallMinPositionRotation;

    // set max allowed kick angle adjustment
    if(!bonusForStealBall)
    {
      // base on the distance to the ball, determine the max allowed rotation change
      const float distanceToBall = (useBallPositionRelative - duelObstacle.center).norm();
      if(distanceToBall < 100.f)
        minMaxAngle = searchParameters.minMaxAngleAngleRange.min;
      else if(distanceToBall > 800.f)
        minMaxAngle = searchParameters.minMaxAngleAngleRange.max;
      else
        minMaxAngle = std::max(((distanceToBall - 100.f) / (800.f - 100.f)), 0.f) * (searchParameters.minMaxAngleAngleRange.max - searchParameters.minMaxAngleAngleRange.min) + searchParameters.minMaxAngleAngleRange.min;
    }
    else
      minMaxAngle = stealBallParameters.bonusForStealBallMinMaxAngle;

    bonusForStealBall &= forwardStealDirectionInterpolation > 0.9f && opponentAndSelfDistanceToBallDiff < obstacleHandling.maxObstacleDistanceForSideStealBallKick;
    const Rangea clipRange(-180_deg, 180_deg);
    const Rangea stealBallMax = !bonusForStealBall ? clipRange : Rangea(stealBallParameters.stealBallRange.min - theRobotPose.rotation, stealBallParameters.stealBallRange.max - theRobotPose.rotation);
    const Rangea stealBallMin = !bonusForStealBall ? clipRange : Rangea(-stealBallParameters.stealBallRange.max - theRobotPose.rotation, -stealBallParameters.stealBallRange.min - theRobotPose.rotation);

    // 4. Calculate all kick angles, that shall be checked
    // Search near the current duel pose
    Angle passAngle;
    bool usePassAngle = false;
    if(theSkillRequestPose.skill == SkillRequest::pass)
    {
      passAngle = (theRobotPose.inverse() * theSkillRequestPose.target).translation.angle();
      usePassAngle = std::abs(passAngle - theDuelPose.kickAngle) > 5_deg;
    }
    if(!theDuelPose.noKick)
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle));
    directionPossibilities.push_back(0_deg);
    for(Angle i = searchParameters.rangeOfBestDuelPose / searchParameters.numOfAnglesNearBestDuelPose; i < searchParameters.rangeOfBestDuelPose; i += searchParameters.rangeOfBestDuelPose / searchParameters.numOfAnglesNearBestDuelPose)
    {
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle + i));
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle - i));
      if(usePassAngle)
      {
        directionPossibilities.push_back(Angle::normalize(passAngle + i));
        directionPossibilities.push_back(Angle::normalize(passAngle - i));
      }
    }

    // Overall search to maybe find something better
    for(Angle i = searchParameters.rangeOfBestDuelPose; i < searchParameters.overallSearchRange; i += (searchParameters.overallSearchRange - searchParameters.rangeOfBestDuelPose) / searchParameters.numOfOverallSearch)
    {
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle + i));
      directionPossibilities.push_back(Angle::normalize(theDuelPose.kickAngle - i));
    }

    // side steal angles
    if(bonusForStealBall)
    {
      directionPossibilities.push_back(Angle::normalize(stealBallMax.min + searchParameters.bonusForStealBallDirectionAdjustmentSmall));
      directionPossibilities.push_back(Angle::normalize(stealBallMin.max - searchParameters.bonusForStealBallDirectionAdjustmentSmall));
      directionPossibilities.push_back(Angle::normalize((stealBallMin.max + stealBallMax.max) / 2.f));
    }

    if(mustStandBetweenBallAndGoal)
      directionPossibilities.push_back(Angle::normalize((forbiddingKickAngle.min + forbiddingKickAngle.max) * 0.5f));
    else
      directionPossibilities.push_back(-theRobotPose.rotation);
    directionPossibilities.push_back(Angle::normalize(90_deg + theRobotPose.rotation));
    directionPossibilities.push_back(Angle::normalize(-90_deg + theRobotPose.rotation));

    // forward steal angles
    directionPossibilities.push_back(Angle::normalize(bestForwardStealRange.max - theRobotPose.rotation));
    directionPossibilities.push_back(Angle::normalize(bestForwardStealRange.min - theRobotPose.rotation));

    // minMaxAngle directions
    directionPossibilities.push_back(Angle::normalize(minMaxAngle + 45_deg));
    directionPossibilities.push_back(Angle::normalize(-minMaxAngle - 45_deg));

    if(theSkillRequestPose.skill != SkillRequest::pass)
    {
      const std::vector<Vector2f> passTargets = theFieldRating.getPossiblePassTargets();
      for(const Vector2f& passTarget : passTargets)
        directionPossibilities.push_back(Angle::normalize((passTarget - useBallPositionInField).angle() - theRobotPose.rotation));
    }

    // 5. Calculate the sector wheel. This is used to help to rate the different kick angles and kick ranges
    calculateSectorWheel();

    // 5.1 compute whether opponent is part of an obstacle wall
    bool obstacleWallBehindBall = false;
    for(const SectorWheel::Sector& sector : kickAngles)
    {
      if(sector.angleRange.isInside((leftAngleRangeBallToOpponent.max - leftAngleRangeBallToOpponent.max) * 0.5f) &&
         sector.type == SectorWheel::Sector::obstacle)
      {
        obstacleWallBehindBall = sector.distance -
                                 std::max(0.f, useBallPositionRelative.norm() - (theRobotDimensions.footLength + theBallSpecification.radius)) -
                                 ((ModuleGraphRunner::getInstance().getProvider("RobotPose") == "LogDataProvider" || ModuleGraphRunner::getInstance().getProvider("RobotPose") == "SimulatedWorldModelProvider") && SystemCall::getMode() == SystemCall::simulatedRobot ? 100.f : 0.f)
                                 <= obstacleHandling.maxObstacleDistanceForWalkStealBallKick;
        break;
      }
    }

    // 6. Based on the sector wheel, we want to make sure we will later check possible goal kick and pass angles
    {
      bool breakPassSearch = !usePassAngle;
      for(const SectorWheel::Sector& sector : kickAngles)
      {
        if(sector.type == SectorWheel::Sector::goal && sector.angleRange.getSize() > searchParameters.goalSectorWidth)
        {
          // only take the middle, because in case the goal kick is the best one, in the next computation cycle most kicks are sampled around this kick direction anyway
          directionPossibilities.push_back(Angle::normalize((sector.angleRange.max + sector.angleRange.min) / 2.f));

          // Check if we could do something, if we just search for more possible angles
          if(theFrameInfo.getTimeSince(timeSinceDoingNothing) > searchParameters.moreSearchAfterDoingNothing || theDuelPose.type != TargetType::goalShot)
          {
            directionPossibilities.push_back(Angle::normalize(sector.angleRange.max - searchParameters.bonusForStealBallDirectionAdjustmentSmall + theRobotPose.rotation));
            directionPossibilities.push_back(Angle::normalize(sector.angleRange.min + searchParameters.bonusForStealBallDirectionAdjustmentSmall + theRobotPose.rotation));
            directionPossibilities.push_back(Angle::normalize(sector.angleRange.max - searchParameters.bonusForStealBallDirectionAdjustmentBig + theRobotPose.rotation));
            directionPossibilities.push_back(Angle::normalize(sector.angleRange.min + searchParameters.bonusForStealBallDirectionAdjustmentBig + theRobotPose.rotation));
          }
        }

        if(!breakPassSearch && sector.angleRange.isInside(passAngle) &&
           sector.type == SectorWheel::Sector::obstacle && sector.distance < checkKickDistancesForFR.back())
        {
          breakPassSearch = true;
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.max + searchParameters.bonusForStealBallDirectionAdjustmentSmall));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.min - searchParameters.bonusForStealBallDirectionAdjustmentSmall));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.max + searchParameters.bonusForStealBallDirectionAdjustmentBig));
          directionPossibilities.push_back(Angle::normalize(sector.angleRange.min - searchParameters.bonusForStealBallDirectionAdjustmentBig));
        }
      }
    }

    // 7. Debug drawings
    // Field drawing for the angle cones for the forwardSteal kick direction and position of the duel obstacle
    COMPLEX_DRAWING("option:Zweikampf:wheelSteal")
    {
      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      wheel.begin(useBallPositionInField);
      wheel.addSector(leftForwardStealRange, std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      wheel.addSector(rightForwardStealRange, std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      wheel.addSector(leftAngleRangeBallToOpponent, std::numeric_limits<float>::max(), SectorWheel::Sector::obstacle);
      sectors = wheel.finish();
      DRAW_SECTOR_WHEEL("option:Zweikampf:wheelSteal", sectors, useBallPositionInField);
    }

    COMPLEX_DRAWING("option:Zweikampf:sideSteal")
    {
      if(stealBallMax != clipRange)
      {
        SectorWheel wheel;
        std::list<SectorWheel::Sector> sectors;
        wheel.begin(useBallPositionInField);
        wheel.addSector(Rangea(stealBallParameters.stealBallRange.min, stealBallParameters.stealBallRange.max), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        wheel.addSector(Rangea(-stealBallParameters.stealBallRange.max, -stealBallParameters.stealBallRange.min), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
        sectors = wheel.finish();
        DRAW_SECTOR_WHEEL("option:Zweikampf:sideSteal", sectors, useBallPositionInField);
      }
    }

    COMPLEX_DRAWING("option:Zweikampf:sidewardRange")
    {
      SectorWheel wheel;
      std::list<SectorWheel::Sector> sectors;
      wheel.begin(theFieldBall.positionOnField);
      Angle minNormal = std::min(Angle::normalize(forbiddingKickAngle.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngle.max + theRobotPose.rotation));
      Angle maxNormal = std::max(Angle::normalize(forbiddingKickAngle.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngle.max + theRobotPose.rotation));
      Angle minExtra = std::min(Angle::normalize(forbiddingKickAngleExtra.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngleExtra.max + theRobotPose.rotation));
      Angle maxExtra = std::max(Angle::normalize(forbiddingKickAngleExtra.min + theRobotPose.rotation), Angle::normalize(forbiddingKickAngleExtra.max + theRobotPose.rotation));
      wheel.addSector(Rangea(std::min(minNormal, minExtra), std::max(maxNormal, maxExtra)), std::numeric_limits<float>::max(), SectorWheel::Sector::goal);
      sectors = wheel.finish();
      DRAW_SECTOR_WHEEL("option:Zweikampf:sidewardRange", sectors, theFieldBall.positionOnField);
    }

    COMPLEX_DRAWING("option:Zweikampf:kickDirections")
    {
      int index = 0;
      for(const Angle kickAngle : directionPossibilities)
      {
        ColorRGBA color = ColorRGBA::black;
        if(index == 0)
          color = ColorRGBA::red;
        else if(index > 1 && index < searchParameters.numOfAnglesNearBestDuelPose * 2)
          color = ColorRGBA::blue;
        const Vector2f direction = Vector2f::polar(3000.f, kickAngle + theRobotPose.rotation);
        LINE("option:Zweikampf:kickDirections", useBallPositionInField.x(), useBallPositionInField.y(), useBallPositionInField.x() + direction.x(), useBallPositionInField.y() + direction.y(), 10, Drawings::solidPen, color);
        index++;
      }
    }

    TargetType highestPriority = TargetType::numOfTargetTypes;
    const Angle dribbleAngle = theDribbleTarget.calculateDribbleAngle(theFieldBall.positionOnField);
    const float extraGoalRange = (std::abs(dribbleAngle) < 45_deg + (shouldDribble ? 10_deg : 0_deg)) ? 1000.f : 0.f;

    // 8. Check every kick angle with every kick
    for(const Angle& kickAngle : directionPossibilities)
    {
      ASSERT(!std::isnan(kickAngle));
      // 8.1. Get max allowed kick range, before hitting an obstacle or the goal
      float maxKickRange = std::numeric_limits<float>::max();
      bool isGoalAngle = false;
      for(const SectorWheel::Sector& sector : kickAngles)
      {
        if(sector.angleRange.isInside(kickAngle))
        {
          if(sector.type == SectorWheel::Sector::goal && sector.angleRange.getSize() > searchParameters.goalSectorWidth)
            isGoalAngle = true;
          else if(sector.type == SectorWheel::Sector::obstacle)
            maxKickRange = sector.distance;
          break;
        }
      }

      // 8.2. Calculate distance for out of field, if in opponent half
      float distanceToFieldBorderSquared = std::numeric_limits<float>::max();

      if(!isGoalAngle)
      {
        Vector2f i1(0.f, 0.f);
        Vector2f i2(0.f, 0.f);
        VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(-opponentHalfFrontLeft, opponentHalfFrontLeft, Geometry::Line(useBallPositionInField, Vector2f(1.f, 0.f).rotated(kickAngle + theRobotPose.rotation)), i1, i2));
        const Vector2f ballToIntersection1 = i1 - useBallPositionInField;
        const Vector2f ballToIntersection2 = i2 - useBallPositionInField;
        // the 5_deg is just a arbitrary number, because the angle to the intersection point is only for a small fraction different to the original kickAngle, resulting from rounding errors.
        if(std::abs(ballToIntersection1.angle() - (kickAngle + theRobotPose.rotation)) < 5_deg)
          distanceToFieldBorderSquared = ballToIntersection1.squaredNorm() - sqr(100.f); // 100.f safe distance
        else if(std::abs(ballToIntersection2.angle() - (kickAngle + theRobotPose.rotation)) < 5_deg)
          distanceToFieldBorderSquared = ballToIntersection2.squaredNorm() - sqr(100.f); // 100.f safe distance
      }

      // 8.2.1 also calculate goal kick range
      // get the range for a goal kick
      // otherwise a long kick might overshoot the goal and gets a really bad rating
      float rangeForGoal = std::numeric_limits<float>::max();
      if(isGoalAngle && rangeForGoal == std::numeric_limits<float>::max())   // only when goal kick and only check once
      {
        Vector2f intersectionPoint;
        VERIFY(Geometry::getIntersectionOfLines(Geometry::Line(useBallPositionInField, Vector2f::polar(1.f, kickAngle + theRobotPose.rotation)), Geometry::Line(Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal), Vector2f(0.f, 1.f)), intersectionPoint));
        rangeForGoal = (intersectionPoint - useBallPositionInField).norm() + 0.01f;
      }

      // Check if the kickAngle is inside the side area, used to modify the rating from the potential field
      const bool isInsideStealBallRange = stealBallMin.contains(kickAngle) || stealBallMax.contains(kickAngle);
      const bool isTypeStealBall = (stealBallMin != stealBallMax &&
                                    (isInsideStealBallRange ||  // normal case, kickAngle is/is not in range. Special case: robot is so much rotated, that the ranges are not inside the -180_deg 180_deg range
                                     (!boundaryCheck.isInside(stealBallMin.min) && (stealBallMin.contains(Angle(kickAngle + 360_deg)) || stealBallMin.contains(Angle(kickAngle - 360_deg)))) ||
                                     (!boundaryCheck.isInside(stealBallMax.max) && (stealBallMax.contains(Angle(kickAngle + 360_deg)) || stealBallMax.contains(Angle(kickAngle - 360_deg))))));

      // 8.3. calculate the fieldRating once before hand to save computation time
      const Vector2f direction = Vector2f::polar(1.f, kickAngle + theRobotPose.rotation);
      RatingMapVector rangeRatingVector(isTypeStealBall, direction, rangeForGoal, theSkillRequestPose.passTarget, useBallPositionInField);

      // Prepare rating map
      for(const float range : checkKickDistancesForFR)
      {
        if(range > maxKickRange || sqr(range) > distanceToFieldBorderSquared) // every range above will not get executed anyway
          break;
        rangeRatingVector.ratingMap.emplace_back();
        rangeRatingVector.ratingMap.back().range = range;
      }
      if(rangeRatingVector.ratingMap.size() == 0)
        continue;

      // 8.4. check every InWalkKick
      for(KickInfo::KickType kickType : allowedKicks)
      {
        // forward steal is not allowed
        if((kickType == KickInfo::walkForwardStealBallLeft || kickType == KickInfo::walkForwardStealBallRight) &&
           (stealBallMin != stealBallMax || // we are not on front of the opponent, therefore the walkSteal kick would not work in time
            (!leftAngleRangeBallToOpponent.isInside(angleFromBallToOpponent) && !obstacleWallBehindBall) || // only if the opponent is standing behind the ball
            !(rightForwardStealRange.isInside(Angle::normalize(kickAngle + theRobotPose.rotation)) || // only for kicks going to the side
              leftForwardStealRange.isInside(Angle::normalize(kickAngle + theRobotPose.rotation))) || // only for kicks going to the side
            opponentAndSelfDistanceToBallDiff > obstacleHandling.maxObstacleDistanceForWalkStealBallKick)) // only if the opponent is closer to the ball
          continue;
        DuelPose pose;
        const bool possible = getDuelRating(kickType, kickAngle, pose,
                                            minMaxAngle, isGoalAngle,
                                            maxKickRange, distanceToFieldBorderSquared,
                                            stealBallMin, stealBallMax,
                                            angleFromBallToOpponent,
                                            rangeRatingVector, rangeForGoal,
                                            angleFromGoalToBall, highestPriority,
                                            extraGoalRange);
        if(!possible)
          continue;
        duelPoses[pose.type].push_back(pose);
        highestPriority = std::min(highestPriority, pose.type);
      }
    }

    // Find duelPose list with highest priority. GoalShots > StealBall > Pass > Others
    std::vector<DuelPose>* toBeCheckDuelPoses = nullptr;
    TargetType targetType = static_cast<TargetType>(0);
    for(auto& vec : duelPoses)
    {
      if(!vec.empty())
      {
        toBeCheckDuelPoses = &vec;
        break;
      }
      targetType = static_cast<TargetType>(targetType + 1);
    }
    if(toBeCheckDuelPoses)
    {
      DuelPose* bestPose = &(toBeCheckDuelPoses->front());
      // 9. Get the best kick
      // TODO use a std::map, with ENUM(GoalShots, ForwardSteal, Otherwise)
      // Only needs to check for the highest priority
      // Can skip lower ones already when still checking for kick angles! Can safe a lot of computation
      for(DuelPose& pose : *toBeCheckDuelPoses)
      {
        if(pose.rating < bestPose->rating) // Better rating is accepted
          bestPose = &pose;
      }

      // If a higher prio kick was decided last frame, keep the old kick for 100 ms
      if(theDuelPose.type >= bestPose->type || theFrameInfo.getTimeSince(kickForcedUpTime[theDuelPose.type]) > 100)
      {
        kickForcedUpTime[bestPose->type] = theFrameInfo.time;
        theDuelPose = *bestPose;
      }

      if(!shouldDribble && theDuelPose.type == TargetType::goalDribbleShot)
      {
        shouldDribble = true;
        lastDribbleRequest = theFrameInfo.time;
      }
      shouldDribble = !(shouldDribble && theMotionInfo.lastKickTimestamp > lastDribbleRequest);

      COMPLEX_DRAWING("option:Zweikampf:kicks")
        drawRating(*toBeCheckDuelPoses);

      // in case of a goal kick, always use the forwardLong
      if(replaceForwardWithLongGoalShot && theDuelPose.type == TargetType::goalShot)
      {
        if(theDuelPose.kickType == KickInfo::rlKickDynamic)
          theDuelPose.kickType = KickInfo::rlKickStrong;
      }

      if(theDuelPose.type == TargetType::goalShot)
      {
        for(const auto& sector : kickAngles)
        {
          if(sector.type == SectorWheel::Sector::goal && sector.angleRange.isInside(theDuelPose.kickAngle))
          {
            const Angle mostOptimalGoalDirectionRelative = (Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldBall.positionOnField).angle() - theRobotPose.rotation;
            const Angle sectorShiftValue = sector.angleRange.getSize() * obstacleHandling.goalSectorShiftFactor;
            const Rangea searchRange(sector.angleRange.min + sectorShiftValue, sector.angleRange.max - sectorShiftValue);
            theDuelPose.kickAngle = searchRange.limit(mostOptimalGoalDirectionRelative);
          }
        }
      }

      theDuelPose.precision = Rangea(0_deg, 0_deg);

      timeSinceDoingNothing = theFrameInfo.time;
      if(theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight)
        lastForwardSteal = theFrameInfo.time;
    }
    else
    {
      // 12. no kick was calculated for a longer period of time. Therefore just stand between the ball and the own goal, to make the life for the opponent hard
      if(((theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight) && theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.noKickStealMinTime) ||
         (!(theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight) && theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.noKickMinTime))
      {
        DuelPose noPose;
        const Angle rotation = (theFieldBall.positionOnField - Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f)).angle();
        noPose.pose = theRobotPose.inverse() * Pose2f(rotation, Vector2f::polar(-obstacleHandling.noKickBlockingPose, rotation) + theFieldBall.positionOnField);
        noPose.noKick = true;
        theDuelPose = noPose;
        theDuelPose.kickAngle = -theRobotPose.rotation;
      }
    }

    // 13.1. The perception of the obstacle might just be bad, so keep the forwardSteal as long as the new decision is not a goal kick or pass
    if((lastDuelPose.kickType == KickInfo::walkForwardStealBallLeft || lastDuelPose.kickType == KickInfo::walkForwardStealBallRight)
       && (theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight)
       && theFrameInfo.getTimeSince(lastForwardSteal) < 500 && theDuelPose.type != TargetType::goalShot && forceForwardSteal)
    {
      theDuelPose = lastDuelPose;  // use the steal kick that matches the new kick angle the most
      theDuelPose.kickAngle = lastKickAngle;
      theDuelPose.kickType = lastKickAngle > 0_deg ? KickInfo::walkForwardStealBallRight : KickInfo::walkForwardStealBallLeft;
      theDuelPose.noKick = false;
      if(theDuelPose.kickType != lastDuelPose.kickType)
        theDuelPose.kickAngle = Angle::normalize(theDuelPose.kickAngle + 180_deg);
      theDuelPose.precision = Rangea(-5_deg, 5_deg);
    }
    if((theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight) && (useBallPositionRelative - duelObstacle.center).squaredNorm() < sqr(obstacleHandling.maxObstacleDistanceToForceForwardSteal))
      forceForwardSteal = true;

    if(!(theDuelPose.kickType == KickInfo::walkForwardStealBallLeft || theDuelPose.kickType == KickInfo::walkForwardStealBallRight))
      forceForwardSteal = false;
  };

  /**
   * Clip ball position into the field
   */
  const auto calculateUseBallPosition = [&]
  {
    if(interpolateBallVelocity)
    {
      const Rangef ttrbRange(0.f, 1.f);
      const float ballVelocityFactor = mapToRange(theFieldBall.positionRelative.norm(), ballPositionInterpolationRange.min, ballPositionInterpolationRange.max, 0.f, 1.f);
      useBallPositionRelative = (1.f - ballVelocityFactor) * theFieldBall.positionRelative + ballVelocityFactor * theFieldBall.positionRelative;
      useBallPositionInField = theRobotPose * useBallPositionRelative;
      useBallPositionInField.x() = ballClipX.limit(useBallPositionInField.x());
      useBallPositionInField.y() = ballClipY.limit(useBallPositionInField.y());
    }
    else
    {
      useBallPositionInField = theFieldBall.positionOnField;
      useBallPositionInField.x() = ballClipX.limit(useBallPositionInField.x());
      useBallPositionInField.y() = ballClipY.limit(useBallPositionInField.y());
      useBallPositionRelative = theFieldBall.positionRelative;
    }
  };

  /**
   * Set up Zweikampf
   */
  const auto reset = [&]
  {
    forceForwardSteal = false;
    lastRobotPoseRotation = theRobotPose.rotation;
    calculateUseBallPosition();

    // Init based on the behavior
    updateSkillRequest();
    if(theSkillRequest.skill == SkillRequest::pass)
      theDuelPose.lastFieldEndPoint = theSkillRequestPose.target.translation;
    else
      theDuelPose.lastFieldEndPoint = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
    theDuelPose.kickAngle = (theDuelPose.lastFieldEndPoint - useBallPositionInField).angle() - theRobotPose.rotation;

    // to reduce computation time later, we want to get a list of all kick ranges, we want to precompute later
    // these ranges are the min and max ranges of all allowed kicks, and an interpolation between the forward and turn kick
    // 1. get interpolation ranges for forward and turn kick
    std::vector<float> distances = distanceParameters.additionalRanges;
    const float rangeStepSize = (theKickInfo[distanceParameters.mainKick].range.max - theKickInfo[distanceParameters.mainKick].range.min) / distanceParameters.numberOfSubRanges;
    for(int i = 1; i < distanceParameters.numberOfSubRanges; i++)
      distances.push_back(theKickInfo[distanceParameters.mainKick].range.min + rangeStepSize * i);

    // 2. get ranges for all kicks
    for(KickInfo::KickType kick : allowedKicks)
    {
      if(theKickInfo[kick].kickLeg == Legs::left) // skip clones
        continue;
      distances.push_back(theKickInfo[kick].range.min);
      distances.push_back(theKickInfo[kick].range.max);
    }

    // sort them
    std::sort(distances.begin(), distances.end(), [](const float& d1, const float& d2) { return d1 < d2; });

    // reset and save every unique range
    float lastRange = -1.f;
    for(float dis : distances)
    {
      if(std::abs(dis - lastRange) > 0.1f)
        checkKickDistancesForFR.push_back(dis);
      lastRange = dis;
    }
  };

  const auto convertKick = [&](KickInfo::KickType& kickType, Angle& kickDirection, float& kickRange)
  {
    if(kickType != KickInfo::walkForwardStealBallLeft && kickType != KickInfo::walkForwardStealBallRight)
      return;

    if(allowedKicks.end() == std::find_if(allowedKicks.begin(), allowedKicks.end(), [](const auto& kickType) {return kickType == KickInfo::rlKickDynamic;}))
    {
      FAIL("No rlKickDynamic available!");
    }

    if(theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentGoalLine - stealKickReplacementParameter.interpolateDirectionGoalDistance.max)
    {
      const float minBallGoalPostOffsetSquared = theFieldDimensions.goalPostRadius + theBallSpecification.radius;
      const Angle leftAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (leftGoalPost - useBallPositionInField).norm()));
      const Angle rightAngleOffset = std::asin(std::min(1.f, minBallGoalPostOffsetSquared / (rightGoalPost - useBallPositionInField).norm()));
      const Angle angleToLeftPost = Angle::normalize((leftGoalPost - useBallPositionInField).angle() - leftAngleOffset - theRobotPose.rotation - 2_deg);
      const Angle angleToRightPost = Angle::normalize((rightGoalPost - useBallPositionInField).angle() + rightAngleOffset - theRobotPose.rotation + 2_deg);

      const Rangea goalDirection(angleToLeftPost, angleToRightPost);
      const Angle replacementDirection = Angle::normalize(theRobotPose.rotation + kickDirection + (kickType == KickInfo::walkForwardStealBallLeft ? 1.f : -1.f) * stealKickReplacementParameter.staticShift);
      const Angle clippedDirection = goalDirection.limit(replacementDirection);
      const Angle interpolatedKickAngle = mapToRange(theFieldBall.positionOnField.x(), theFieldDimensions.xPosOpponentGoalLine - stealKickReplacementParameter.interpolateDirectionGoalDistance.max, theFieldDimensions.xPosOpponentGoalLine - stealKickReplacementParameter.interpolateDirectionGoalDistance.min, static_cast<float>(replacementDirection), static_cast<float>(clippedDirection));
      kickDirection = interpolatedKickAngle;
    }
    else
      kickDirection += (kickType == KickInfo::walkForwardStealBallLeft ? 1.f : -1.f) * stealKickReplacementParameter.staticShift;

    kickType = KickInfo::rlKickDynamic;

    Vector2f i1(0.f, 0.f);
    Vector2f i2(0.f, 0.f);
    VERIFY(Geometry::getIntersectionPointsOfLineAndRectangle(-opponentHalfFrontLeft, opponentHalfFrontLeft, Geometry::Line(useBallPositionInField, Vector2f(1.f, 0.f).rotated(kickDirection + theRobotPose.rotation)), i1, i2));
    const Vector2f ballToIntersection1 = i1 - useBallPositionInField;
    const Vector2f ballToIntersection2 = i2 - useBallPositionInField;
    // the 5_deg is just a arbitrary number, because the angle to the intersection point is only for a small fraction different to the original kickAngle, resulting from rounding errors.
    float distanceToFieldBorder = 0.f;
    if(std::abs(ballToIntersection1.angle() - (kickDirection + theRobotPose.rotation)) < 5_deg)
      distanceToFieldBorder = ballToIntersection1.squaredNorm() - sqr(100.f); // 100.f safe distance
    else if(std::abs(ballToIntersection2.angle() - (kickDirection + theRobotPose.rotation)) < 5_deg)
      distanceToFieldBorder = ballToIntersection2.squaredNorm() - sqr(100.f); // 100.f safe distance
    distanceToFieldBorder = std::max(stealKickReplacementParameter.distanceToFieldBorderRange.min, std::sqrt(distanceToFieldBorder) - stealKickReplacementParameter.distanceToFieldBorderRange.max);

    kickRange = std::min(distanceToFieldBorder, mapToRange(useBallPositionInField.x(), theFieldDimensions.centerCircleRadius, theFieldDimensions.xPosOpponentPenaltyArea, std::min(minKickRange.max, theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyArea * 0.5f), minKickRange.min));
  };

  initial_state(execute)
  {
    action
    {
      calculateUseBallPosition();
      calculateClosestObstacle(theObstacleModel, duelObstacle, duelObstacleMovedNearBall);
      for(std::size_t i = 0; i < TargetType::numOfTargetTypes; i++) // Handle jumps in logs
      {
        if(kickForcedUpTime[i] > theFrameInfo.time)
          kickForcedUpTime[i] = theFrameInfo.time;
      }

      // if ball is close and not moving -> interpolate the ball position based on the velocity for future use
      if(theFieldBall.positionRelative.squaredNorm() < sqr(250.f) && theBallModel.estimate.velocity.x() == 0.f && theBallModel.estimate.velocity.y() == 0.f)
        interpolateBallVelocity = true;
      // once the ball is far away again -> use intercepted end position of ball
      else if(interpolateBallVelocity && theFieldBall.positionRelative.squaredNorm() > sqr(500.f))
        interpolateBallVelocity = false;

      if(!option_time)
        reset();

      STOPWATCH("option:Zweikampf:duel")
      {
        updateSkillRequest();
        if(forcedInactive)
        {
          forcedInactive = theFrameInfo.getTimeSince(timeSinceDoingNothing) < duelTimings.inactiveAfterDoingNothing;
          DribbleToGoal({.alignPrecisely = KickPrecision::justHitTheBall});
        }
        else
        {
          calculateDuel();
          if(!theDuelPose.noKick)
          {
            float kickRange = theDuelPose.type == TargetType::goalShot ? 10000.f : theDuelPose.range; // Goal shots shall take the strongest version
            KickInfo::KickType kickType = theDuelPose.kickType;
            Angle kickDirection = theDuelPose.kickAngle;
            convertKick(kickType, kickDirection, kickRange);
            GoToBallAndKick({.targetDirection = Angle::normalize(kickDirection),
                             .kickType = kickType,
                             .lookActiveWithBall = theFieldBall.positionRelative.squaredNorm() > sqr(lookActiveMinBallDistance), // intentionally the current position!
                             .alignPrecisely = KickPrecision::justHitTheBall,
                             .length = kickRange});
          }
          else
          {
            WalkToPoint({.target = theDuelPose.pose,
                         .reduceWalkSpeedType = ReduceWalkSpeedType::noChange, // TODO no idea if normal or none would be better
                         .rough = true,
                         .disableStanding = true});
            LookActive({.withBall = true});
          }

          // check of opponent half is needed to prevent stupid stuff near the own goal, gets used in the next computation frame
          forcedInactive = theFrameInfo.getTimeSince(timeSinceDoingNothing) > duelTimings.maxTimeDoingNothing;
        }
      }
    }
  }
}
