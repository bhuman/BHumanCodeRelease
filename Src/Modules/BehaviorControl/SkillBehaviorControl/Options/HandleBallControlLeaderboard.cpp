#include "Debugging/DebugDrawings.h"

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl)HandleBallControlLeaderboard,
       load((float) maxDiffToTarget,
            (std::vector<Vector2f>) dribbleTargets,
            (float) maximumExpectedKickDistance,
            (float) obstacleSafetyMargin,
            (float) robotSafetyPerimiter,
            (Vector2f) finishTarget),
       vars((unsigned int)(0) currentTargetIndex))
{
  const Vector2f ballTargetPos(theFieldDimensions.xPosOwnGoalLine + theBallSpecification.radius * 4.f,
                               theFieldDimensions.yPosRightTouchline - theBallSpecification.radius * 4.f);
  const Vector2f robotTargetPos(theFieldDimensions.xPosOwnGoalLine + theRobotDimensions.footLength * 2.5f,
                                theFieldDimensions.yPosRightTouchline - theRobotDimensions.footLength * 2.5f);


/**
 * Determines which foot to use based on nearest obstacle position with hysteresis
 * @return KickInfo::KickType for the safer foot choice
 */
auto selectSaferFoot = [&]() -> KickInfo::KickType
{
  // Static variables for hysteresis (persistent across calls)
  static Vector2f lastNearestObstacle = Vector2f::Zero();
  static bool hasLastObstacle = false;
  static float lastFootChoice = 0.f; // -1.0 = right, +1.0 = left

  float nearestObstacleDistance = std::numeric_limits<float>::max();
  Vector2f nearestObstacleRelative = Vector2f::Zero();
  bool foundObstacle = false;

  // Find nearest obstacle in robot-relative coordinates
  for(const auto& obstacle : theObstacleModel.obstacles)
  {
    // ObstacleModel is already in relative coordinates according to comment
    Vector2f obstacleRelative = obstacle.center;
    float distanceSquared = obstacleRelative.squaredNorm(); // Use squaredNorm for performance

    if(distanceSquared < nearestObstacleDistance * nearestObstacleDistance)
    {
      nearestObstacleDistance = std::sqrt(distanceSquared);
      nearestObstacleRelative = obstacleRelative;
      foundObstacle = true;
    }
  }

  // If no obstacles found, use default behavior
  if(!foundObstacle)
  {
    hasLastObstacle = false;
    return KickInfo::KickType::walkForwardsLeft; // Default choice
  }

  // Check if this is the same obstacle as last frame (hysteresis)
  const float sameObstacleThreshold = 0.3f; // 30cm threshold for "same obstacle"
  bool isSameObstacle = hasLastObstacle &&
                       (nearestObstacleRelative - lastNearestObstacle).squaredNorm() <
                       sameObstacleThreshold * sameObstacleThreshold;

  // Calculate raw preference based on obstacle position
  float rawPreference = nearestObstacleRelative.y(); // Positive = left side

  // Special case: if ball is very close to obstacle, prefer straight kick
  // rather than trying to kick away from obstacle
  const float ballObstacleDistance = (theFieldBall.positionRelative - nearestObstacleRelative).norm();
  const float criticalDistance = 0.4f; // 40cm - if ball is this close to obstacle

  if(ballObstacleDistance < criticalDistance)
  {
    // Ball is very close to obstacle - prefer straight kick
    // Reduce the influence of obstacle position
    float reductionFactor = ballObstacleDistance / criticalDistance;
    rawPreference *= reductionFactor;
  }

  // Apply smooth interpolation to avoid oscillation
  const float interpolationFactor = 0.3f; // How much to blend with previous choice
  const float maxDistance = 2.0f; // Max distance to consider for normalization

  // Normalize preference to [-1, 1] range
  float normalizedPreference = std::clamp(rawPreference / maxDistance, -1.0f, 1.0f);

  // Apply interpolation with previous choice to smooth transitions
  float smoothedPreference;
  if(hasLastObstacle && isSameObstacle)
  {
    smoothedPreference = (1.0f - interpolationFactor) * normalizedPreference +
                        interpolationFactor * lastFootChoice;
  }
  else
  {
    smoothedPreference = normalizedPreference;
  }

  // Update static variables for next frame
  lastNearestObstacle = nearestObstacleRelative;
  hasLastObstacle = true;
  lastFootChoice = smoothedPreference;

  // Convert smoothed preference to kick type
  // Use a small deadzone around 0 to avoid jittering
  const float deadzone = 0.1f;

  if(smoothedPreference > deadzone)
  {
    return KickInfo::KickType::walkForwardsLeft; // Obstacle on left, use left foot
  }
  else if(smoothedPreference < -deadzone)
  {
    return KickInfo::KickType::walkForwardsRight; // Obstacle on right, use right foot
  }
  else
  {
    // In deadzone - stick with previous choice or default
    return lastFootChoice > 0 ? KickInfo::KickType::walkForwardsLeft :
                                KickInfo::KickType::walkForwardsRight;
  }
};


  /**
   * Checks if kicking the ball in a given direction for MINIMUM_KICK_DISTANCE is safe
   * @param ballPos Current ball position
   * @param direction Normalized direction vector
   * @return true if safe (no obstacles or field boundary violations), false otherwise
   */
  auto isKickDirectionSafe = [&](const Vector2f& ballPos, const Vector2f& direction) -> bool
  {
    // Calculate where ball would be after minimum kick distance
    Vector2f projectedBallPos = ballPos + direction * maximumExpectedKickDistance;

    // Check if projected position is still inside the field boundaries
    // Explicit checks for goal line boundaries: xPosOwnGoalLine to xPosOpponentGoalLine
    if(projectedBallPos.x() < theFieldDimensions.xPosOwnGoalLine ||
       projectedBallPos.x() > theFieldDimensions.xPosOpponentGoalLine ||
       projectedBallPos.y() < theFieldDimensions.yPosRightTouchline ||
       projectedBallPos.y() > theFieldDimensions.yPosLeftTouchline)
      return false;

    // Check for collisions with obstacles (other robots)
    for(const auto& obstacle : theObstacleModel.obstacles)
    {
      // Calculate obstacle radius from left/right points
      float obstacleRadius = (obstacle.left - obstacle.right).norm() * 0.5f + obstacleSafetyMargin;

      // Check if projected ball position is too close to obstacle center
      float distanceToObstacle = (projectedBallPos - obstacle.center).norm();
      if(distanceToObstacle < obstacleRadius)
        return false;

      // Check if robot itself would collide with obstacle when positioning for kick
      // Robot needs space to approach ball and position for kick
      float robotRadius = theRobotDimensions.footLength * 0.5f + robotSafetyPerimiter;
      float totalSafetyDistance = obstacleRadius + robotRadius;

      // Check distance from current robot position to obstacle
      float robotDistanceToObstacle = (theRobotPose.translation - obstacle.center).norm();
      if(robotDistanceToObstacle < totalSafetyDistance)
        return false;

      // Check distance from ball position to obstacle (where robot needs to go)
      float ballDistanceToObstacle = (ballPos - obstacle.center).norm();
      if(ballDistanceToObstacle < totalSafetyDistance)
        return false;
    }

    return true;
  };

  /**
   * Calculates a safe target direction that accounts for minimum kick distance
   * @param ballPos Current ball position
   * @param targetPos Original dribble target position
   * @return Safe direction vector (not normalized) to aim for
   */
  auto calculateSafeTargetDirection = [&](const Vector2f& ballPos, const Vector2f& targetPos) -> Vector2f
  {
    Vector2f originalDirection = targetPos - ballPos;
    Vector2f originalDirectionNorm = originalDirection.normalized();

    // Check if direct direction is safe
    if(isKickDirectionSafe(ballPos, originalDirectionNorm))
      return originalDirection;

    // Direct direction is unsafe - need to adjust target to account for 60cm overshoot
    Vector2f projectedBallPos = ballPos + originalDirectionNorm * maximumExpectedKickDistance;

    // Adjust target position to compensate for minimum kick distance
    // The idea: aim at a point such that after 60cm kick, ball ends up at original target
    Vector2f adjustedTarget = targetPos - originalDirectionNorm * maximumExpectedKickDistance;

    // If the adjusted target would be too close to current ball position, use alternative approach
    float distanceToAdjustedTarget = (adjustedTarget - ballPos).norm();
    if(distanceToAdjustedTarget < maximumExpectedKickDistance * 0.5f)
    {
      // Target is too close - use a safe direction parallel to goal line instead
      Vector2f safeDirection;

      // Check if goal line is a problem
      if(projectedBallPos.x() < theFieldDimensions.xPosOwnGoalLine)
      {
        // Would go over own goal line - aim parallel to it
        safeDirection = Vector2f(0.f, targetPos.y() - ballPos.y());
      }

      // Ensure we have a valid direction
      if(safeDirection.squaredNorm() > 0.1f)
        return safeDirection;
    }

    // Use the adjusted target direction
    return adjustedTarget - ballPos;
  };

  initial_state(initial)
  {
    transition
    {
      goto dribble;
    }
  }

  state(ballSearch)
  {
    transition
    {
      if(theFieldBall.ballWasSeen())
        goto dribble;
    }
    action
    {
      DemoSearchForBall();
    }
  }

  state(dribble)
  {
    DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
    {
      for(unsigned int i = 0; i < dribbleTargets.size(); i++)
      {
        auto penColor = i == currentTargetIndex ? ColorRGBA::cyan : ColorRGBA::blue;
        CIRCLE("option:BallControlLeaderboard",
               dribbleTargets[i].x(), dribbleTargets[i].y(), maxDiffToTarget, 0,
               Drawings::solidPen, penColor,
               Drawings::noBrush, penColor);
        DRAW_TEXT("option:BallControlLeaderboard",
                  dribbleTargets[i].x(), dribbleTargets[i].y(), 100,
                  penColor, std::to_string(i));
      }
    }
    transition
    {
      ASSERT(currentTargetIndex < dribbleTargets.size());
      // if the ball is next to the current target, we dribble to the next target
      if((theFieldBall.positionOnField - dribbleTargets[currentTargetIndex]).squaredNorm() < sqr(maxDiffToTarget))
        currentTargetIndex++;
      if(currentTargetIndex == dribbleTargets.size()) // the last target was reached
        goto dribbleOverTouchline;
      if (!theFieldBall.ballWasSeen(2000)) {
         goto ballSearch;
      }
    }

    action
    {
      ASSERT(currentTargetIndex < dribbleTargets.size());

      // Calculate safe target direction accounting for minimum kick distance
      Vector2f safeTargetDirection = calculateSafeTargetDirection(theFieldBall.positionOnField, dribbleTargets[currentTargetIndex]);
      Vector2f safeTargetDirectionNorm = safeTargetDirection.normalized();

      // If current target direction is still unsafe, try next target as fallback
      if(!isKickDirectionSafe(theFieldBall.positionOnField, safeTargetDirectionNorm) &&
         currentTargetIndex + 1 < dribbleTargets.size())
      {
        Vector2f nextTargetDirection = calculateSafeTargetDirection(theFieldBall.positionOnField, dribbleTargets[currentTargetIndex + 1]);
        Vector2f nextTargetDirectionNorm = nextTargetDirection.normalized();

        if(isKickDirectionSafe(theFieldBall.positionOnField, nextTargetDirectionNorm))
        {
          safeTargetDirection = nextTargetDirection;
          safeTargetDirectionNorm = nextTargetDirectionNorm;
        }
      }

      // Debug visualization for safety adjustments
      DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
      {
        // Only show original direction if it's different from adjusted direction
        Vector2f originalDirection = (dribbleTargets[currentTargetIndex] - theFieldBall.positionOnField).normalized();
        Vector2f originalProjected = theFieldBall.positionOnField + originalDirection * maximumExpectedKickDistance;
        bool originalSafe = isKickDirectionSafe(theFieldBall.positionOnField, originalDirection);

        // Only draw original direction if it differs significantly from adjusted direction
        if((safeTargetDirection.normalized() - originalDirection).squaredNorm() > 0.1f)
        {
          ColorRGBA originalColor = originalSafe ? ColorRGBA::green : ColorRGBA::red;
          LINE("option:BallControlLeaderboard",
               theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(),
               originalProjected.x(), originalProjected.y(), 10, Drawings::dottedPen, originalColor);
        }

        // Show adjusted safe direction - thinner line to reduce clutter
        Vector2f adjustedProjected = theFieldBall.positionOnField + safeTargetDirectionNorm * maximumExpectedKickDistance;
        bool adjustedSafe = isKickDirectionSafe(theFieldBall.positionOnField, safeTargetDirectionNorm);
        ColorRGBA adjustedColor = adjustedSafe ? ColorRGBA::green : ColorRGBA::orange;

        LINE("option:BallControlLeaderboard",
             theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y(),
             adjustedProjected.x(), adjustedProjected.y(), 15, Drawings::solidPen, adjustedColor);

        // Small circle at end - reduced size
        CIRCLE("option:BallControlLeaderboard", adjustedProjected.x(), adjustedProjected.y(), 30, 8,
               Drawings::solidPen, adjustedColor, Drawings::solidBrush, adjustedColor);
      }

      Angle dribblingAngle = Angle::normalize(safeTargetDirection.angle());
      auto ballSpeed = theFieldBall.velocityRelative.squaredNorm();

      if(ballSpeed <= 0.5f)
      {
        // Select safer foot based on obstacle positions
        KickInfo::KickType saferKickType = selectSaferFoot();

        // Single debug marker for kick state - positioned away from ball
        DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
        {
          Vector2f statusPos = theFieldBall.positionOnField + Vector2f(150.f, 100.f); // Offset from ball
          CROSS("option:BallControlLeaderboard", statusPos.x(), statusPos.y(),
                60, 12, Drawings::solidPen, ColorRGBA::green);
          DRAW_TEXT("option:BallControlLeaderboard", statusPos.x() + 80, statusPos.y(),
                    60, ColorRGBA::green, "KICK");
        }

        float kickLength = dribblingAngle > 135_deg && dribblingAngle < -135_deg ? 0.2f : 0.f;
        GoToBallAndKick({.targetDirection = Angle::normalize(dribblingAngle - theRobotPose.rotation),
                         .kickType = saferKickType,
                         .lookActiveWithBall = true,
                         .alignPrecisely = KickPrecision::precise,
                         .length = kickLength, // make kicks as short as possible
                         .preStepType = PreStepType::notAllowed
                        });
      }
      else
      {
        // Calculate positioning relative to ball - just move back 5cm from ball without rotation
        Vector2f positionAdjustment = Vector2f(-50.f - theBallSpecification.radius - theRobotDimensions.footLength/2.f, 0.f); // 5cm behind ball in robot coordinates
        Vector2f targetPositionRelative = theFieldBall.positionRelative + positionAdjustment;
        auto targetDirection = Angle::normalize(dribblingAngle - theRobotPose.rotation);

        // Create pose with current robot orientation to avoid unwanted rotation
        Pose2f targetPoseRelative(theRobotPose.rotation, targetDirection); // No rotation change, just position
        // GoToBallAndDribble({.targetDirection = targetDirection,
        //                 .alignPrecisely = KickPrecision::precise,
        //                 .kickLength = 0.f, // make kicks as short as possible
        //                 .lookActiveWithBall = true});
        LookActive();
        WalkToPoint({.target = targetPoseRelative,
                     .disableAligning = true,        // Prevent rotation alignment - key to stop side-stepping
                     .disableAvoidFieldBorder = true,
                    });

                    // Single debug marker for wait state - positioned away from other elements
                    DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
      {
        Vector2f statusPos = theFieldBall.positionOnField + Vector2f(-150.f, -100.f); // Different offset
          // CROSS("option:BallControlLeaderboard", statusPos.x(), statusPos.y(),
          //       60, 12, Drawings::solidPen, ColorRGBA::red);
          DRAW_TEXT("option:BallControlLeaderboard", statusPos.x() + 80, statusPos.y(),
                    60, ColorRGBA::red, "WAIT");

          // Show wait position with smaller, less intrusive marker
          Vector2f targetPositionAbsolute = theRobotPose * targetPositionRelative;
          CIRCLE("option:BallControlLeaderboard", targetPositionAbsolute.x(), targetPositionAbsolute.y(),
                 25, 8, Drawings::solidPen, ColorRGBA::cyan, Drawings::noBrush, ColorRGBA::cyan);
        }
      }
    }
  }

  state(dribbleOverTouchline)
  {
    transition
    {
      if(theFieldBall.positionOnField.x() > ballTargetPos.x() && theFieldBall.positionOnField.y() < ballTargetPos.y())
        goto walkOverTouchline;
    }
    action
    {
      Vector2f dribbleTarget(std::max(finishTarget.x(), theFieldBall.positionOnField.x()), finishTarget.y());
      Angle dribblingAngle = (dribbleTarget - theFieldBall.positionOnField).angle();
      GoToBallAndDribble({.targetDirection = Angle::normalize(dribblingAngle - theRobotPose.rotation),
                          .alignPrecisely = KickPrecision::precise,
                          .kickLength = 0.2f, // make kicks slightly longer
                          .lookActiveWithBall = true});
      DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
      {
        CROSS("option:BallControlLeaderboard",
              dribbleTarget.x(), dribbleTarget.y(), 40, 12,
              Drawings::solidPen, ColorRGBA::cyan);
        DRAW_TEXT("option:BallControlLeaderboard",
                  dribbleTarget.x() + 50, dribbleTarget.y() - 100, 100,
                  ColorRGBA::cyan, "finish");
      }
    }
  }

  state(walkOverTouchline)
  {
    transition
    {
      if(theRobotPose.translation.x() > robotTargetPos.x() && theRobotPose.translation.y() < robotTargetPos.y())
        goto finish;
    }
    action
    {
      Vector2f walkTargetAbsolute(std::max(finishTarget.x(), theFieldBall.positionOnField.x()), finishTarget.y());
      Vector2f walkTargetRelative = theRobotPose.inverse() * walkTargetAbsolute;
      WalkToPoint({.target = walkTargetRelative, .disableAvoidFieldBorder = true});
      LookForward();
      DEBUG_DRAWING("option:BallControlLeaderboard", "drawingOnField")
      {
        CROSS("option:BallControlLeaderboard",
              walkTargetAbsolute.x(), walkTargetAbsolute.y(), 40, 12,
              Drawings::solidPen, ColorRGBA::cyan);
        DRAW_TEXT("option:BallControlLeaderboard",
                  walkTargetAbsolute.x() + 50, walkTargetAbsolute.y() - 100, 100,
                  ColorRGBA::cyan, "finish");
      }
    }
  }

  state(finish)
  {
    transition
    {
      if(!(theFieldBall.positionOnField.x() > ballTargetPos.x() && theFieldBall.positionOnField.y() < ballTargetPos.y()))
        goto dribbleOverTouchline;
      if(!(theRobotPose.translation.x() > robotTargetPos.x() && theRobotPose.translation.y() < robotTargetPos.y()))
        goto walkOverTouchline;
    }
    action
    {
      LookActive();
      Stand();
    }
  }
}
