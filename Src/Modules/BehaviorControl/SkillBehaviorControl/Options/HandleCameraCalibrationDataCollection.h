option(HandleCameraCalibrationDataCollection)
{
  const Pose2f startPose = Pose2f(0.f, 0.f);
  const Pose2f firstWalkingPose = Pose2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
  const Vector2f sidePoint = startPose.translation + Vector2f(0, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f halfWayWalkingPoint = startPose.translation + 0.5 * (firstWalkingPose.translation - startPose.translation);
  const float patternHeight = 580.f;
  const Vector2f patternPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
  const int lookAtPointDuration = 10000;
  const int settleDuration = 2000;
    common_transition
        {
          if(!theGameState.isPlaying())
            goto notPlaying;
        }

    state(goToStart)
    {
      transition
      {
        if(theWalkToPointSkill.isDone())
          goto firstLookAtPoint;
      }

      action
      {
        theLookActiveSkill({.ignoreBall = true});
        theWalkToPointSkill({.target = theRobotPose.inverse() * startPose});
      }
    }

  state(firstLookAtPoint)
  {
    transition
    {
        if(state_time > lookAtPointDuration)
          goto turnLeft;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(turnLeft)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto lookOverRightShoulder;
    }

    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      const Pose2f targetPose = Pose2f(90_deg, startPose.translation);
      theWalkToPointSkill({.target = theRobotPose.inverse() * targetPose});
    }
  }

  state(lookOverRightShoulder)
  {
    transition
    {
      if(state_time > lookAtPointDuration)
        goto walkSidewaysToward;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkSidewaysToward)
  {
    transition
    {
      if(theRobotPose.translation.x() > firstWalkingPose.translation.x())
        goto firstSettle;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkAtRelativeSpeedSkill({.speed = {0.f, -1.f}});
    }
  }

  state(firstSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkSidewaysAway;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkSidewaysAway)
  {
    transition
    {
      if(theRobotPose.translation.x() < startPose.translation.x())
        goto turnRight;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkAtRelativeSpeedSkill({.speed = {0.f, 1.f}});
    }
  }

  state(turnRight)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto lookOverLeftShoulder;
    }

    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      const Pose2f targetPose = theRobotPose.rotation > 60_deg ? Pose2f(0_deg, startPose.translation) : Pose2f(-90_deg, startPose.translation);
      theWalkToPointSkill({.target = theRobotPose.inverse() * targetPose});
    }
  }

  state(lookOverLeftShoulder)
  {
    transition
    {
      if(state_time > lookAtPointDuration)
        goto walkBackwardsWhileLookingToOneSide;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkBackwardsWhileLookingToOneSide)
  {
    transition
    {
      if(theRobotPose.translation.y() > sidePoint.y())
        goto secondSettle;
    }

    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkAtRelativeSpeedSkill({.speed = {-1.f, 0.f}});
    }
  }

  state(secondSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkForwardsWhileLookingOverOneShoulder;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkForwardsWhileLookingOverOneShoulder)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto walkCurve;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkToPointSkill({.target = theRobotPose.inverse() * Pose2f(-90_deg, startPose.translation + Vector2f(0, sidePoint.y()/2))});
    }
  }

  state(walkCurve)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto thirdSettle;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkToPointSkill({.target = theRobotPose.inverse() * Pose2f(halfWayWalkingPoint)});
    }
  }

  state(thirdSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkStrait;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkStrait)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto forthSettle;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkToPointSkill({.target = theRobotPose.inverse() * Pose2f(firstWalkingPose)});
    }
  }

  state(forthSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkBackwards;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkBackwards)
  {
    transition
    {
      if(theRobotPose.translation.x() < halfWayWalkingPoint.x())
        goto fithSettle;
    }

    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkAtRelativeSpeedSkill({.speed = {-1.f, 0.f}});
    }
  }

  state(fithSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkSideways;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkSideways)
  {
    transition
    {
      if(theRobotPose.translation.y() > sidePoint.y())
        goto sixtSettle;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkAtRelativeSpeedSkill({.speed = {0.f, 1.f}});
    }
  }

  state(sixtSettle)
  {
    transition
    {
      if(state_time > settleDuration)
        goto walkDiagonal;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theStandSkill();
    }
  }

  state(walkDiagonal)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto lookActive;
    }
    action
    {
      const Vector2f targetRelative = theRobotPose.inverse() * patternPosition;
      theLookAtPointSkill({.target = (Vector3f() << targetRelative, patternHeight).finished(),
                              .camera = HeadMotionRequest::upperCamera});
      theWalkToPointSkill({.target = theRobotPose.inverse() * firstWalkingPose,
                           .forceSideWalking = true});
    }
  }

  state(lookActive)
  {
    transition
    {
      if(state_time > lookAtPointDuration)
        goto lookActiveBackwardsWalking;
    }
    action
    {
      theLookActiveSkill({.ignoreBall = false});
      theStandSkill();
    }
  }

  state(lookActiveBackwardsWalking)
  {
    transition
    {
      if(theRobotPose.translation.x() < halfWayWalkingPoint.x())
        goto walkStraitLookActive;
    }

    action
    {
      theLookActiveSkill({.ignoreBall = false});
      theWalkAtRelativeSpeedSkill({.speed = {-1.f, 0.f}});
    }
  }

  state(walkStraitLookActive)
  {
    transition
    {
      if(theWalkToPointSkill.isDone())
        goto finishedDataCollection;
    }
    action
    {
      theLookActiveSkill({.ignoreBall = true});
      theWalkToPointSkill({.target = theRobotPose.inverse() * Pose2f(firstWalkingPose)});
    }
  }

  target_state(finishedDataCollection)
  {
    action
    {
      theLookForwardSkill();
      theStandSkill();
    }
  }

    initial_state(notPlaying)
    {
      transition
      {
        if(theGameState.isPlaying())
          goto goToStart;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }
}
