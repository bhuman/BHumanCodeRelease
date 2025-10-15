#include "SkillBehaviorControl.h"

option((SkillBehaviorControl)HandleFastestWalkLeaderboard,
       load((float) yTouchlineOffset,
            (std::vector<Vector2f>) targetsOfInterest))
{
  // the locations the robot wants to walk to in field coordinates
  const Vector2f forwardFinishLocation(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightTouchline - yTouchlineOffset);
  const Vector2f slalomFinishLocation((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGoalLine) / 2, theFieldDimensions.yPosRightTouchline - yTouchlineOffset);

  const Vector2f robotTargetPos(theFieldDimensions.xPosOwnGoalLine + theRobotDimensions.footLength * 2.5f,
                                theFieldDimensions.yPosRightTouchline - theRobotDimensions.footLength * 2.5f);

  auto getClosestTarget = [&]() -> Vector2f
  {
    for(Vector2f target : targetsOfInterest)
    {
      if(theRobotPose.translation.y() > target.y())
        return target;
    }
    return slalomFinishLocation;
  };

  initial_state(initial)
  {
    transition
    {
      FastestWalkState::WalkState walkState = theFastestWalkState.state;
      switch(walkState)
      {
        case FastestWalkState::WalkState::forward:
          goto forwardWalk;
          break;
        case FastestWalkState::WalkState::slalom:
          goto slalomWalk;
          break;
      }
    }
  }

  state(slalomWalk)
  {
    const Pose2f finishPose = theRobotPose.inverse() * Pose2f(-90_deg, slalomFinishLocation);
    transition
    {
      if(theRobotPose.translation.x() > robotTargetPos.x() && theRobotPose.translation.y() < robotTargetPos.y())
        goto finish;
    }
    action
    {
      LookActive();
      WalkToPoint({.target = finishPose,
                   .reduceWalkSpeedType = ReduceWalkSpeedType::noChange,
                   .disableStanding = true,
                   .disableAvoidFieldBorder = true,
                   .targetOfInterest = theRobotPose.inverse() * getClosestTarget(),
                   .sideWalkingRequest = SideWalkingRequest::notAllowed});
    }
  }

  state(forwardWalk)
  {
    const Pose2f finishPose = theRobotPose.inverse() * Pose2f(-90_deg, forwardFinishLocation);
    transition
    {
      if(theRobotPose.translation.x() > robotTargetPos.x() && theRobotPose.translation.y() < robotTargetPos.y())
        goto finish;
    }
    action
    {
      LookForward();
      WalkToPoint({.target = finishPose,
                   .reduceWalkSpeedType = ReduceWalkSpeedType::noChange,
                   .disableObstacleAvoidance = true,
                   .disableStanding = true,
                   .disableAvoidFieldBorder = true,
                   .sideWalkingRequest = SideWalkingRequest::notAllowed});
    }
  }

  state(finish)
  {
    transition
    {
      const Pose2f finishPose = theRobotPose.inverse() * Pose2f(-90_deg, slalomFinishLocation);
      if(!(theRobotPose.translation.x() > robotTargetPos.x() && theRobotPose.translation.y() < robotTargetPos.y()))
      {
        FastestWalkState::WalkState walkState = theFastestWalkState.state;
        switch(walkState)
        {
          case FastestWalkState::WalkState::forward:
            goto forwardWalk;
            break;
          case FastestWalkState::WalkState::slalom:
            goto slalomWalk;
            break;
        }
      }
    }
    action
    {
      LookActive();
      Stand();
    }
  }
}
