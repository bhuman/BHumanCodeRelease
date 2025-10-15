#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleBestKickLeaderboard,
       load((ENUM_INDEXED_ARRAY(KickInfo::KickType, BestKickState::Target)) kickTypes,
            (ENUM_INDEXED_ARRAY(float, BestKickState::Target)) kickLengths,
            (ENUM_INDEXED_ARRAY(Angle, BestKickState::Target)) kickAngles))
{
  initial_state(kickBall)
  {
    transition
    {
      if(action_done)
        goto finish;
    }
    action
    {
      BestKickState::Target target = theBestKickState.target;
      GoToBallAndKick({.targetDirection = Angle::normalize(-theRobotPose.rotation + kickAngles[target]),
                       .kickType = kickTypes[target],
                       .alignPrecisely = KickPrecision::precise,
                       .length = kickLengths[target]});
    }
  }

  state(finish)
  {
    action
    {
      LookForward();
      Stand();
    }
  }
}
