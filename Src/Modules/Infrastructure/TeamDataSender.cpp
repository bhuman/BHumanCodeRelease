/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/
#include <math.h>
#include "TeamDataSender.h"
#include "Tools/Team.h"

MAKE_MODULE(TeamDataSender, Cognition Infrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if(theTeammateData.sendThisFrame)
  {
    ++sendFrames;

    TEAM_OUTPUT(idRobot, bin, theRobotInfo.number);
    TEAM_OUTPUT(idTeam, bin, theOwnTeamInfo.teamColor);

    // Own pose information and ball observation:
    TEAM_OUTPUT(idTeammateRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idTeammateSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idTeammateBallModel, bin, BallModelCompressed(theBallModel));
    TEAM_OUTPUT(idTeammateBallAge, bin, (theBallModel.timeWhenLastSeen ? theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) : -1));
    TEAM_OUTPUT(idTeammateGoalPercept, bin, theGoalPercept);

    // Obstacle stuff
    TEAM_OUTPUT(idObstacleClusters, bin, ObstacleClustersCompressed(theObstacleClusters, maxNumberOfObstacleClustersToSend));
    TEAM_OUTPUT(idTeammateObstacleModel, bin, ObstacleModelCompressed(theObstacleModel, maxNumberOfObstaclesToSend));

    // Robot status
    TEAM_OUTPUT(idTeammateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeammateHasGroundContact, bin, (theGroundContactState.contact && theMotionInfo.motion != MotionInfo::getUp && theMotionRequest.motion != MotionRequest::getUp));
    TEAM_OUTPUT(idTeammateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact)
    {
      TEAM_OUTPUT(idTeammateTimeSinceLastGroundContact, bin, theFrameInfo.time);
    }
    TEAM_OUTPUT(idTeamCameraHeight, bin, theCameraMatrix.translation.z);

    if(sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);

    // send intention
    if(theSideConfidence.confidenceState == SideConfidence::CONFUSED)
    {
      TEAM_OUTPUT(idTeammateIntention, bin, DROPIN_INTENTION_LOST);
    }
    else
    {
      TEAM_OUTPUT(idTeammateIntention, bin, TeammateData::getIntentionForRole(theBehaviorControlOutput.behaviorStatus.role));
    }
  }
}
