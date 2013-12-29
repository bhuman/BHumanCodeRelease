/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/

#include "TeamDataSender.h"
#include "Tools/Team.h"
#include "Representations/Modeling/ObstacleClusters.h"

MAKE_MODULE(TeamDataSender, Cognition Infrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if(theTeamMateData.sendThisFrame)
  {
    ++sendFrames;

    // Own pose information and ball observation:
    TEAM_OUTPUT(idTeamMateRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idTeamMateSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idTeamMateBallModel, bin, BallModelCompressed(theBallModel));

    // Obstacle stuff
    TEAM_OUTPUT(idObstacleClusters, bin, ObstacleClustersCompressed(theObstacleClusters, maxNumberOfObstacleClustersToSend));
    TEAM_OUTPUT(idTeamMateRobotsModel, bin, RobotsModelCompressed(theRobotsModel, maxNumberOfRobotsToSend));
    TEAM_OUTPUT(idTeamMateObstacleModel, bin,ObstacleModelCompressed(theObstacleModel, maxNumberOfObstaclesToSend));

    // Robot status
    TEAM_OUTPUT(idTeamMateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeamMateHasGroundContact, bin, theGroundContactState.contact);
    TEAM_OUTPUT(idTeamMateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact)
      TEAM_OUTPUT(idTeamMateTimeSinceLastGroundContact, bin, theFrameInfo.time);
    TEAM_OUTPUT(idTeamCameraHeight, bin, theCameraMatrix.translation.z);

    if(sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);
  }
}
