/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/
#include "TeamDataSender.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type,format,expression) \
{ Global::getTeamOut().format << expression;\
Global::getTeamOut().finishMessage(type); }


MAKE_MODULE(TeamDataSender, cognitionInfrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if(theTeammateData.sendThisFrame)
  {
    ++sendFrames;

    TEAM_OUTPUT(idRobot, bin, theRobotInfo.number);

    // Own pose information and ball observation:
    TEAM_OUTPUT(idRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idBallModel, bin, BallModelCompressed(theBallModel));
    TEAM_OUTPUT(idTeammateBallAge, bin, static_cast<float>(theBallModel.timeWhenLastSeen ? theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f : -1.f));
    TEAM_OUTPUT(idGoalPercept, bin, theGoalPercept);

    // Obstacle stuff
    TEAM_OUTPUT(idObstacleModelCompressed, bin, ObstacleModelCompressed(theObstacleModel, maxNumberOfObstaclesToSend));

    // Information about the behavior (i.e. the robot's state and intentions)
    TEAM_OUTPUT(idBehaviorStatus, bin, theBehaviorStatus);
    TEAM_OUTPUT(idSPLStandardBehaviorStatus, bin, theSPLStandardBehaviorStatus);
    TEAM_OUTPUT(idTeammateRoles, bin, theTeammateRoles);

    // Robot status
    TEAM_OUTPUT(idTeammateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeammateHasGroundContact, bin, (theGroundContactState.contact && theMotionInfo.motion != MotionInfo::getUp && theMotionRequest.motion != MotionRequest::getUp));
    TEAM_OUTPUT(idTeammateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact)
    {
      TEAM_OUTPUT(idTeammateTimeOfLastGroundContact, bin, theFrameInfo.time);
    }

    TEAM_OUTPUT(idWhistle, bin, theWhistle);

    if(sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);
  }
}
