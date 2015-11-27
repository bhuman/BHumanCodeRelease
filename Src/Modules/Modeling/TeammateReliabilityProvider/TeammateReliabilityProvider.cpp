/**
* @file TeammateReliabilityProvider.cpp
*
* Implementation of a module that tries to estimate how reliable the robots of the own team are.
* The main application is the SPL drop-in competition.
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#include "TeammateReliabilityProvider.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(TeammateReliabilityProvider,modeling)

void TeammateReliabilityProvider::update(TeammateReliability& teammateReliability)
{
  const int minPlayerNum = Global::getSettings().lowestValidPlayerNumber;
  const int maxPlayerNum = Global::getSettings().highestValidPlayerNumber;
  if(teammateReliability.states.size() != static_cast<unsigned int>(maxPlayerNum + 1))
    teammateReliability.states.resize(maxPlayerNum  + 1);
  if(teammates.size() != static_cast<unsigned int>(maxPlayerNum + 1))
    teammates.resize(maxPlayerNum  + 1);
  for(int i=minPlayerNum; i <= maxPlayerNum; ++i)
  {
    if(i == theRobotInfo.number)
      continue;
    updateTeammateInformation(theReceivedSPLStandardMessages.messages[i],
                              theReceivedSPLStandardMessages.timestamps[i], i,
                              teammates[i]);
    computeReliability(teammates[i], teammateReliability.states[i]);
  }
}

// TODO:
// (5.) Die Varianz der Positionen berechnen


void TeammateReliabilityProvider::updateTeammateInformation(const SPLStandardMessage& msg,
                                                            unsigned timestamp, int robotNumber,
                                                            TeammateReliabilityProvider::TeammateInformation& teammate)
{
  // No new information about this robot, leave internal information untouched
  if(timestamp == teammate.lastPackageReceived)
    return;
  // The teammate communicates the first time after a longer timeout (reboot or what ever) => Reinitialize information
  if(theFrameInfo.getTimeSince(teammate.lastPackageReceived) > timeout)
  {
    TeammateInformation tm;
    teammate = tm;
  }
  // Is this the first package that we receive?
  if(teammate.firstPackageReceived == 0)
  {
    teammate.firstPackageReceived = timestamp;
  }
  // We have new information about a teammate
  teammate.lastPackageReceived = timestamp;
  teammate.isPenalized = theOwnTeamInfo.players[robotNumber-1].penalty != 0;
  teammate.hasInvalidPose = !(theFieldDimensions.isInsideCarpet(Vector2f(msg.pose[0], msg.pose[1])) && std::abs(msg.pose[2]) <= pi);
  teammate.seesBallOutsideField = !(theFieldDimensions.isInsideCarpet(Vector2f(msg.ball[0], msg.ball[1])));
  if(!teammate.isPenalized)
  {
    if(theGameInfo.state == STATE_READY)
    {
      teammate.receivedPackagesInReady++;
      teammate.xPosRange.add(msg.pose[0]);
      teammate.yPosRange.add(msg.pose[1]);
      teammate.rotRange.add(msg.pose[2]);
    }
    if(theGameInfo.state == STATE_PLAYING)
    {
      teammate.receivedPackagesInPlaying++;
      teammate.xPosRange.add(msg.pose[0]);
      teammate.yPosRange.add(msg.pose[1]);
      teammate.rotRange.add(msg.pose[2]);
      // Check, if the teammate perceives the ball at the same position than us:
      if(msg.ballAge < 0.5f && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 100)
      {
        Pose2f teammatePose(msg.pose[2], msg.pose[0], msg.pose[1]);
        Vector2f teammateRelBall(msg.ball[0], msg.ball[1]);
        Vector2f teammateAbsBall = Transformation::robotToField(teammatePose, teammateRelBall);
        Vector2f ownAbsBall = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
        if((ownAbsBall - teammateAbsBall).norm() < maxBallAgreementDistance)
          teammate.lastTimeAgreedOnBall = theFrameInfo.time;
      }
    }
  }
}

void TeammateReliabilityProvider::computeReliability(const TeammateReliabilityProvider::TeammateInformation& teammate,
                                                     TeammateReliability::ReliabilityState& state)
{
  // Currently, we do not communicate with this robot :-(
  if(theFrameInfo.getTimeSince(teammate.lastPackageReceived) > timeout)
  {
    state = TeammateReliability::UNKNOWN;
    return;
  }
  // Switch to default state, if communication starts
  if(state == TeammateReliability::UNKNOWN)
  {
    state = TeammateReliability::CHECKING;
  }
  // If we have received enough packages, we try to make a guess. Initially, the other robot is unreliable
  if(state == TeammateReliability::CHECKING)
  {
    if(teammate.receivedPackagesInReady + teammate.receivedPackagesInPlaying > minPackagesNeeded)
    {
      state = TeammateReliability::UNRELIABLE;
    }
    else
    {
      return;
    }
  }
  // If the robot sends stuff that does not make sense, it has to be considered as unreliable:
  if(state != TeammateReliability::UNKNOWN && state != TeammateReliability::CHECKING)
  {
    if(teammate.hasInvalidPose || teammate.seesBallOutsideField)
      state = TeammateReliability::UNRELIABLE;
  }
  // The robot receives an upgrade, if the position values are within a reasonable range
  if(state == TeammateReliability::UNRELIABLE)
  {
    if(teammate.xPosRange.getSize() > expectedMinTranslationRange &&
       teammate.yPosRange.getSize() > expectedMinTranslationRange &&
       teammate.rotRange.getSize() > expectedMinRotationRange)
    {
      state = TeammateReliability::OK;
    }
    else
    {
      return;
    }
  }
  // Cool, the teammate and I agree on the ball. This is GOOD!
  if(theFrameInfo.getTimeSince(teammate.lastTimeAgreedOnBall) < ballAgreementTimeout)
  {
    state = TeammateReliability::GOOD;
  }
  else
  {
    state = TeammateReliability::OK;
  }
}
