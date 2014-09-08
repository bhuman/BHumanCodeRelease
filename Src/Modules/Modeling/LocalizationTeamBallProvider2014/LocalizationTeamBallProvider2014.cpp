/**
* @file LocalizationTeamBallProvider2014.cpp
*
* Implementation of a module that computes a combined ball model based on
* the reliability of teammates and the consistency of their obervations.
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#include "LocalizationTeamBallProvider2014.h"

MAKE_MODULE(LocalizationTeamBallProvider2014,Modeling)

void LocalizationTeamBallProvider2014::update(LocalizationTeamBall& localizationTeamBall)
{
  // Initialize representation
  localizationTeamBall.isValid = false;
  hypotheses.clear();
  goalieSawTheBall = false;
  
  // Gather new information from all teammates
  updateObservations();
  
  // Cluster observations
  clusterObservations();
  
  // Compute representation
  computeTeamBallFromBestCluster(localizationTeamBall);
}

void LocalizationTeamBallProvider2014::updateObservations()
{
  // Check list of teammates for new observations:
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; i++)
  {
    if((i != theRobotInfo.number) &&
       (theTeammateData.isFullyActive[i]) &&
       (theFrameInfo.getTimeSince(theTeammateData.ballModels[i].timeWhenLastSeen) < timeout) &&
       (theTeammateData.ballModels[i].estimate.velocity.abs() < maxBallVelocity) &&
       (theTeammateData.robotsSideConfidence[i].confidenceState == SideConfidence::CONFIDENT || theTeammateData.robotsSideConfidence[i].confidenceState == SideConfidence::ALMOST_CONFIDENT))
    {
      BallObservation ball;
      ball.robotNumber = i;
      ball.robotPose = theTeammateData.robotPoses[i];
      ball.ballPositionAbsolute = ball.robotPose * theTeammateData.ballModels[i].estimate.position;
      if(!theFieldDimensions.isInsideCarpet(ball.ballPositionAbsolute))
      {
        continue;
      }
      ball.timeOfObservation = theTeammateData.ballModels[i].timeWhenLastSeen;
      ball.sideConfidence = theTeammateData.robotsSideConfidence[i].sideConfidence;
      ball.cameraZ = theTeammateData.cameraHeights[i];
      addObservationToList(ball);
    }
  }
  // Remove old observations:
  std::vector<BallObservation>::iterator iter = balls.begin();
  while(iter != balls.end())
  {
    if(theFrameInfo.getTimeSince(iter->timeOfObservation) > timeout)
      iter = balls.erase(iter);
    else
      ++iter;
  }
  // Update goalie observation information
  for(iter = balls.begin(); iter != balls.end(); ++iter)
  {
    if(iter->robotNumber == 1)
    {
      goalieSawTheBall = true;
      break;
    }
  }
}

void LocalizationTeamBallProvider2014::addObservationToList(const LocalizationTeamBallProvider2014::BallObservation& ball)
{
  for(unsigned int i=0; i<balls.size(); ++i)
  {
    if(balls[i].robotNumber == ball.robotNumber)
    {
      balls[i] = ball;
      return;
    }
  }
  balls.push_back(ball);
}

void LocalizationTeamBallProvider2014::clusterObservations()
{
  for(unsigned int i=0; i<balls.size(); ++i)
  {
    TeamBallHypothesis cluster;
    cluster.observationIndizes.push_back(i);
    bool clusterContainsGoalie = balls[i].robotNumber == 1;
    for(unsigned int j=0; j<balls.size(); ++j)
    {
      if((j != i) && observationsAreCompatible(balls[i], balls[j]))
      {
        cluster.observationIndizes.push_back(j);
        if(balls[j].robotNumber == 1)
          clusterContainsGoalie = true;
      }
    }
    if(preferGoalieClusters && goalieSawTheBall && !clusterContainsGoalie)
      continue;
    else
      hypotheses.push_back(cluster);
  }
}

bool LocalizationTeamBallProvider2014::observationsAreCompatible(const LocalizationTeamBallProvider2014::BallObservation& a,
                                                                 const LocalizationTeamBallProvider2014::BallObservation& b)
{
  return (a.ballPositionAbsolute - b.ballPositionAbsolute).abs() < clusterThreshold;
}

void LocalizationTeamBallProvider2014::computeTeamBallFromBestCluster(LocalizationTeamBall& localizationTeamBall)
{
  if(hypotheses.size() == 0)
    return;
  unsigned int bestIdx = 0;
  for(unsigned int i = 1; i<hypotheses.size(); i++)
  {
    if(hypotheses[i].observationIndizes.size() > hypotheses[bestIdx].observationIndizes.size())
      bestIdx = i;
  }
  localizationTeamBall.isValid = true;
  localizationTeamBall.numOfObservers = (int) hypotheses[bestIdx].observationIndizes.size();
  localizationTeamBall.goalieHasObserved = false;
  localizationTeamBall.ballStateOthersMaxSideConfidence = -1.f;
  localizationTeamBall.lastObservation = 0;
  Vector2<> avgBallPos(0.f,0.f);
  for(unsigned int i=0; i<hypotheses[bestIdx].observationIndizes.size(); ++i)
  {
    BallObservation& ball = balls[hypotheses[bestIdx].observationIndizes[i]];
    if(ball.robotNumber == 1)
    {
      localizationTeamBall.goalieHasObserved = true;
    }
    if(ball.sideConfidence > localizationTeamBall.ballStateOthersMaxSideConfidence)
    {
      localizationTeamBall.ballStateOthersMaxSideConfidence = ball.sideConfidence;
    }
    if(ball.timeOfObservation > localizationTeamBall.lastObservation)
    {
      localizationTeamBall.lastObservation = ball.timeOfObservation;
    }
    avgBallPos += ball.ballPositionAbsolute;
  }
  localizationTeamBall.position = avgBallPos / (float) localizationTeamBall.numOfObservers;
}
