/**
* @file LocalizationTeamBallProvider2014.cpp
*
* Implementation of a module that computes a combined ball model based on
* the reliability of teammates and the consistency of their obervations.
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#include "LocalizationTeamBallProvider2014.h"

MAKE_MODULE(LocalizationTeamBallProvider2014,modeling)

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
  for(auto const& teammate : theTeammateData.teammates)
  {
    if(teammate.status == Teammate::FULLY_ACTIVE &&
       theFrameInfo.getTimeSince(teammate.ball.timeWhenLastSeen) < timeout &&
       teammate.ball.estimate.velocity.norm() < maxBallVelocity &&
       (teammate.sideConfidence.confidenceState == SideConfidence::CONFIDENT || teammate.sideConfidence.confidenceState == SideConfidence::ALMOST_CONFIDENT))
       {
         BallObservation ball;
         ball.robotNumber = teammate.number;
         ball.isGoalkeeper = teammate.isGoalkeeper;
         ball.robotPose = teammate.pose;
         ball.ballPositionAbsolute = ball.robotPose * teammate.ball.estimate.position;
         if(!theFieldDimensions.isInsideCarpet(ball.ballPositionAbsolute))
         {
           continue;
         }
         ball.timeOfObservation = teammate.ball.timeWhenLastSeen;
         ball.sideConfidence = teammate.sideConfidence.sideConfidence;
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
    if(iter->isGoalkeeper)
    {
      goalieSawTheBall = true;
      break;
    }
  }
}

void LocalizationTeamBallProvider2014::addObservationToList(const LocalizationTeamBallProvider2014::BallObservation& ball)
{
  for(unsigned int i=0; i < balls.size(); ++i)
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
    bool clusterContainsGoalie = balls[i].isGoalkeeper;
    for(unsigned int j=0; j < balls.size(); ++j)
    {
      if((j != i) && observationsAreCompatible(balls[i], balls[j]))
      {
        cluster.observationIndizes.push_back(j);
        if(balls[j].isGoalkeeper)
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
  return (a.ballPositionAbsolute - b.ballPositionAbsolute).norm() < clusterThreshold;
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
  Vector2f avgBallPos(0.f, 0.f);
  for(unsigned int i=0; i<hypotheses[bestIdx].observationIndizes.size(); ++i)
  {
    BallObservation& ball = balls[hypotheses[bestIdx].observationIndizes[i]];
    if(ball.isGoalkeeper)
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
