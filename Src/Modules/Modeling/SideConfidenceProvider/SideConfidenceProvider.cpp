/**
 * @file SideConfidenceProvider.h
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SideConfidenceProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"

MAKE_MODULE(SideConfidenceProvider, modeling)

SideConfidenceProvider::SideConfidenceProvider()
: lost(false),
  timeOfLastFall(0),
  lastTimeWithoutArmContact(0),
  timeOfLastBallObservation(0),
  timeOfLastTeamBallObservation(0)
{
  const Vector2f maxPos(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
  maxDistanceToFieldCenterForArmConsideration = maxPos.norm() / 2.f;
  maxDistanceToFieldCenterForFallDownConsideration = maxPos.norm() / 3.f;
}

void SideConfidenceProvider::update(SideConfidence& sideConfidence)
{
  // Debug stuff
  DECLARE_PLOT("module:SideConfidenceProvider:originalWeighting");
  DECLARE_PLOT("module:SideConfidenceProvider:mirroredWeighting");

  // Setting the time when the robot has a good stand
  if(theFallDownState.state != theFallDownState.upright &&
     theFallDownState.state != theFallDownState.undefined &&
     theFallDownState.state != theFallDownState.staggering &&
     theFrameInfo.getTimeSince(timeOfLastFall) > 8000)
    timeOfLastFall = theFrameInfo.time;

  // Setting the time when the robot not has arm contact
  if(!theArmContactModel.contactLeft && !theArmContactModel.contactRight)
    lastTimeWithoutArmContact = theFrameInfo.time;

  // Remove old entries from confidence buffer
  while(!confidenceBuffer.empty())
  {
    if(theFrameInfo.getTimeSince(confidenceBuffer.back().timeStamp) > maxBufferAge)
      confidenceBuffer.pop_back();
    else
      break;
  }

  updateBallConfidences(sideConfidence);
  updateSideConfidenceFromOthers(sideConfidence);
  updateSideConfidenceFromOwn(sideConfidence);
  updateConfidenceState(sideConfidence);

  // MOOOOH!
  if(sideConfidence.mirror)
    SystemCall::playSound("theMirrorCow.wav");
}

void SideConfidenceProvider::updateBallConfidences(SideConfidence& sideConfidence)
{
  // Check, if a mirrorcle has occured in the last frame
  if(sideConfidence.mirror)
  {
    confidenceBuffer.clear();
    averageBallConfidence = UNKNOWN;
    sideConfidence.mirror = false;
  }
  // Special handling for certain game states:
  if(theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
  {
    confidenceBuffer.clear();
    averageBallConfidence = UNKNOWN;
  }
  // Save current local ball observation:
  if((theBallModel.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0) &&
     (theFieldDimensions.isInsideField(theRobotPose * theBallModel.estimate.position)) &&
     (theBallModel.estimate.velocity.norm() <= maxBallVelocity) &&
     (theRobotPose * theBallModel.estimate.position).norm() > centerBanZoneRadius)
  {
    lastBallObservation = theBallModel.estimate.position;
    timeOfLastBallObservation = theFrameInfo.time;
  }
  // Odometry update for buffered perception
  else if(theFrameInfo.getTimeSince(timeOfLastBallObservation) < ballBufferingInterval)
  {
    lastBallObservation = theOdometer.odometryOffset.inverse() * lastBallObservation;
  }
  // Add current confidence to buffer:
  if((theFrameInfo.getTimeSince(timeOfLastBallObservation) < ballBufferingInterval) &&
     (theLocalizationTeamBall.isValid) &&
     (theLocalizationTeamBall.position.norm() > centerBanZoneRadius) &&
     (theLocalizationTeamBall.lastObservation > timeOfLastTeamBallObservation))
  {
    SideConfidenceMeasurement scm;
    scm.ballConfidence = computeCurrentBallConfidence();
    scm.timeStamp = timeOfLastBallObservation;
    confidenceBuffer.push_front(scm);
    timeOfLastBallObservation = 0; //For next confidence computation, a "fresh" observation is needed
    timeOfLastTeamBallObservation = theLocalizationTeamBall.lastObservation;
  }
  else
  {
    return;
  }
  // Check current buffer content for possible mirror situation
  if(!confidenceBuffer.full())
  {
    return;
  }
  size_t mirrorCount(0);
  size_t okCount(0);
  for(const SideConfidenceMeasurement& confidence : confidenceBuffer)
  {
    switch(confidence.ballConfidence)
    {
      case MIRROR: ++mirrorCount; break;
      case OK:     ++okCount; break;
    }
  }
  size_t unknownCount = confidenceBuffer.size() - mirrorCount - okCount;
  if((okCount == 0) && (mirrorCount > unknownCount))
    averageBallConfidence = MIRROR;
  else if((okCount > unknownCount) && (mirrorCount == 0))
    averageBallConfidence = OK;
  else
    averageBallConfidence = UNKNOWN;
}

SideConfidenceProvider::BallModelSideConfidence SideConfidenceProvider::computeCurrentBallConfidence()
{
  // Some constant parameters
  const float distanceObserved = lastBallObservation.norm();
  const float angleObserved = lastBallObservation.angle();
  const float& camZ = theCameraMatrix.translation.z();
  const float distanceAsAngleObserved = (pi_2 - std::atan2(camZ,distanceObserved));

  // Weighting for original pose
  float originalWeighting = computeAngleWeighting(angleObserved, theLocalizationTeamBall.position, theRobotPose,
                                                  standardDeviationBallAngle);
  originalWeighting *= computeDistanceWeighting(distanceAsAngleObserved, theLocalizationTeamBall.position, theRobotPose,
                                                camZ, standardDeviationBallDistance);

  // Weighting for mirrored pose
  const Pose2f  mirroredPose = Pose2f(pi) + (Pose2f)(theRobotPose);
  float mirroredWeighting = computeAngleWeighting(angleObserved, theLocalizationTeamBall.position, mirroredPose,
                                                  standardDeviationBallAngle);
  mirroredWeighting *= computeDistanceWeighting(distanceAsAngleObserved, theLocalizationTeamBall.position, mirroredPose,
                                                camZ, standardDeviationBallDistance);

  PLOT("module:SideConfidenceProvider:originalWeighting", originalWeighting);
  PLOT("module:SideConfidenceProvider:mirroredWeighting", mirroredWeighting);

  // Decide state based on weights
  if((mirroredWeighting < minWeighting) && (originalWeighting < minWeighting))
    return UNKNOWN;
  if((mirroredWeighting > weightingFactor * originalWeighting) || (originalWeighting > weightingFactor * mirroredWeighting))
    return mirroredWeighting > originalWeighting ? MIRROR : OK;
  return UNKNOWN;
}

float SideConfidenceProvider::computeAngleWeighting(float measuredAngle, const Vector2f& modelPosition,
                                                    const Pose2f& robotPose, float standardDeviation) const
{
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  return gaussianProbability(std::abs(Angle::normalize(modelAngle-measuredAngle)), standardDeviation);
}

float SideConfidenceProvider::computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2f& modelPosition,
                                                       const Pose2f& robotPose, float cameraZ, float standardDeviation) const
{
  const float modelDistance = (robotPose.translation - modelPosition).norm();
  const float modelDistanceAsAngle = (pi_2 - std::atan2(cameraZ,modelDistance));
  return gaussianProbability(std::abs(modelDistanceAsAngle-measuredDistanceAsAngle), standardDeviation);
}

void SideConfidenceProvider::updateSideConfidenceFromOthers(SideConfidence& sideConfidence)
{
  const float& best = theLocalizationTeamBall.ballStateOthersMaxSideConfidence;
  // The best other sideConfidence must be better than the own.. and it cannot be
  // the robot itself.
  if((best > sideConfidence.sideConfidence) || (theLocalizationTeamBall.numOfObservers >= minTeammateOverride))
  {
    if(averageBallConfidence == MIRROR)
    {
      sideConfidence.mirror = true;
      return;
      //confidence value does not change yet!
    }
    else if(averageBallConfidence == OK)
    {
      // I can trust my teammates and can update my confidence
      sideConfidence.sideConfidence = best;
    }
  }
  sideConfidence.mirror = false;
}

void SideConfidenceProvider::updateSideConfidenceFromOwn(SideConfidence& sideConfidence)
{
  // Not playing -> sideConfidence 100%
  if(theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
  {
    lost = false;
    sideConfidence.sideConfidence = sideConfidenceConfident;
    return;
  }

  // Playing but on the safe side -> sideConfidence 100%
  if(theOwnSideModel.stillInOwnSide)
  {
    sideConfidence.sideConfidence = sideConfidenceConfident;
    return;
  }
  // Leaving the safe side -> sideConfidence 95% and may get worse
  else if(sideConfidence.sideConfidence == sideConfidenceConfident)
  {
    sideConfidence.sideConfidence = sideConfidenceAlmostConfident;
  }

  //robot has fallen down
  if(theFrameInfo.getTimeSince(timeOfLastFall) == 0) // fallen now
  {
    const float distToFieldCenter(theRobotPose.translation.norm());
    if(distToFieldCenter < maxDistanceToFieldCenterForFallDownConsideration)
    {
      float currentFallDownModificator(1.0);
      const float rCircle = theFieldDimensions.centerCircleRadius;
      if(distToFieldCenter > rCircle)
      {
        currentFallDownModificator *= 1 - ((distToFieldCenter - rCircle) / (maxDistanceToFieldCenterForFallDownConsideration - rCircle));
      }
      sideConfidence.sideConfidence -= currentFallDownModificator;
    }
  }

  //arm contact, another robot may change my direction
  if(theFrameInfo.getTimeSince(lastTimeWithoutArmContact) > minContinuousArmContact)
  {
    const float distToFieldCenter(theRobotPose.translation.norm());
    if(distToFieldCenter < maxDistanceToFieldCenterForArmConsideration)
    {
      float currentArmContactModificator(armContactModificator);
      const float rCircle = theFieldDimensions.centerCircleRadius;
      if(distToFieldCenter > rCircle)
      {
        currentArmContactModificator *= 1 - ((distToFieldCenter - rCircle) / (maxDistanceToFieldCenterForArmConsideration - rCircle));
      }
      sideConfidence.sideConfidence -= currentArmContactModificator;
    }
    lastTimeWithoutArmContact = theFrameInfo.time;
  }

  //normalize
  if(sideConfidence.sideConfidence < sideConfidenceConfused)
    sideConfidence.sideConfidence = sideConfidenceConfused;

  //debug sound
  if(!lost && (sideConfidence.sideConfidence == sideConfidenceConfused))
  {
    lost = true;
    SystemCall::playSound("lost.wav");
  }
  else if(lost && (sideConfidence.sideConfidence > sideConfidenceConfused))
  {
    lost = false;
    SystemCall::playSound("allright.wav");
  }
}

void SideConfidenceProvider::updateConfidenceState(SideConfidence& sideConfidence)
{
  if(Global::getSettings().isGoalkeeper) // I AM THE GOALIE, RULER OF ALL SIDES!
  {
    sideConfidence.sideConfidence = sideConfidenceConfident;
    sideConfidence.mirror = false;
  }
  if(sideConfidence.sideConfidence == sideConfidenceConfident)
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  else if(sideConfidence.sideConfidence >= sideConfidenceAlmostConfident)
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;
  else if(sideConfidence.sideConfidence > sideConfidenceConfused)
    sideConfidence.confidenceState = SideConfidence::UNSURE;
  else
    sideConfidence.confidenceState = SideConfidence::CONFUSED;
}
