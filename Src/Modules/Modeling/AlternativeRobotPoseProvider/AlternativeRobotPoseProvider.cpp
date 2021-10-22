/**
 * @file AlternativeRobotPoseProvider.cpp
 *
 * Implementation of a module that uses recent field feature observations
 * and combines them to an alternative pose of the robot.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "AlternativeRobotPoseProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(AlternativeRobotPoseProvider, modeling);

void AlternativeRobotPoseProvider::update(AlternativeRobotPoseHypothesis& alternativeRobotPoseHypothesis)
{
  clusters.clear();
  alternativeRobotPoseHypothesis.isValid = false;
  if(currentStateOrMotionRuinsOdometry() || // Things are going on and might (but must not) ruin the odometry completely
     (theExtendedGameInfo.timeSinceLastCalibrationStarted > 5000 && theRobotInfo.mode == RobotInfo::calibration)) // Robot is calibrating
  {
    observations.clear();
    return;
  }
  removeOldObservations();
  odometryUpdate();

  // Buffer new features, if possible:
  if(currentMotionAllowsAcceptanceOfNewFeatures())
  {
    addFieldFeatureToBuffer(&theMidCircle);
    addFieldFeatureToBuffer(&thePenaltyArea);
    addFieldFeatureToBuffer(&thePenaltyMarkWithPenaltyAreaLine);
  }

  drawObservations();
  if(observations.empty())
    return;
  clusterObservations();
  unsigned int bestClusterIdx = 0;
  int bestClusterSize = clusters[0].numOfPoses;
  for(unsigned int i = 1; i < clusters.size(); i++)
  {
    if(clusters[i].numOfPoses > bestClusterSize)
    {
      bestClusterIdx = i;
      bestClusterSize = clusters[i].numOfPoses;
    }
  }
  alternativeRobotPoseHypothesis.pose = clusters[bestClusterIdx].pose;
  alternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate = clusters[bestClusterIdx].timeOfNewestObservation;
  alternativeRobotPoseHypothesis.isInOwnHalf = clusters[bestClusterIdx].isInOwnHalf;
  alternativeRobotPoseHypothesis.numOfContributingObservations = bestClusterSize;
  alternativeRobotPoseHypothesis.isValid = theFieldDimensions.isInsideCarpet(alternativeRobotPoseHypothesis.pose.translation);
}

bool AlternativeRobotPoseProvider::currentStateOrMotionRuinsOdometry()
{
  // The robot was manually placed:
  if(theGameInfo.state == STATE_SET && !theGroundContactState.contact)
    return true;
  // There is nothing reasonable to do before the robot finally enters READY:
  if(theGameInfo.state == STATE_INITIAL)
    return true;
  // Some motion phases can be troublesome as the odometry offset between configurations cannot be determined:
  if(theMotionInfo.isMotion(bit(MotionPhase::playDead) | bit(MotionPhase::fall) | bit(MotionPhase::getUp)))
    return true;
  // Everything else if probably OK:
  return false;
}

bool AlternativeRobotPoseProvider::currentMotionAllowsAcceptanceOfNewFeatures()
{
  // Anything else than walking or standing might lead to wrong features:
  if(!theMotionInfo.isMotion(bit(MotionPhase::walk) | bit(MotionPhase::stand)))
    return false;
  // Making fast turns produces strange projections of lines to the ground:
  // TODO: Move hardcoded number to parameter. Value is guessed by Philip.
  if(std::abs(theGyroState.mean.z()) > 50_deg)
    return false;
  // Everything else if probably OK:
  return true;
}


void AlternativeRobotPoseProvider::addFieldFeatureToBuffer(const FieldFeature* ff)
{
  if(ff->isValid)
  {
    const FieldFeature::RobotPoseToFF poses = ff->getGlobalRobotPosition();
    const Pose2f& ownSidePose = poses.pos1.translation.x() <= 0.f ? poses.pos1 : poses.pos2;
    PoseObservation newObservation;
    newObservation.pose              = ownSidePose;
    newObservation.timeOfObservation = theFrameInfo.time;
    newObservation.stillInOwnHalf    = theSideInformation.robotMustBeInOwnHalf;
    observations.push_front(newObservation);
    if(!newObservation.stillInOwnHalf) // We have to consider the alternative, too
    {
      const Pose2f& oppSidePose = poses.pos1.translation.x() > 0.f ? poses.pos1 : poses.pos2;
      newObservation.pose = oppSidePose;
      // timeOfObservation and stillInOwnHalf have the same value as for the first new pose
      observations.push_front(newObservation);
    }
  }
}

void AlternativeRobotPoseProvider::removeOldObservations()
{
  int timeToKeepObservation = maxTimeToKeepObservation;
  while(!observations.empty())
  {
    if(theFrameInfo.getTimeSince(observations.back().timeOfObservation) > timeToKeepObservation)
      observations.pop_back();
    else
      break;
  }
}

void AlternativeRobotPoseProvider::clusterObservations()
{
  if(observations.empty())
    return;
  const unsigned numOfClusters = std::min<unsigned>(maxClusters, static_cast<unsigned>(observations.size()));
  for(unsigned int i = 0; i < numOfClusters; i++)
  {
    const Pose2f& base = observations[i].pose;
    Cluster c;
    float angleX = 0.f;
    float angleY = 0.f;
    for(unsigned int j = 0; j < observations.size(); j++)
    {
      const Pose2f& oPose = observations[j].pose;
      if(std::abs(base.translation.x() - oPose.translation.x()) < translationDifference &&
         std::abs(base.translation.y() - oPose.translation.y()) < translationDifference &&
         std::abs(Angle::normalize(base.rotation - oPose.rotation)) < rotationDifference)
      {
        c.numOfPoses++;
        c.pose.translation += oPose.translation;
        angleX += std::cos(oPose.rotation);
        angleY += std::sin(oPose.rotation);
        if(observations[j].timeOfObservation > c.timeOfNewestObservation)
          c.timeOfNewestObservation = observations[j].timeOfObservation;
        if(observations[j].stillInOwnHalf)
          c.isInOwnHalf = true;
      }
    }
    ASSERT(c.numOfPoses > 0); // Should not happen!
    ASSERT(c.timeOfNewestObservation > 0); // Should not happen!
    c.pose.translation /= static_cast<float>(c.numOfPoses);
    c.pose.rotation = std::atan2(angleY, angleX);
    clusters.push_back(c);
  }
}

void AlternativeRobotPoseProvider::odometryUpdate()
{
  for(unsigned int i = 0; i < observations.size(); i++)
  {
    PoseObservation& obs = observations[i];
    obs.pose += theOdometer.odometryOffset;
  }
}

void AlternativeRobotPoseProvider::drawObservations()
{
  DECLARE_DEBUG_DRAWING("module:AlternativeRobotPoseProvider:buffer", "drawingOnField");
  for(unsigned int i = 0; i < observations.size(); i++)
  {
    const PoseObservation& obs = observations[i];
    ColorRGBA col = obs.stillInOwnHalf ? ColorRGBA(180, 180, 255) : ColorRGBA(255, 0, 0);
    DRAW_ROBOT_POSE("module:AlternativeRobotPoseProvider:buffer", obs.pose, col);
  }
}
