/**
 * @file SelfLocator.cpp
 *
 * Implements a class that performs self-localization
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SelfLocator.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>

using namespace std;

SelfLocator::SelfLocator() : perceptRegistration(theCameraInfo, theCameraMatrix, theCirclePercept,
  theFieldDimensions, theFrameInfo, theGameInfo,
  theOwnTeamInfo, theGoalFrame, theFieldLineIntersections,
  theFieldLines, theMidCircle, theMidCorner, theOuterCorner,
  thePenaltyArea, thePenaltyMarkPercept, goalFrameIsPerceivedAsLines,
  lineAssociationCorridor, longLineAssociationCorridor, centerCircleAssociationDistance,
  penaltyMarkAssociationDistance, intersectionAssociationDistance,
  minimumLineLengthForAssociationToLongHorizontalLine,
  globalPoseAssociationMaxDistanceDeviation, globalPoseAssociationMaxAngularDeviation,
  covarianceScalarLongHorizontalLine, minLengthLongHorizontalLine, yDifferenceLongHorizontalLine),
  lastTimeFarFieldBorderSeen(0), lastTimeJumpSound(0), timeOfLastReturnFromPenalty(0), lastTimeNotInStandWalkKick(0),
  nextSampleNumber(0), idOfLastBestSample(-1), averageWeighting(0.5f), lastAlternativePoseTimestamp(0),
  lastTimePenaltyMarkSeen(0), lastTimeCirclePerceptSeen(0), validitiesHaveBeenUpdated(false)
{
  //              //
  //  3        4  //
  //  2        5  //
  //  1       (6) //
  //     Goal     //
  walkInPositions.clear();
  float robotPlacementDistance = 1000.f; // Standard distance between two robots on normal size field
  const float xLengthOfHalf = theFieldDimensions.xPosOpponentGroundline;
  if(xLengthOfHalf < 4000.f)             // Small field, robots need to stand closer to each other
    robotPlacementDistance *= xLengthOfHalf / 4000.f;
  const Vector2f pos1(-3.f * robotPlacementDistance, theFieldDimensions.yPosLeftSideline  + walkInYModificator);
  const Vector2f pos2(-2.f * robotPlacementDistance, theFieldDimensions.yPosLeftSideline  + walkInYModificator);
  const Vector2f pos3(-1.f * robotPlacementDistance, theFieldDimensions.yPosLeftSideline  + walkInYModificator);
  const Vector2f pos4(-1.f * robotPlacementDistance, theFieldDimensions.yPosRightSideline - walkInYModificator);
  const Vector2f pos5(-2.f * robotPlacementDistance, theFieldDimensions.yPosRightSideline - walkInYModificator);
  const Vector2f pos6(-3.f * robotPlacementDistance, theFieldDimensions.yPosRightSideline - walkInYModificator);
  const Vector2f centerOfGoal(theFieldDimensions.xPosOwnGroundline, 0.f);
  const Vector2f rightPenaltyAreaCorner(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f leftPenaltyAreaCorner(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f pos1ToRightCorner  = rightPenaltyAreaCorner - pos1;
  const Vector2f pos2ToRightCorner  = rightPenaltyAreaCorner - pos2;
  const Vector2f pos3ToCenterCircle = -pos3;
  const Vector2f pos4ToCenterCircle = -pos4;
  const Vector2f pos5ToLeftCorner   = leftPenaltyAreaCorner - pos5;
  const Vector2f pos6ToLeftCorner   = leftPenaltyAreaCorner - pos6;
  const Angle angle1 = pos1ToRightCorner.angle();
  const Angle angle2 = pos2ToRightCorner.angle();
  const Angle angle3 = pos3ToCenterCircle.angle();
  const Angle angle4 = pos4ToCenterCircle.angle();
  const Angle angle5 = pos5ToLeftCorner.angle();
  const Angle angle6 = pos6ToLeftCorner.angle();
  walkInPositions.push_back(Pose2f(angle1, pos1)); // Robot 1
  walkInPositions.push_back(Pose2f(angle2, pos2)); // Robot 2
  walkInPositions.push_back(Pose2f(angle3, pos3)); // Robot 3
  walkInPositions.push_back(Pose2f(angle4, pos4)); // Robot 4
  walkInPositions.push_back(Pose2f(angle5, pos5)); // Robot 5
  walkInPositions.push_back(Pose2f(angle6, pos6)); // Robot 6
  if(mirrorWalkInPositionsFieldPlayers)
  {
    for(unsigned int i = 1; i < walkInPositions.size(); ++i)
    {
      walkInPositions[i] = Pose2f(Constants::pi) * walkInPositions[i];
    }
  }

  nextWalkInPoseNumber = 0;
  nextGPKCPoseNumber = 0;

  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFSample>(numberOfSamples);

  for(int i = 0; i < samples->size(); ++i)
    samples->at(i).init(getNewPoseAtWalkInPosition(), walkInPoseDeviation, nextSampleNumber++, 0.5f);
}

SelfLocator::~SelfLocator()
{
  delete samples;
}

void SelfLocator::update(RobotPose& robotPose)
{
#ifndef NDEBUG
  ASSERT(!std::isnan(theOdometer.distanceWalked));
  ASSERT(!std::isnan(static_cast<float>(theOdometer.odometryOffset.rotation)));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.x()));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.y()));
  ASSERT(!std::isnan(theOwnSideModel.largestXPossible));
  for(const FieldLines::Line& l : theFieldLines.lines)
  {
    ASSERT(!std::isnan(static_cast<float>(l.alpha)));
    ASSERT(!std::isnan(l.d));
    ASSERT(!std::isnan(l.first.x()));
    ASSERT(!std::isnan(l.last.y()));
  }
  if(theFrameInfo.getTimeSince(lastTimeCirclePerceptSeen) < 10 && lastTimeCirclePerceptSeen != 0)
  {
    ASSERT(!std::isnan(theCirclePercept.pos.x()));
    ASSERT(!std::isnan(theCirclePercept.pos.y()));
  }
  if(thePenaltyMarkPercept.wasSeen)
  {
    ASSERT(!std::isnan(thePenaltyMarkPercept.positionOnField.x()));
    ASSERT(!std::isnan(thePenaltyMarkPercept.positionOnField.y()));
  }
  if(theFieldBoundary.isValid)
  {
    for(const Vector2f& v : theFieldBoundary.boundaryOnField)
    {
      ASSERT(!std::isnan(v.x()));
      ASSERT(!std::isnan(v.y()));
    }
  }
#endif

  /* Initialize variable(s) */
  sampleSetHasBeenResetted = false;
  validitiesHaveBeenUpdated = false;
  inverseCameraMatrix = theCameraMatrix.inverse();
  currentRotationDeviation = theMotionInfo.isStanding() ? robotRotationDeviationInStand : robotRotationDeviation;
  if(thePenaltyMarkPercept.wasSeen)
    lastTimePenaltyMarkSeen = theFrameInfo.time;
  if(theCirclePercept.wasSeen)
    lastTimeCirclePerceptSeen = theFrameInfo.time;

  /* Modify sample set according to certain changes of the game state:
   *  - Reset mirror in SET
   *  - Handling of penalty positions
   *  - ...
   */
  handleGameStateChanges(theWorldModelPrediction.robotPose);

  /* Move all samples according to the current odometry.
   */
  STOPWATCH("SelfLocator:motionUpdate")
  {
    motionUpdate();
  }

  /* Integrate perceptions
   *  - goal posts, lines, corners ...
   * and compute validity of each sample
   */
  STOPWATCH("SelfLocator:sensorUpdate")
  {
    sensorUpdate();
  }

  /* Check and plot current weightings */
  float minWeighting = 2.f;
  float maxWeighting = -1.f;
  float weightingSum = 0.f;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    samples->at(i).computeWeightingBasedOnValidity(baseValidityWeighting);
    const float w = samples->at(i).weighting;
    weightingSum += w;
    if(w > maxWeighting)
      maxWeighting = w;
    if(w < minWeighting)
      minWeighting = w;
  }
  averageWeighting = weightingSum / numberOfSamples;
  PLOT("module:SelfLocator:minWeighting", minWeighting);
  PLOT("module:SelfLocator:maxWeighting", maxWeighting);
  PLOT("module:SelfLocator:averageWeighting", averageWeighting);

  /* Handle mirror information from SideConfidence */
  handleSideConfidence();

  /* Actually: Hacks and workaround :-)
   *  - Treat keeper rotation problem
   */
  domainSpecificSituationHandling();

  /* Particle filter resampling step
   *  - not executed during penalty shootout!
   *  - not executed for a few seconds after returning from a penalty
   *  - only executed if validities have changed this frame
   */
  if(validitiesHaveBeenUpdated &&
     theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) > 4000 &&
     theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
    resampling();

  /* Fill the RobotPose representation based on the current sample set
   */
  computeModel(robotPose);

  /* Replace a sample, if necessary
   *   This step is done at the end to make sure that the new sample
   *   gets the full motion and sensor update steps before being involved
   *   in any pose computations.
   */
  if(sensorResetting(robotPose))
  {
    PLOT("module:SelfLocator:sampleResetting", 6.f);
  }
  else
  {
    PLOT("module:SelfLocator:sampleResetting", 0.f);
  }

  /* Finally, update internal variables, debug and draw stuff.
   */
  DEBUG_RESPONSE("module:SelfLocator:templates_only")
  {
    if(theAlternativeRobotPoseHypothesis.isValid)
    {
      for(int i = 0; i < numberOfSamples; ++i)
      {
        UKFSample newSample;
        if(theOwnSideModel.stillInOwnSide)
          newSample.init(getNewPoseBasedOnObservations(true, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        else
          newSample.init(getNewPoseBasedOnObservations(false, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        samples->at(i) = newSample;
      }
    }
  }

  if((theWorldModelPrediction.robotPose.translation - robotPose.translation).norm() > positionJumpNotificationDistance &&
     theGameInfo.state == STATE_PLAYING && !theSideConfidence.mirror)
  {
    if(theFrameInfo.getTimeSince(lastTimeJumpSound) > 1337)
    {
      SystemCall::say("Jump");
      lastTimeJumpSound = theFrameInfo.time;
    }
    robotPose.timestampLastJump = theFrameInfo.time;
    ANNOTATION("SelfLocator", "Robot position has jumped!");
  }

  MODIFY("representation:RobotPose", robotPose);
  draw(robotPose);
  DEBUG_DRAWING("cognition:Odometry", "drawingOnField")
  {
    Pose2f origin = robotPose + theOdometryData.inverse();
    ORIGIN("cognition:Odometry", origin.translation.x(), origin.translation.y(), origin.rotation);
  }
}

void SelfLocator::update(SelfLocalizationHypotheses& selfLocalizationHypotheses)
{
  selfLocalizationHypotheses.hypotheses.resize(numberOfSamples);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    SelfLocalizationHypotheses::Hypothesis& h = selfLocalizationHypotheses.hypotheses[i];
    h.pose = samples->at(i).getPose();
    h.validity = samples->at(i).validity;
    Matrix3f cov = samples->at(i).getCov();
    h.xVariance = cov(0, 0);
    h.yVariance = cov(1, 1);
    h.xyCovariance = cov(1, 0);
    h.rotVariance = cov(2, 2);
  }
}

void SelfLocator::computeModel(RobotPose& robotPose)
{
  UKFSample& result = getMostValidSample();
  Pose2f resultPose = result.getPose();
  // Override side information for testing on one side of a field only
  if(alwaysAssumeOpponentHalf && resultPose.translation.x() < 0)
  {
    resultPose = Pose2f(pi) + resultPose;
  }
  robotPose = resultPose;
  Matrix3f cov = result.getCov();
  robotPose.deviation = sqrt(std::max(cov(0, 0), cov(1, 1)));
  robotPose.validity = result.validity;
  robotPose.covariance = cov;
  if(robotPose.validity >= validityThreshold)
    robotPose.validity = 1.f;
  else
    robotPose.validity *= (1.f / validityThreshold);
  if(theMidCircle.isValid || theMidCorner.isValid || theGoalFrame.isValid || theOuterCorner.isValid ||
     thePenaltyArea.isValid)
  {
    robotPose.timeOfLastConsideredFieldFeature = theFrameInfo.time;
  }
  if(theSideConfidence.mirror)
    robotPose.timestampLastJump = theFrameInfo.time;
  idOfLastBestSample = result.id;
}

void SelfLocator::motionUpdate()
{
  // This is a nasty workaround but should help us in cases of bad/slow assistant referees:
  // If the robot returns from a penalty and is not walking, any rotation (that might come from
  // the z-axis gyro as the robot is turned too late!) is ignored!
  float odometryRotation = theOdometer.odometryOffset.rotation;
  if(theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 10000 && theMotionInfo.motion != MotionRequest::walk)
    odometryRotation = 0.f;

  const float transX = theOdometer.odometryOffset.translation.x();
  const float transY = theOdometer.odometryOffset.translation.y();
  const float dist = theOdometer.odometryOffset.translation.norm();
  const float angle = abs(odometryRotation);
  const float angleWeightNoise = theMotionInfo.motion == MotionRequest::walk ? movedAngleWeightRotationNoise : movedAngleWeightRotationNoiseNotWalking;

  // Precalculate rotational error that has to be adapted to all samples
  float rotError = max(dist * movedDistWeightRotationNoise, angle * angleWeightNoise);

  // precalculate translational error that has to be adapted to all samples
  const float transXError = max(abs(transX * majorDirTransWeight), abs(transY * minorDirTransWeight));
  const float transYError = max(abs(transY * majorDirTransWeight), abs(transX * minorDirTransWeight));

  // update samples
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Vector2f transOffset((transX - transXError) + (2 * transXError) * Random::uniform(),
                               (transY - transYError) + (2 * transYError) * Random::uniform());
    const float rotationOffset = odometryRotation + Random::uniform(-rotError, rotError);

    samples->at(i).motionUpdate(Pose2f(rotationOffset, transOffset), filterProcessDeviation, odometryDeviation, odometryRotationDeviation);
  }
}

void SelfLocator::sensorUpdate()
{
  // Do not update during certain motions as the percepts might have a low quality
  if(!(theMotionRequest.motion == MotionRequest::walk || theMotionInfo.isStanding()))
    return;
  // In the penalty shootout, the goalkeeper should not perform any real localization
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theTeamBehaviorStatus.role.isGoalkeeper)
    return;

  // Integrate Landmarks and lines
  bool useLines = true;
  bool useLandmarks = true;
  bool usePoses = true;
  MODIFY("module:SelfLocator:useLines", useLines);
  MODIFY("module:SelfLocator:useLandmarks", useLandmarks);
  MODIFY("module:SelfLocator:usePoses", usePoses);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Pose2f samplePose = samples->at(i).getPose();
    perceptRegistration.update(samplePose, registeredPercepts, inverseCameraMatrix, currentRotationDeviation);
    if(usePoses)
      for(const auto& pose : registeredPercepts.poses)
        samples->at(i).updateByPose(pose, theCameraMatrix, inverseCameraMatrix, currentRotationDeviation, theFieldDimensions);
    if(useLandmarks)
      for(const auto& landmark : registeredPercepts.landmarks)
        samples->at(i).updateByLandmark(landmark);
    if(useLines)
      for(const auto& line : registeredPercepts.lines)
        samples->at(i).updateByLine(line);
    float numerator = 0.f;
    float denominator = 0.f;
    if(registeredPercepts.totalNumberOfPerceivedLines && useLines && considerLinesForValidityComputation)
    {
      numerator = static_cast<float>(registeredPercepts.lines.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedLines);
      denominator = 1.f;
    }
    if(registeredPercepts.totalNumberOfPerceivedLandmarks && useLandmarks)
    {
      numerator += 3.f * (static_cast<float>(registeredPercepts.landmarks.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedLandmarks));
      denominator += 3.f;
    }
    if(registeredPercepts.totalNumberOfPerceivedPoses && usePoses)
    {
      numerator += 5.f * (static_cast<float>(registeredPercepts.poses.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedPoses));
      denominator += 5.f;
    }
    if(denominator != 0.f)
    {
      const float currentValidity = numerator / denominator;
      samples->at(i).updateValidity(numberOfConsideredFramesForValidity, currentValidity);
      validitiesHaveBeenUpdated = true;
    }
  }

  // Apply OwnSideModel:
  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).getPose().translation.x() > theOwnSideModel.largestXPossible ||
         (theTeamBehaviorStatus.role.isGoalkeeper && samples->at(i).getPose().translation.x() > 0.f))
        samples->at(i).invalidate();
    }
  }

  // Check, if sample is still on the carpet
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Vector2f& position = samples->at(i).getPose().translation;
    if(!theFieldDimensions.isInsideCarpet(position))
      samples->at(i).invalidate();
  }
}

bool SelfLocator::sensorResetting(const RobotPose& robotPose)
{
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) // Don't replace samples in penalty shootout
    return false;
  if(theSideConfidence.mirror)                          // Don't replace samples in mirror cycle
    return false;
  if(timeOfLastReturnFromPenalty != 0 && theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 7000)
    return false;
  if(theFallDownState.state != FallDownState::upright
     && theFallDownState.state != FallDownState::squatting)  // Don't replace samples, if robot is not upright (e.g. lying or staggering)
    return false;
  if(sampleSetHasBeenResetted)
    return false;

  // Can we perform the resetting of a sample?
  if(theAlternativeRobotPoseHypothesis.isValid &&
     theAlternativeRobotPoseHypothesis.numOfContributingObservations >= minNumberOfObservationsForResetting &&
     theAlternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate > lastAlternativePoseTimestamp)
  {
    // Should we perform the resetting of a sample?
    bool alternativeIsCompatibleToRobotPose = alternativeRobotPoseIsCompatibleToPose(robotPose);
    if(theAlternativeRobotPoseHypothesis.isInOwnHalf)
    {
      if(alternativeIsCompatibleToRobotPose)
        return false; // Everything is OK, no reseting required
    }
    else // mirrored alternative has also to be considered
    {
      const Pose2f mirrorPose = Pose2f(pi) + robotPose;
      bool mirrorIsCompatibleToRobotPose = alternativeRobotPoseIsCompatibleToPose(mirrorPose);
      if(mirrorIsCompatibleToRobotPose || alternativeIsCompatibleToRobotPose)
        return false; // Everything is OK, no resetting required
    }
    // Resetting seems to be required:
    float resettingValidity = max(0.5f, averageWeighting); // TODO: Recompute?
    int worstSampleIdx = 0;
    float worstSampleValidity = samples->at(0).validity;
    for(int i = 1; i < numberOfSamples; ++i)
    {
      if(samples->at(i).validity < worstSampleValidity)
      {
        worstSampleIdx = i;
        worstSampleValidity = samples->at(i).validity;
      }
    }
    UKFSample newSample;
    if(theOwnSideModel.stillInOwnSide || theTeamBehaviorStatus.role.isGoalkeeper)
      newSample.init(getNewPoseBasedOnObservations(true, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
    else
      newSample.init(getNewPoseBasedOnObservations(false, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
    samples->at(worstSampleIdx) = newSample;
    lastAlternativePoseTimestamp = theAlternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate;
    return true;
  }
  return false;
}

bool SelfLocator::alternativeRobotPoseIsCompatibleToPose(const Pose2f& robotPose)
{
  const float xDev = std::abs(robotPose.translation.x() - theAlternativeRobotPoseHypothesis.pose.translation.x());
  const float yDev = std::abs(robotPose.translation.y() - theAlternativeRobotPoseHypothesis.pose.translation.y());
  const float rDev = std::abs(Angle::normalize(robotPose.rotation - theAlternativeRobotPoseHypothesis.pose.rotation));
  return xDev < translationalDeviationForResetting && yDev < translationalDeviationForResetting &&
         rDev < rotationalDeviationForResetting;
}

void SelfLocator::resampling()
{
  if(averageWeighting == 0.f)
    return;
  // actual resampling step:
  UKFSample* oldSet = samples->swap();
  const float weightingBetweenTwoDrawnSamples = averageWeighting;
  float nextPos(Random::uniform() * weightingBetweenTwoDrawnSamples);
  float currentSum(0);

  // resample:
  int replacements(0);
  int j(0);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    currentSum += oldSet[i].weighting;
    int replicationCount(0);
    while(currentSum > nextPos && j < numberOfSamples)
    {
      samples->at(j) = oldSet[i];
      if(replicationCount) // An old sample becomes copied multiple times: we need new identifier for the new instances
      {
        samples->at(j).id = nextSampleNumber++;
        replacements++;
      }
      replicationCount++;
      j++;
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }
  int missingSamples = numberOfSamples - j;
  // fill up missing samples (could happen in rare cases due to numerical imprecision / rounding / whatever) with new poses:
  for(; j < numberOfSamples; ++j)
  {
    if(theAlternativeRobotPoseHypothesis.isValid) // Try to use the currently best available alternative
    {
      const Pose2f pose = getNewPoseBasedOnObservations(false, theWorldModelPrediction.robotPose);
      samples->at(j).init(pose, defaultPoseDeviation, nextSampleNumber++, averageWeighting);
      ANNOTATION("SelfLocator", "Missing sample was replaced by alternative hypothesis! Current number of samples: " << j);
    }
    else if(j > 0) // if no alternative is available, just use the first sample
    {
      const Pose2f pose = samples->at(0).getPose();
      samples->at(j).init(pose, defaultPoseDeviation, nextSampleNumber++, averageWeighting);
      ANNOTATION("SelfLocator", "Missing sample was replaced by sample #0! Current number of samples: " << j);
    }
    else
    {
      ASSERT(false); // <- If we reach this line, we have a serious problem. Should not happen ;-)
    }
  }
  ASSERT(allSamplesIDsAreUnique());
  PLOT("module:SelfLocator:missingSamples", missingSamples);
  PLOT("module:SelfLocator:sampleReplacements", replacements);
}

void SelfLocator::handleSideConfidence()
{
  if(!theSideConfidence.mirror || sampleSetHasBeenResetted)
    return;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    samples->at(i).mirror();
  }
  ANNOTATION("SelfLocator", "Mirrrrrrooaaaarred!");
}

void SelfLocator::handleGameStateChanges(const Pose2f& robotPose)
{
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    // penalty shoot: if game state switched to playing reset samples to start position
    if((theCognitionStateChanges.lastGameState != STATE_PLAYING && theGameInfo.state == STATE_PLAYING) ||
       (theCognitionStateChanges.lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
    {
      for(int i = 0; i < samples->size(); ++i)
        samples->at(i).init(getNewPoseAtPenaltyShootoutPosition(), penaltyShootoutPoseDeviation, nextSampleNumber++, 1.f);
      sampleSetHasBeenResetted = true;
    }
  }
  // If the robot has been lifted during SET, reset samples to manual positioning line positions
  else if(theOwnSideModel.manuallyPlaced && theGameInfo.state == STATE_SET)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtManualPlacementPosition(), manualPlacementPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
  }
  // If a penalty is over, reset samples to reenter positions
  else if(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)
  {
    int startOfSecondHalfOfSampleSet = samples->size() / 2;
    // The first half of the new sample set is left of the own goal ...
    for(int i = 0; i < startOfSecondHalfOfSampleSet; ++i)
      samples->at(i).init(getNewPoseReturnFromPenaltyPosition(true), returnFromPenaltyPoseDeviation, nextSampleNumber++, 0.5f);
    // ... and the second half of new sample set is right of the own goal.
    for(int i = startOfSecondHalfOfSampleSet; i < samples->size(); ++i)
      samples->at(i).init(getNewPoseReturnFromPenaltyPosition(false), returnFromPenaltyPoseDeviation, nextSampleNumber++, 0.5f);
    sampleSetHasBeenResetted = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
  }
  // Normal game is about to start: We start on the sidelines looking at our goal: (this is for checking in TeamCom)
  else if(theCognitionStateChanges.lastGameState != STATE_INITIAL && theGameInfo.state == STATE_INITIAL)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtWalkInPosition(), walkInPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
  }
  // Normal game really starts: We start on the sidelines looking at our goal: (this is for actual setup)
  else if(theCognitionStateChanges.lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtWalkInPosition(), walkInPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
  }
  if(sampleSetHasBeenResetted)
  {
    idOfLastBestSample = -1;
  }
}

UKFSample& SelfLocator::getMostValidSample()
{
  float validityOfLastBestSample = -1.f;
  UKFSample* lastBestSample = 0;
  if(idOfLastBestSample != -1)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).id == idOfLastBestSample)
      {
        validityOfLastBestSample = samples->at(i).validity;
        lastBestSample = &(samples->at(i));
        break;
      }
    }
  }
  UKFSample* returnSample = &(samples->at(0));
  float maxValidity = -1.f;
  float minVariance = 0.f; // Initial value does not matter
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const float val = samples->at(i).validity;
    if(val > maxValidity)
    {
      maxValidity = val;
      minVariance = samples->at(i).getVarianceWeighting();
      returnSample = &(samples->at(i));
    }
    else if(val == maxValidity)
    {
      float variance = samples->at(i).getVarianceWeighting();
      if(variance < minVariance)
      {
        maxValidity = val;
        minVariance = variance;
        returnSample = &(samples->at(i));
      }
    }
  }
  if(lastBestSample && returnSample->validity <= validityOfLastBestSample * 1.1) // Bonus for stability
    return *lastBestSample;
  else
    return *returnSample;
}

void SelfLocator::domainSpecificSituationHandling()
{
  // Keep track of the time during which the robot was in "normal" and stable operation, i.e. not jumping or getting up or stuff like that
  if(!theMotionInfo.isStanding() && theMotionInfo.motion != MotionInfo::walk && theMotionInfo.motion != MotionInfo::kick)
    lastTimeNotInStandWalkKick = theFrameInfo.time;

  // Calculate distance to furthest point on field boundary
  float maxDistance = 0.f;
  for(const Vector2f& p : theFieldBoundary.boundaryOnField)
  {
    if(p.norm() > maxDistance)
      maxDistance = p.norm();
  }
  if(maxDistance > goalieFieldBorderDistanceThreshold)
    lastTimeFarFieldBorderSeen = theFrameInfo.time;

  // Currently, this method only handles the case that the
  // keeper is turned by 180 degrees but assumes to stand correctly
  if(!activateGoalieTwistHandling ||
     !theTeamBehaviorStatus.role.isGoalkeeper ||
     theGameInfo.state != STATE_PLAYING ||
     theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    return;
  // The robot is in its penalty area and assumes to look at the opponent half
  // and guards its goal.
  if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea + 200 &&
     abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea &&
     abs(theRobotPose.rotation) < 45_deg &&
     theTeamBehaviorStatus.role.isGoalkeeper &&
     theFrameInfo.getTimeSince(lastTimeNotInStandWalkKick) > goalieTwistNoStandWalkKickTimeout &&
     theFieldBoundary.isValid)
  {
    // It has not seen important stuff for a long time but the field border appears
    // to be close. This means that it is probably twisted:
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > goalieNoPerceptsThreshold &&
       lastTimeCirclePerceptSeen != 0 && lastTimePenaltyMarkSeen != 0 &&
       theFrameInfo.getTimeSince(lastTimeCirclePerceptSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(lastTimePenaltyMarkSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(theFieldFeatureOverview.combinedStatus.lastSeen) > goalieNoPerceptsThreshold &&
       lastTimeFarFieldBorderSeen != 0 &&
       theFrameInfo.getTimeSince(lastTimeFarFieldBorderSeen) > goalieNoFarFieldBorderThreshold)
    {
      ANNOTATION("SelfLocator", "Goalie Twist!");
      for(int i = 0; i < numberOfSamples; ++i)
        samples->at(i).twist();
    }
  }
}

void SelfLocator::draw(const RobotPose& robotPose)
{
  // Visualizes effect of current function's parameters
  DEBUG_DRAWING("module:SelfLocator:isMirrorCloser", "drawingOnField")
  {
    const float stepSize = 500.f;
    const float degStepSize = 30.f;
    const float length = stepSize / 3.f;
    for(float x = theFieldDimensions.xPosOwnFieldBorder; x <= theFieldDimensions.xPosOpponentFieldBorder; x += stepSize)
    {
      for(float y = theFieldDimensions.yPosRightFieldBorder; y <= theFieldDimensions.yPosLeftFieldBorder; y += stepSize)
      {
        for(float rot = 0.f; rot < 360.f; rot += degStepSize)
        {
          Pose2f samplePose(Angle::fromDegrees(rot), x, y);
          ColorRGBA col = isMirrorCloser(samplePose, robotPose) ? ColorRGBA(255, 0, 0) : ColorRGBA(0, 0, 0);
          Vector2f vec(length, 0.f);
          vec.rotate(Angle::fromDegrees(rot));
          LINE("module:SelfLocator:isMirrorCloser", x, y, x + vec.x(), y + vec.y(), 20, Drawings::solidPen, col);
        }
      }
    }
  }
  // Visualize hypothetical percept registration for the current pose
  DEBUG_DRAWING("module:SelfLocator:perceptRegistrationRobot", "drawingOnField")
  {
    THREAD("module:SelfLocator:perceptRegistrationRobot", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
  }
  DEBUG_DRAWING("module:SelfLocator:perceptRegistrationWorld", "drawingOnField")
  {
    THREAD("module:SelfLocator:perceptRegistrationWorld", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
    perceptRegistration.update(robotPose, registeredPercepts, inverseCameraMatrix, currentRotationDeviation, true);
  }
}

bool SelfLocator::allSamplesIDsAreUnique()
{
  for(int i = 0; i < numberOfSamples - 1; ++i)
  {
    for(int j = i + 1; j < numberOfSamples; ++j)
    {
      if(samples->at(i).id == samples->at(j).id)
        return false;
    }
  }
  return true;
}

Pose2f SelfLocator::getNewPoseBasedOnObservations(bool forceOwnHalf, const Pose2f& lastRobotPose) const
{
  const Pose2f& p = theAlternativeRobotPoseHypothesis.pose;
  if(forceOwnHalf)
  {
    if(p.translation.x() <= 0.f)
      return p;
    else
      return Pose2f(pi) + p;
  }
  else
  {
    if(isMirrorCloser(p, lastRobotPose))
      return Pose2f(pi) + p;
    else
      return p;
  }
}

Pose2f SelfLocator::getNewPoseReturnFromPenaltyPosition(bool leftSideOfGoal)
{
  // Special stuff for some demos:
  if(useCustomReturnFromPenaltyPoses)
  {
    if(theTeamBehaviorStatus.role.isGoalkeeper)
      return customReturnFromPenaltyPoseGoalie;
    else
      return customReturnFromPenaltyPoseFieldPlayer;
  }
  // Testing purposes: Static start position
  if(theStaticInitialPose.isActive && (theCognitionStateChanges.lastPenalty == PENALTY_MANUAL || theCognitionStateChanges.lastPenalty == PENALTY_SPL_PLAYER_PUSHING))
  {
    return theStaticInitialPose.staticPoseOnField;
  }
  // Normal stuff:
  float xPosition = Random::triangular(theFieldDimensions.xPosOwnPenaltyMark - returnFromPenaltyMaxXOffset,
                                       theFieldDimensions.xPosOwnPenaltyMark,
                                       theFieldDimensions.xPosOwnPenaltyMark + returnFromPenaltyMaxXOffset);
  if(leftSideOfGoal)
    return Pose2f(-pi_2, xPosition, theFieldDimensions.yPosLeftSideline);
  else
    return Pose2f(pi_2, xPosition, theFieldDimensions.yPosRightSideline);
}

Pose2f SelfLocator::getNewPoseAtWalkInPosition()
{
  const int effectivePlayerNumber = theGameInfo.competitionType != COMPETITION_TYPE_MIXEDTEAM ? theOwnTeamInfo.getSubstitutedPlayerNumber(theRobotInfo.number) : theRobotInfo.number;
  if(effectivePlayerNumber >= 1 && effectivePlayerNumber <= 6)
  {
    const unsigned index = effectivePlayerNumber - 1;
    return walkInPositions[index];
  }
  else
  {
    Pose2f result = walkInPositions[nextWalkInPoseNumber];
    nextWalkInPoseNumber = (nextWalkInPoseNumber + 1) % walkInPositions.size();
    return result;
  }
}

Pose2f SelfLocator::getNewPoseAtManualPlacementPosition()
{
  // Goalie
  if(theTeamBehaviorStatus.role.isGoalkeeper)
  {
    return Pose2f(0.f, theFieldDimensions.xPosOwnGroundline + 52.f, 0.f);
  }
  else
  {
    float x = theFieldDimensions.xPosOwnPenaltyArea + 100.f;
    float minY = theFieldDimensions.yPosRightSideline + 750.f;
    float y = minY + Random::uniform(2 * std::abs(minY));
    return Pose2f(0.f, x, y);
  }
}

Pose2f SelfLocator::getNewPoseAtPenaltyShootoutPosition()
{
  if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
  {
    //striker pose (1 meter behind the penalty spot, looking towards opponent goal)
    return Pose2f(0.f, theFieldDimensions.xPosPenaltyStrikerStartPosition, 0.f);
  }
  else
  {
    //goalie pose (in the center of the goal, looking towards the field's center)
    return Pose2f(0.f, theFieldDimensions.xPosOwnGroundline, 0.f);
  }
}

bool SelfLocator::isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const
{
  const Vector2f& translation = currentPose.translation;
  Vector2f rotationWeight(std::max(theFieldDimensions.yPosLeftSideline * 1.1f - std::min(std::abs(translation.x()), std::abs(lastPose.translation.x())), 0.f), 0);
  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoalPost, 0.f);
  const Vector2f rotation = Pose2f(Geometry::angleTo(currentPose, opponentGoal)) * rotationWeight;
  const Vector2f lastRotation = Pose2f(Geometry::angleTo(lastPose, opponentGoal)) * rotationWeight;
  bool result = (lastPose.translation - translation).norm() + (lastRotation - rotation).norm() >
                (lastPose.translation + translation).norm() + (lastRotation + rotation).norm();
  return result;
}

MAKE_MODULE(SelfLocator, modeling)
