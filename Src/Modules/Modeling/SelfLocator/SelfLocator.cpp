/**
 * @file SelfLocator.cpp
 *
 * Implements a class that performs self-localization
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SelfLocator.h"
#include "Debugging/Annotation.h"
#include "Debugging/Plot.h"
#include "Framework/Settings.h"
#include "Math/Eigen.h"
#include "Math/Probabilistics.h"
#include "Platform/SystemCall.h"
#include <algorithm>

SelfLocator::SelfLocator() : lastTimeJumpSound(0),
  timeOfLastReturnFromPenalty(0),
  nextSampleNumber(0), idOfLastBestSample(-1), averageWeighting(0.5f), lastAlternativePoseTimestamp(0),
  validitiesHaveBeenUpdated(false)
{
  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFRobotPoseHypothesis>(numberOfSamples);
  for(int i = 0; i < samples->size(); ++i)
    samples->at(i).init(getNewPoseAtWalkInPosition(), walkInPoseDeviation, nextSampleNumber++, 0.5f);
  lastGroundTruthRobotPose = theGroundTruthRobotPose;

  // Initialize statistics:
  sumOfPerceivedLandmarks = sumOfPerceivedLines = 0;
  sumOfUsedLines = sumOfUsedLandmarks = 0.f;
}

SelfLocator::~SelfLocator()
{
  delete samples;
}

void SelfLocator::update(RobotPose& robotPose)
{
  /* Initialize variable(s) */
  sampleSetHasBeenReset = false;
  validitiesHaveBeenUpdated = false;

  /* Modify sample set according to certain changes of the game state:
   *  - Reset mirror in SET
   *  - Handling of penalty positions
   *  - ...
   */
  handleGameStateChanges();

  DEBUG_RESPONSE("module:SelfLocator:activateSampleResettingToGroundTruth")
    resetSamplesToGroundTruth();
  // In any case, remember last ground truth robot pose
  lastGroundTruthRobotPose = theGroundTruthRobotPose;

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

  /* Add hacks and workarounds here. ***************
   * However, always remember: "Ich sage Ihnen, gleich richtig machen – das bringt's immer und überall, und das bringt einen auch überall hin." (c) Katz & Goldt
   */

  /* Particle filter resampling step
   *  - not executed during penalty shootout!
   *  - not executed for a few seconds after returning from a penalty
   *  - only executed if validities have changed this frame
   */
  if(validitiesHaveBeenUpdated &&
     theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) > timeoutForResamplingAfterReturnFromPenalty &&
     !theGameState.isPenaltyShootout())
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

  /** In Initial, the setup pose is directly set to avoid jumping.
   */
  if(theGameState.gameControllerActive &&
     ((theGameState.isInitial() && !theGameState.isPenaltyShootout()) ||
     (theGameState.isSet() && !theGameState.isPenaltyKick() && Global::getSettings().scenario.starts_with("SharedAutonomy"))))
    robotPose = getNewPoseAtWalkInPosition();

  /* Finally, update internal variables, debug and draw stuff.
   */
  DEBUG_RESPONSE("module:SelfLocator:templates_only")
  {
    if(theAlternativeRobotPoseHypothesis.isValid)
    {
      for(int i = 0; i < numberOfSamples; ++i)
      {
        UKFRobotPoseHypothesis newSample;
        if(theSideInformation.robotMustBeInOwnHalf)
          newSample.init(getNewPoseBasedOnObservations(true, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        else
          newSample.init(getNewPoseBasedOnObservations(false, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        samples->at(i) = newSample;
      }
    }
  }

  /* Some statistics */
  DEBUG_RESPONSE_ONCE("module:SelfLocator:output_statistics")
  {
    const int percentLandmarks = static_cast<int>(sumOfUsedLandmarks / sumOfPerceivedLandmarks * 100.f);
    const int percentLines = static_cast<int>(sumOfUsedLines / sumOfPerceivedLines * 100.f);
    OUTPUT_TEXT("Landmarks perceived: " << sumOfPerceivedLandmarks << "  used: " << sumOfUsedLandmarks << " = " << percentLandmarks << "%");
    OUTPUT_TEXT("Lines perceived: " << sumOfPerceivedLines << "  used: " << sumOfUsedLines << " = " << percentLines << "%");
  }

  // Check, if there is a huge difference between the current and the previous robot pose:
  const float robotPoseDelta = (theWorldModelPrediction.robotPose.translation - robotPose.translation).norm();
  if(robotPoseDelta > positionJumpNotificationDistance && theGameState.isPlaying())
  {
    // Play annoying sound
    if(theFrameInfo.getTimeSince(lastTimeJumpSound) > timeoutBetweenTwoJumpSounds)
    {
      SystemCall::say("Jump");
      lastTimeJumpSound = theFrameInfo.time;
    }
    // Annotate jump depending on current situation:
    // A jump when returning from a penalty is intended, thus we do not annotate it (in most cases) for some seconds.
    if(theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < timeoutForJumpAnnotationAfterReturnFromPenalty)
    {
      // This indicates that the robot was unsure about the side at which it was placed.
      // This is not so cool and should be annotated:
      if(timeOfLastReturnFromPenalty < robotPose.timestampLastJump)
      {
        ANNOTATION("SelfLocator", "Robot was unsure after return from penalty!");
      }
    }
    // Standard case:
    else
    {
      ANNOTATION("SelfLocator", "Robot position has jumped over " << static_cast<int>(robotPoseDelta) << " mm!");
    }
    robotPose.timestampLastJump = theFrameInfo.time;
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
  UKFRobotPoseHypothesis& bestSample = getMostValidSample();
  Pose2f resultPose = bestSample.getPose();
  // Override side information for testing in the opponent half of a field only
  if(theSideInformation.robotMustBeInOpponentHalf && resultPose.translation.x() < 0) // TL: This appears a bit too simple. TODO: Make better.
  {
    resultPose = Pose2f(pi) + resultPose;
  }
  robotPose = resultPose;
  Matrix3f cov = bestSample.getCov();
  robotPose.covariance = cov;
  idOfLastBestSample = bestSample.id;
  // Finally, set the quality information:
  float validityOfBestHypothesis = bestSample.validity;
  setLocalizationQuality(robotPose, validityOfBestHypothesis);
}

void SelfLocator::setLocalizationQuality(RobotPose& robotPose, float validityOfBestHypothesis)
{
  // Overall, the probability distribution seems to be OK. Quality depends on deviation and validity:
  if(sampleSetIsUnimodal(robotPose))
  {
    const float translationalStandardDeviation = robotPose.getTranslationalStandardDeviation();
    const float rotationalStandardDeviation = std::sqrt(robotPose.covariance(2, 2));
    if(validityOfBestHypothesis >= minValidityForSuperbLocalizationQuality &&
       translationalStandardDeviation < maxTranslationDeviationForSuperbLocalizationQuality &&
       rotationalStandardDeviation < maxRotationalDeviationForSuperbLocalizationQuality)
      robotPose.quality = RobotPose::superb;
    else
      robotPose.quality = RobotPose::okay;
  }
  // Meow, there are samples at different places (e.g. after returning from a penalty or
  // after some sensor resetting of samples (as the robot appeared to have a bad localization)
  else
  {
    robotPose.quality = RobotPose::poor;
  }
}

bool SelfLocator::sampleSetIsUnimodal(const RobotPose& robotPose)
{
  // This is just some kind of temporary hack.
  // However, I have discovered a truly convincing solution for this problem which this comment is too small to contain. TL
  const float maxDistanceDeviation = 2019.f;
  const Angle maxRotationDeviation(84_deg);
  const Angle robotPoseRotation(robotPose.rotation);
  const float sqrMaxDistanceDeviation = maxDistanceDeviation * maxDistanceDeviation;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Pose2f& p = samples->at(i).getPose();
    if((robotPose.translation - p.translation).squaredNorm() > sqrMaxDistanceDeviation)
      return false;
    if(robotPoseRotation.diffAbs(Angle(p.rotation)) > maxRotationDeviation)
      return false;;
  }
  return true;
}

void SelfLocator::motionUpdate()
{
  // This is a nasty workaround but should help us in cases of bad/slow assistant referees:
  // If the robot returns from a penalty and is not walking, any rotation (that might come from
  // the z-axis gyro as the robot is turned too late!) is ignored!
  // As of 2024, this workaround should not be necessary anymore as the updated rules require the assistant
  // referees to place the robots "facing towards the field of play" for the whole time.
  // TODO: Observe, if this always happens correctly in 2024. If yes, remove this workaround.
  float odometryRotation = theOdometer.odometryOffset.rotation;
  if(theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 10000 && theMotionInfo.executedPhase != MotionPhase::walk)
    odometryRotation = 0.f;

  const float transX = theOdometer.odometryOffset.translation.x();
  const float transY = theOdometer.odometryOffset.translation.y();
  const float dist = theOdometer.odometryOffset.translation.norm();
  const float angle = std::abs(odometryRotation);
  const float angleWeightNoise = theMotionInfo.executedPhase == MotionPhase::walk ? movedAngleWeightRotationNoise : movedAngleWeightRotationNoiseNotWalking;

  // Precalculate rotational error that has to be adapted to all samples
  const float rotError = std::max(dist * movedDistWeightRotationNoise, angle * angleWeightNoise);

  // pre-calculate translational error that has to be adapted to all samples
  const float transXError = std::max(std::abs(transX * majorDirTransWeight), std::abs(transY * minorDirTransWeight));
  const float transYError = std::max(std::abs(transY * majorDirTransWeight), std::abs(transX * minorDirTransWeight));

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
  bool useLines = true;
  bool useLandmarks = true;
  bool usePoses = true;
  MODIFY("module:SelfLocator:useLines", useLines);
  MODIFY("module:SelfLocator:useLandmarks", useLandmarks);
  MODIFY("module:SelfLocator:usePoses", usePoses);

  // Do not update during certain motions as the percepts might have a low quality
  if(currentMotionIsUnsafe())
    return;
  // In the penalty shootout, the goalkeeper should not perform any real localization
  if(theGameState.isPenaltyShootout() && theGameState.isForOpponentTeam())
    return;

  // Perform integration of measurements:
  unsigned int usedLines = 0;
  unsigned int usedLandmarks = 0;
  std::vector<RegisteredAbsolutePoseMeasurement> absolutePoseMeasurements;
  std::vector<RegisteredLandmark> landmarks;
  std::vector<RegisteredLine> lines;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    float numerator = 0.f;
    float denominator = 0.f;
    const Pose2f samplePose = samples->at(i).getPose();
    if(usePoses && thePerceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements > 0)
    {
      thePerceptRegistration.registerAbsolutePoseMeasurements(samplePose, absolutePoseMeasurements);
      for(const auto& measuredPose : absolutePoseMeasurements)
        samples->at(i).updateByPose(measuredPose);
      numerator += validityFactorPoseMeasurement * (static_cast<float>(absolutePoseMeasurements.size()) / thePerceptRegistration.totalNumberOfAvailableAbsolutePoseMeasurements);
      denominator += validityFactorPoseMeasurement;
    }
    if(useLandmarks && thePerceptRegistration.totalNumberOfAvailableLandmarks > 0)
    {
      thePerceptRegistration.registerLandmarks(samplePose, landmarks);
      usedLandmarks += static_cast<unsigned int>(landmarks.size());
      for(const auto& landmark : landmarks)
        samples->at(i).updateByLandmark(landmark);
      numerator += validityFactorLandmarkMeasurement * (static_cast<float>(landmarks.size()) / thePerceptRegistration.totalNumberOfAvailableLandmarks);
      denominator += validityFactorLandmarkMeasurement;
    }
    if(useLines && thePerceptRegistration.totalNumberOfAvailableLines > 0)
    {
      thePerceptRegistration.registerLines(samplePose, lines);
      usedLines += static_cast<unsigned int>(lines.size());
      for(const auto& line : lines)
      {
        if(line.partOfCenterCircle) // This is not a classic line and is thus treated as a different kind of measurement
          samples->at(i).updateByLineOnCenterCircle(line, theFieldDimensions.centerCircleRadius);
        else // Normal line
          samples->at(i).updateByLine(line);
      }
      if(considerLinesForValidityComputation)
      {
        int numberOfLinesForValidityComputation = thePerceptRegistration.totalNumberOfAvailableLines - thePerceptRegistration.totalNumberOfIgnoredLines;
        if(numberOfLinesForValidityComputation > 0)
        {
          numerator += validityFactorLineMeasurement * static_cast<float>(lines.size()) / numberOfLinesForValidityComputation;
          denominator += validityFactorLineMeasurement;
        }
      }
    }
    // Update validities, if any features have been observed (no matter, if they have actually been used):
    if(denominator != 0.f)
    {
      const float currentValidity = numerator / denominator;
      samples->at(i).updateValidity(numberOfConsideredFramesForValidity, currentValidity);
      validitiesHaveBeenUpdated = true;
    }
  }

  // Apply side information:
  if(!theGameState.isPenaltyShootout())
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).getPose().translation.x() > theSideInformation.largestXCoordinatePossible)
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

  // Statistics
  sumOfPerceivedLines += thePerceptRegistration.totalNumberOfAvailableLines;
  sumOfPerceivedLandmarks += thePerceptRegistration.totalNumberOfAvailableLandmarks;
  sumOfUsedLines += static_cast<float>(usedLines) / numberOfSamples;
  sumOfUsedLandmarks += static_cast<float>(usedLandmarks) / numberOfSamples;
}

bool SelfLocator::currentMotionIsUnsafe()
{
  // Walking and standing is OK. Everything else (kicking, falling, getting up, ...) probably not.
  if(!theMotionInfo.isMotion(bit(MotionPhase::walk) | bit(MotionPhase::stand)))
    return true;
  // If the robot turns too fast, things go wrong.
  // TODO: Move hardcoded number to parameter. Values is guessed by Philip.
  if(std::abs(theIMUValueState.gyroValues.mean.z()) > 50_deg)
    return true;
  return false;
}

bool SelfLocator::sensorResetting(const RobotPose& robotPose)
{
  if(theGameState.isPenaltyShootout())                       // Don't replace samples in penalty shootout
    return false;
  if(timeOfLastReturnFromPenalty != 0 && theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < timeoutForSensorResettingAfterReturnFromPenalty)
    return false;
  if(theFallDownState.state != FallDownState::upright
     && theFallDownState.state != FallDownState::squatting)  // Don't replace samples, if robot is not upright (e.g. lying or staggering)
    return false;
  if(sampleSetHasBeenReset)
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
        return false; // Everything is OK, no resetting required
    }
    else // mirrored alternative has also to be considered
    {
      const Pose2f mirrorPose = Pose2f(pi) + robotPose;
      bool mirrorIsCompatibleToRobotPose = alternativeRobotPoseIsCompatibleToPose(mirrorPose);
      if(mirrorIsCompatibleToRobotPose || alternativeIsCompatibleToRobotPose)
        return false; // Everything is OK, no resetting required
    }
    // Resetting seems to be required:
    float resettingValidity = std::max(0.5f, averageWeighting); // TODO: Recompute?
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
    UKFRobotPoseHypothesis newSample;
    if(theSideInformation.robotMustBeInOwnHalf)
      newSample.init(getNewPoseBasedOnObservations(true, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
    else
      newSample.init(getNewPoseBasedOnObservations(false, theWorldModelPrediction.robotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
    samples->at(worstSampleIdx) = newSample;
    lastAlternativePoseTimestamp = theAlternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate;
    return true;
  }
  return false;
}

void SelfLocator::resetSamplesToGroundTruth()
{
  if(theGroundTruthRobotPose.timestamp != theFrameInfo.time)
    return;
  const float distanceDeviation = std::abs((theGroundTruthRobotPose.translation - lastGroundTruthRobotPose.translation).norm());
  const float rotationDeviation = std::abs(Angle::normalize(theGroundTruthRobotPose.rotation - lastGroundTruthRobotPose.rotation));
  // Numbers are guessed and hardcoded, but this is a debugging function anyway ...
  if(distanceDeviation > 300.f || rotationDeviation > Angle::fromDegrees(30.f))
  {
    for(int i = 0; i < samples->size(); ++i)
      samples->at(i).init(theGroundTruthRobotPose, penaltyShootoutPoseDeviation, nextSampleNumber++, 0.9f);
    sampleSetHasBeenReset = true;
    idOfLastBestSample = -1;
  }
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
  UKFRobotPoseHypothesis* oldSet = samples->swap();
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

void SelfLocator::handleGameStateChanges()
{
  if(theGameState.isPenaltyShootout())
  {
    // penalty shoot: if game state switched to playing reset samples to start position
    if((theGameState.isPlaying() && !theExtendedGameState.wasPlaying()) ||
       (!theGameState.isPenalized() && theExtendedGameState.wasPenalized()))
    {
      for(int i = 0; i < samples->size(); ++i)
        samples->at(i).init(getNewPoseAtPenaltyShootoutPosition(), penaltyShootoutPoseDeviation, nextSampleNumber++, 1.f);
      sampleSetHasBeenReset = true;
    }
  }
  // If the robot has been lifted during SET in a 1v1 demo (and not just been unpenalized), reset samples to custom positions
  else if(theLibDemo.isOneVsOneDemoActive && theGameState.isSet() &&
          theGameState.playerState == GameState::active && theFrameInfo.getTimeSince(theExtendedGameState.timeWhenPlayerStateStarted[GameState::active]) > 5000 &&
          theFallDownState.state == FallDownState::pickedUp)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtManualPlacementPosition(), manualPlacementPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenReset = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
  }
  // If a penalty is over, reset samples to reenter positions
  else if(theExtendedGameState.returnFromGameControllerPenalty || theExtendedGameState.returnFromManualPenalty ||
          (theGameState.playerState == GameState::calibration && theGameState.playerState != theExtendedGameState.playerStateLastFrame))
  {
    int startOfSecondHalfOfSampleSet = samples->size() / 2;
    // The first half of the new sample set is left of the own goal ...
    for(int i = 0; i < startOfSecondHalfOfSampleSet; ++i)
      samples->at(i).init(getNewPoseReturnFromPenaltyPosition(true), returnFromPenaltyPoseDeviation, nextSampleNumber++, 0.5f);
    // ... and the second half of new sample set is right of the own goal.
    for(int i = startOfSecondHalfOfSampleSet; i < samples->size(); ++i)
      samples->at(i).init(getNewPoseReturnFromPenaltyPosition(false), returnFromPenaltyPoseDeviation, nextSampleNumber++, 0.5f);
    sampleSetHasBeenReset = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
  }
  // Normal game is about to start: We start on the touchlines looking at our goal: (this is for checking in TeamCom)
  else if((theGameState.isInitial() && !theExtendedGameState.wasInitial()) ||
          // Normal game really starts: We start on the touchlines looking at our goal: (this is for actual setup)
          (theGameState.isReady() && theExtendedGameState.wasInitial()) ||
          (theGameState.isPlaying() && theExtendedGameState.wasSet() && !theGameState.isPenaltyKick() && Global::getSettings().scenario.starts_with("SharedAutonomy")))
  {
    for(int i = 0; i < samples->size(); ++i)
      samples->at(i).init(getNewPoseAtWalkInPosition(), walkInPoseDeviation, nextSampleNumber++, 0.5f);
    sampleSetHasBeenReset = true;
  }
  /* For testing purposes in simulator */
  else if(theStaticInitialPose.isActive && theStaticInitialPose.jump)
  {
    for(int i = 0; i < samples->size(); ++i)
      samples->at(i).init(theStaticInitialPose.staticPoseOnField, manualPlacementPoseDeviation, nextSampleNumber++, 0.5f);
    sampleSetHasBeenReset = true;
  }
  if(sampleSetHasBeenReset)
  {
    idOfLastBestSample = -1;
  }
}

UKFRobotPoseHypothesis& SelfLocator::getMostValidSample()
{
  float validityOfLastBestSample = -1.f;
  UKFRobotPoseHypothesis* lastBestSample = 0;
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
  UKFRobotPoseHypothesis* returnSample = &(samples->at(0));
  float maxValidity = -1.f;
  float minVariance = 0.f; // Initial value does not matter
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const float val = samples->at(i).validity;
    if(val > maxValidity)
    {
      maxValidity = val;
      minVariance = samples->at(i).getCombinedVariance();
      returnSample = &(samples->at(i));
    }
    else if(val == maxValidity)
    {
      float variance = samples->at(i).getCombinedVariance();
      if(variance < minVariance)
      {
        maxValidity = val;
        minVariance = variance;
        returnSample = &(samples->at(i));
      }
    }
  }
  if(lastBestSample && returnSample->validity <= validityOfLastBestSample * 1.1f) // Bonus for stability
    return *lastBestSample;
  else
    return *returnSample;
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
  std::vector<RegisteredAbsolutePoseMeasurement> absolutePoseMeasurements;
  std::vector<RegisteredLandmark> landmarks;
  std::vector<RegisteredLine> lines;
  thePerceptRegistration.registerAbsolutePoseMeasurements(robotPose, absolutePoseMeasurements);
  thePerceptRegistration.registerLandmarks(robotPose, landmarks);
  thePerceptRegistration.registerLines(robotPose, lines);

  DEBUG_DRAWING("module:SelfLocator:perceptRegistrationWorld", "drawingOnField")
  {
    THREAD("module:SelfLocator:perceptRegistrationWorld", theCameraInfo.getThreadName());
    for(const auto& line : lines)
    {
      const ColorRGBA col = line.partOfCenterCircle ? ColorRGBA(128, 255, 0, 180) : ColorRGBA(128, 0, 255, 180);
      // Associated model line in absolute field coordinates:
      LINE("module:SelfLocator:perceptRegistrationWorld", line.modelStart.x(), line.modelStart.y(), line.modelEnd.x(), line.modelEnd.y(),
           80, Drawings::solidPen, col);
    }
    for(const auto& landmark : landmarks)
    {
      // Associated landmark position in absolute field coordinates:
      CIRCLE("module:SelfLocator:perceptRegistrationWorld", landmark.model.x(), landmark.model.y(),
             200, 20, Drawings::noPen, ColorRGBA::blue, Drawings::solidPen, ColorRGBA::blue);
    }
    for(const auto& absolutePose : absolutePoseMeasurements)
    {
      // Covariance of the pose measurement
      ColorRGBA c99(255, 255, 100, 100);
      ColorRGBA c95(255, 128, 50, 100);
      ColorRGBA c68(255, 100, 100, 100);
      Matrix2f covMatrix2D = absolutePose.covariance.topLeftCorner(2, 2);
      covMatrix2D(0, 1) = covMatrix2D(1, 0) = 0.1f; // Set to small value as drawing does not always work properly, if matrix is axis-aligned ...

      COVARIANCE_ELLIPSES_2D_OWN_COLORS("module:SelfLocator:perceptRegistrationWorld",
                                        covMatrix2D, absolutePose.absolutePoseOnField.translation, c99, c95, c68);
      // Depict rotational covariance as an arc of the standard deviation
      float stdDev = std::sqrt(absolutePose.covariance(2, 2));
      ARC("module:SelfLocator:perceptRegistrationWorld", absolutePose.absolutePoseOnField.translation.x(), absolutePose.absolutePoseOnField.translation.y(),
          200.f, (-(stdDev) / 2.f + absolutePose.absolutePoseOnField.rotation), stdDev, 20, Drawings::solidPen, ColorRGBA::white, Drawings::noBrush, ColorRGBA::white);
    }
  }
  DEBUG_DRAWING("module:SelfLocator:perceptRegistrationRobot", "drawingOnField")
  {
    THREAD("module:SelfLocator:perceptRegistrationRobot", theCameraInfo.getThreadName());
    for(const auto& line : lines)
    {
      // Covariance of center of field line (has to be drawn relative to the robot)
      COVARIANCE_ELLIPSES_2D("module:SelfLocator:perceptRegistrationRobot", line.covPerceptCenter, line.perceptCenter);
    }
    ColorRGBA c99(255, 255, 100, 100);
    ColorRGBA c95(255, 128, 50, 100);
    ColorRGBA c68(255, 100, 100, 100);
    for(const auto& landmark : landmarks)
    {
      // Drawing of covariance (has to be drawn relative to the robot)
      COVARIANCE_ELLIPSES_2D_OWN_COLORS("module:SelfLocator:perceptRegistrationRobot", landmark.covPercept, landmark.percept, c99, c95, c68);
    }
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
  if(demoUseCustomReturnFromPenaltyPoses)
  {
    if(theGameState.isGoalkeeper())
      return demoCustomReturnFromPenaltyPoseGoalie;
    else
      return demoCustomReturnFromPenaltyPoseFieldPlayer;
  }
  // Testing purposes: Static start position
  if(theStaticInitialPose.isActive && (theExtendedGameState.playerStateLastFrame == GameState::penalizedPlayerPushing || theExtendedGameState.playerStateLastFrame == GameState::penalizedManual))
  {
    return theStaticInitialPose.staticPoseOnField;
  }
  // Normal stuff:
  float xPosition = Random::triangular(theFieldDimensions.xPosReturnFromPenalty - returnFromPenaltyMaxXOffset,
                                       theFieldDimensions.xPosReturnFromPenalty,
                                       theFieldDimensions.xPosReturnFromPenalty + returnFromPenaltyMaxXOffset);
  if(leftSideOfGoal)
    return Pose2f(-pi_2, xPosition, theFieldDimensions.yPosLeftReturnFromPenalty);
  else
    return Pose2f(pi_2, xPosition, theFieldDimensions.yPosRightReturnFromPenalty);
}

Pose2f SelfLocator::getNewPoseAtWalkInPosition()
{
  const SetupPoses::SetupPose& p = theSetupPoses.getPoseOfRobot(theGameState.playerNumber);
  Pose2f result;
  result.translation = p.position;
  result.rotation    = (p.turnedTowards - p.position).angle();
  return result;
}

Pose2f SelfLocator::getNewPoseAtManualPlacementPosition()
{
  // Goalie
  if(theGameState.isGoalkeeper())
  {
    return Pose2f(0.f, theFieldDimensions.xPosOwnGoalLine + 52.f, 0.f);
  }
  else
  {
    return Pose2f(0.f, demoCustomReturnFromPenaltyPoseFieldPlayer.translation.x(), demoCustomReturnFromPenaltyPoseFieldPlayer.translation.y());
  }
}

Pose2f SelfLocator::getNewPoseAtPenaltyShootoutPosition()
{
  if(theGameState.isForOwnTeam())
  {
    //striker pose (edge of penalty area, looking towards opponent goal)
    return Pose2f(0.f, theFieldDimensions.xPosPenaltyStrikerStartPosition, 0.f);
  }
  else
  {
    //goalie pose (in the center of the goal, looking towards the field's center)
    return Pose2f(0.f, theFieldDimensions.xPosOwnGoalLine, 0.f);
  }
}

bool SelfLocator::isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const
{
  const Vector2f& translation = currentPose.translation;
  Vector2f rotationWeight(std::max(theFieldDimensions.yPosLeftTouchline * 1.1f - std::min(std::abs(translation.x()), std::abs(lastPose.translation.x())), 0.f), 0);
  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoalPost, 0.f);
  const Vector2f rotation = Pose2f(Geometry::angleTo(currentPose, opponentGoal)) * rotationWeight;
  const Vector2f lastRotation = Pose2f(Geometry::angleTo(lastPose, opponentGoal)) * rotationWeight;
  bool result = (lastPose.translation - translation).norm() + (lastRotation - rotation).norm() >
                (lastPose.translation + translation).norm() + (lastRotation + rotation).norm();
  return result;
}

MAKE_MODULE(SelfLocator);
