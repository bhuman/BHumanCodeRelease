/**
* @file SelfLocator.cpp
*
* Implements a class that performs self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "SelfLocator.h"
#include "Tools/Global.h"
#include "Tools/Math/Probabilistics.h"
#include <algorithm>

using namespace std;


SelfLocator::SelfLocator() : fieldModel(theFieldDimensions, parameters, theCameraMatrix),
  sampleGenerator(parameters, theGoalPercept, theLinePercept, theFrameInfo,
  theFieldDimensions, theOdometryData), mirrorLikelihood(0.f), lastPenalty(-1), lastGameState(-1),
  timeOfLastFall(0), lastTimeWithoutArmContact(0), lastTimeFarGoalSeen(0)
{
  // Read parameters from disk
  InMapFile stream("selfLocator.cfg");
  ASSERT(stream.exists());
  stream >> parameters;

  // Set up subcomponent for sample resetting
  sampleGenerator.init();

  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFSample>(parameters.numberOfSamples);
  for(int i=0; i<samples->size(); ++i)
  {
    Pose2D pose = sampleGenerator.getTemplateAtWalkInPosition();
    samples->at(i).init(pose, parameters);
  }
}


SelfLocator::~SelfLocator()
{
  delete samples;
}


void SelfLocator::update(RobotPose& robotPose)
{
  /* Initialize variable(s) */
  sampleSetHasBeenResetted = false;

  /* Keep the sample generator up to date:
  *  - new goal perceptions are buffered
  *  - old goal perceptions are deleted
  *  - the pose is needed for making guesses about unknown goal post assignment
  */
  Pose2D propagatedRobotPose = robotPose + theOdometer.odometryOffset;
  sampleGenerator.bufferNewPerceptions(propagatedRobotPose);

  /* Modify sample set according to certain changes of the game state:
  *  - Reset mirror in SET
  *  - Handling of penalty positions
  *  - ...
  */
  handleGameStateChanges(propagatedRobotPose);

  /* Move all samples according to the current odometry.
  */
  motionUpdate();

  /* Integrate perceptions
  *  - goal posts, lines, corners ...
  *  - mirror flag of samples is treated in a separate method
  *     -- update depending on arm contacts, falls, ...
  */
  //mirrorFlagUpdate();    <--------- Deactivated for GO/RC 2013 (needs rethinking)
  sensorUpdate();

  /* After motion update and sensor update: Maintain some variables
  *  - mirror likelihood
  *  - sample validity
  */
  computeMirrorLikelihoodAndAdaptMirrorFlags();
  computeSampleValidities();

  /* Handle mirror information from SideConfidence
   * - Only for GO 2013
   */
  handleSideConfidence();

  /* Detect mirrored samples and put the back
   * - IS THIS USEFUL ???!?!?
   */
  if(theGameInfo.state == STATE_PLAYING && theGameInfo.secondaryState != STATE2_PENALTYSHOOT &&
    !theSideConfidence.mirror && !sampleSetHasBeenResetted)
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      const Pose2D samplePose =  samples->at(i).getPose();
      if(isMirrorCloser(samplePose, robotPose))
        samples->at(i).mirror();
    }
  }

  /* Actually: Hacks and workaround :-)
  *  - Treat keeper rotation problem
  */
  domainSpecificSituationHandling();

  /* Particle filter resampling step
  *  - not executed during penalty shootout!
  */
  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
    resampling();

  /* Fill the RobotPose representation based on the current sample set
  */
  computeModel(robotPose);

  /* Replace invalid samples
  *   This step is done at the end to make sure that the new samples
  *   get the full motion and sensor update steps before being involved
  *   in any pose computations.
  */
  sensorResetting(robotPose);

  /* Finally, update internal variables, debug and draw stuff.
  */
  DEBUG_RESPONSE("module:SelfLocator:templates_only",
  {
    if(sampleGenerator.templatesAvailable())
    {
      for(int i = 0; i < parameters.numberOfSamples; ++i)
      {
        UKFSample newSample;
        if(theOwnSideModel.stillInOwnSide)
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::OWN_HALF, mirrorLikelihood, robotPose, lastRobotPose), parameters);
        else
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, mirrorLikelihood, robotPose, lastRobotPose), parameters);
        samples->at(i) = newSample;
      }
    }
  });

  lastRobotPose = robotPose;
  MODIFY("representation:RobotPose", robotPose);

  MODIFY("parameters:SelfLocator", parameters);

  draw();
  EXECUTE_ONLY_IN_DEBUG(robotPose.draw(theOwnTeamInfo.teamColor != TEAM_BLUE););
  DECLARE_DEBUG_DRAWING("origin:Odometry", "drawingOnField",
  {
    Pose2D origin = robotPose + theOdometryData.invert();
    ORIGIN("origin:Odometry", origin.translation.x, origin.translation.y, origin.rotation);
  });
}


void SelfLocator::update(SideConfidence& sideConfidence)
{
  // mirror flag is not computed by this module
  sideConfidence.mirror = false;

  // Compute confidence value
  // Not playing -> sideConfidence 100%
  if(theGameInfo.state != STATE_PLAYING || theRobotInfo.penalty != PENALTY_NONE)
  {
    sideConfidence.sideConfidence = parameters.sideConfidenceConfident;
  }
  // Playing but on the safe side -> sideConfidence 100%
  else if(theOwnSideModel.stillInOwnSide)
  {
    sideConfidence.sideConfidence = parameters.sideConfidenceConfident;
  }
  // Leaving the safe side -> sideConfidence max 95%
  else
  {
    float confidence = parameters.sideConfidenceAlmostConfident;
    int numberOfMirroredSamples = 0;
    const int numberOfSamples = samples->size();
    for(int i=0; i<numberOfSamples; ++i)
    {
      if(samples->at(i).isMirrored())
        ++numberOfMirroredSamples;
    }
    if(numberOfMirroredSamples > numberOfSamples / 2)
      numberOfMirroredSamples = numberOfSamples / 2; // FIXME
    sideConfidence.sideConfidence = confidence * (1.f - static_cast<float>(numberOfMirroredSamples) / (numberOfSamples / 2));
  }

  // Set confidence state:
  if(sideConfidence.sideConfidence == parameters.sideConfidenceConfident)
    sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  else if(sideConfidence.sideConfidence == parameters.sideConfidenceAlmostConfident)
    sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;
  else if(sideConfidence.sideConfidence > parameters.sideConfidenceConfused)
    sideConfidence.confidenceState = SideConfidence::UNSURE;
  else
    sideConfidence.confidenceState = SideConfidence::CONFUSED;
}


void SelfLocator::computeModel(RobotPose& robotPose)
{
  UKFSample& result = getMostValidSample();
  Pose2D resultPose = result.getPose();

  // Determine average deviation of other samples from best sample:
  float xDiffSum(0.f);
  float yDiffSum(0.f);
  float rotDiffCosSum(0.f);
  float rotDiffSinSum(0.f);
  int count(0);
  for(int i=0; i<samples->size(); ++i)
  {
    if(samples->at(i).isMirrored())
      continue;
    Pose2D p = samples->at(i).getPose();
    xDiffSum      += abs(p.translation.x - resultPose.translation.x);
    yDiffSum      += abs(p.translation.y - resultPose.translation.y);
    float rotDiff =  abs(normalize(p.rotation - resultPose.rotation));
    rotDiffCosSum += cos(rotDiff);
    rotDiffSinSum += sin(rotDiff);
    ++count;
  }
  if(count == 0)
  {
    ASSERT(false); // Should not happen in the version for RoboCup 2013. REALLY. REALLY!
    return;
  }
  const float avgDiffX =   xDiffSum / count;
  const float avgDiffY =   yDiffSum / count;
  const float avgRotDiff = abs(atan2(rotDiffSinSum, rotDiffCosSum));

  // Build average of all samples that are within the average
  float xSum(resultPose.translation.x);
  float ySum(resultPose.translation.y);
  float rotCosSum(cos(resultPose.rotation));
  float rotSinSum(sin(resultPose.rotation));
  int count2(1);
  for(int i=0; i<samples->size(); ++i)
  {
    if(samples->at(i).isMirrored())
      continue;
    Pose2D p = samples->at(i).getPose();
    if(abs(p.translation.x - resultPose.translation.x) <= avgDiffX &&
       abs(p.translation.y - resultPose.translation.y) <= avgDiffY &&
       abs(normalize(p.rotation - resultPose.rotation)) <= avgRotDiff)
    {
      xSum += p.translation.x;
      ySum += p.translation.y;
      rotCosSum += cos(p.rotation);
      rotSinSum += sin(p.rotation);
      count2++;
    }
  }
  resultPose.translation.x = xSum / count2;
  resultPose.translation.y = ySum / count2;
  resultPose.rotation = atan2(rotSinSum, rotCosSum);

  // Override side information for testing on one side of a field only
  if(parameters.alwaysAssumeOpponentHalf && resultPose.translation.x < 0)
  {
    resultPose = Pose2D(pi) + resultPose;
  }
  robotPose.translation = resultPose.translation;
  robotPose.rotation = resultPose.rotation;
  Matrix3x3f cov = result.getCov();
  robotPose.deviation = sqrt(std::max(cov[0].x, cov[1].y));
  robotPose.validity = result.computeValidity(theFieldDimensions);
  robotPose.covariance[0][0] = cov[0][0];
  robotPose.covariance[0][1] = cov[0][1];
  robotPose.covariance[0][2] = cov[0][2];
  robotPose.covariance[1][0] = cov[1][0];
  robotPose.covariance[1][1] = cov[1][1];
  robotPose.covariance[1][2] = cov[1][2];
  robotPose.covariance[2][0] = cov[2][0];
  robotPose.covariance[2][1] = cov[2][1];
  robotPose.covariance[2][2] = cov[2][2];
  if(robotPose.validity >= parameters.validityThreshold)
    robotPose.validity = 1.f;
  else
    robotPose.validity *= (1.f / parameters.validityThreshold);
}


void SelfLocator::motionUpdate()
{
  const float transNoise = parameters.translationNoise;
  const float rotNoise = parameters.rotationNoise;
  const float transX = theOdometer.odometryOffset.translation.x;
  const float transY = theOdometer.odometryOffset.translation.y;
  const float dist = theOdometer.odometryOffset.translation.abs();
  const float angle = abs(theOdometer.odometryOffset.rotation);

  // precalculate rotational error that has to be adapted to all samples
  const float rotError = max(rotNoise, max(dist * parameters.movedDistWeight, angle * parameters.movedAngleWeight));

  // precalculate translational error that has to be adapted to all samples
  const float transXError = max(transNoise,
                                    max(abs(transX * parameters.majorDirTransWeight),
                                        abs(transY * parameters.minorDirTransWeight)));
  const float transYError = max(transNoise,
                                    max(abs(transY * parameters.majorDirTransWeight),
                                        abs(transX * parameters.minorDirTransWeight)));

  // update samples
  for(int i = 0; i < parameters.numberOfSamples; ++i)
  {
    const Vector2<> transOffset( (transX - transXError) + (2 * transXError) * randomFloat(),
                                 (transY - transYError) + (2 * transYError) * randomFloat());
    const float rotationOffset = theOdometer.odometryOffset.rotation + (randomFloat() * 2 - 1) * rotError;

    samples->at(i).motionUpdate(Pose2D(rotationOffset, transOffset), parameters);
  }
}


void SelfLocator::sensorUpdate()
{
  // Integrate lines
  if(theLinePercept.lines.size())
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      samples->at(i).updateByLinePercept(theLinePercept, fieldModel, parameters, theFieldDimensions, theMotionInfo, theCameraMatrix);
    }
  }
  // Integrate goal posts:
  if(theGoalPercept.goalPosts.size())
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      samples->at(i).updateByGoalPercept(theGoalPercept, fieldModel, parameters, theMotionInfo, theCameraMatrix);
    }
  }
  // Apply OwnSideModel:
  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      if(samples->at(i).getPose().translation.x > theOwnSideModel.largestXPossible)
        samples->at(i).invalidate();
    }
  }
}


void SelfLocator::sensorResetting(const RobotPose& robotPose)
{
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT) // Don't replace samples in penalty shootout
    return;
  if(theSideConfidence.mirror)                          // Don't replace samples in mirror cycle
    return;
  if(sampleGenerator.templatesAvailable())
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      if(samples->at(i).computeValidity(theFieldDimensions) == 0.f)
      {
        UKFSample newSample;
        if(theOwnSideModel.stillInOwnSide)
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::OWN_HALF, mirrorLikelihood, robotPose, lastRobotPose), parameters);
        else
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, mirrorLikelihood, robotPose, lastRobotPose), parameters);
        samples->at(i) = newSample;
      }
    }
  }
}


void SelfLocator::mirrorFlagUpdate()
{
  // Setting the time when the robot has a good stand
  if((theFallDownState.state != theFallDownState.upright &&
      theFallDownState.state != theFallDownState.undefined &&
      theFallDownState.state != theFallDownState.staggering) && (theFrameInfo.getTimeSince(timeOfLastFall) > 8000))
    timeOfLastFall = theFrameInfo.time;
  bool robotHasFallen = (timeOfLastFall == theFrameInfo.time);

  // Setting the time when the robot has no arm contact
  if(!theArmContactModel.contactLeft && !theArmContactModel.contactRight)
    lastTimeWithoutArmContact = theFrameInfo.time;
  // Arm contact!? Another robot may change my direction
  bool robotHasArmContact = theFrameInfo.getTimeSince(lastTimeWithoutArmContact) > parameters.minContinuousArmContact;
  if(robotHasArmContact)
    lastTimeWithoutArmContact = theFrameInfo.time;

  // Update all samples
  for(int i = 0; i < parameters.numberOfSamples; ++i)
  {
    samples->at(i).updateMirrorFlag(robotHasFallen, robotHasArmContact, parameters, theFieldDimensions);
  }
}


void SelfLocator::resampling()
{
  float totalWeighting(0.f);

  // Try to resample based on compliance with team ball:
  if(sampleSetIsMultimodal())
  {
    if(sufficientBallInformationAvailable())
    {
      for(int i = 0; i < parameters.numberOfSamples; ++i)
      {
        samples->at(i).computeWeightingBasedOnBallObservation(theBallModel.estimate.position, theCombinedWorldModel.ballStateOthers.position,
          theCameraMatrix.translation.z, parameters);
        totalWeighting += samples->at(i).weighting;
      }
    }
    else
    {
      return;
    }
  }
  // Check, if resampling should performed based on validity (currently only when goal posts are seen):
  else if(theGoalPercept.goalPosts.size())
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      samples->at(i).computeWeightingBasedOnValidity(theFieldDimensions, parameters);
      totalWeighting += samples->at(i).weighting;
    }
  }
  // Do not resample, otherwise:
  else
  {
    return;
  }

  // actual resampling step:
  UKFSample* oldSet = samples->swap();
  const int numberOfSamples(samples->size());
  const float weightingBetweenTwoDrawnSamples(totalWeighting / numberOfSamples);
  float nextPos(randomFloat() * weightingBetweenTwoDrawnSamples);
  float currentSum(0);

  // resample:
  int j(0);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    currentSum += oldSet[i].weighting;
    while(currentSum > nextPos && j < numberOfSamples)
    {
      samples->at(j++) = oldSet[i];
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }

  // fill up missing samples (could happen in rare cases) with new poses:
  for(; j < numberOfSamples; ++j)
  {
    const Pose2D pose = sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, mirrorLikelihood, lastRobotPose, lastRobotPose);
    samples->at(j).init(pose, parameters);
  }
}


int SelfLocator::sampleSetIsMultimodal()
{
  int mirrorCount(0);
  for(int i = 0; i < parameters.numberOfSamples; ++i)
  {
    if(samples->at(i).isMirrored())
      ++mirrorCount;
  }
  return mirrorCount;
}


bool SelfLocator::sufficientBallInformationAvailable()
{
  return ((theBallModel.timeWhenLastSeen == theFrameInfo.time) &&
          (theFieldDimensions.isInsideField(lastRobotPose * theBallModel.estimate.position)) && // Not 100% correct...
          (theBallModel.estimate.velocity.abs() <= parameters.maxBallVelocity) &&
          (theCombinedWorldModel.ballIsValidOthers) &&
          (theCombinedWorldModel.ballStateOthers.velocity.abs() <= parameters.maxBallVelocity));
}


void SelfLocator::handleSideConfidence()
{
  if(!theSideConfidence.mirror || sampleSetHasBeenResetted)
    return;
  for(int i = 0; i < parameters.numberOfSamples; ++i)
  {
    samples->at(i).mirror();
  }
}


void SelfLocator::handleGameStateChanges(const Pose2D& propagatedRobotPose)
{
  // We are in a penalty shootout!
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
  {
    // penalty shoot: if game state switched to playing reset samples to start position
    if((lastGameState != STATE_PLAYING && theGameInfo.state == STATE_PLAYING) ||
       (lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
    {
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColour)
      {
        //striker pose (1 meter behind the penalty spot, looking towards opponent goal)
        for(int i=0; i<samples->size(); ++i)
          samples->at(i).init(Pose2D(0.f, theFieldDimensions.xPosPenaltyStrikerStartPosition, 0.f), parameters);
        sampleSetHasBeenResetted = true;
      }
      else
      {
        //goalie pose (in the center of the goal, looking towards the field's center)
        for(int i=0; i<samples->size(); ++i)
          samples->at(i).init(Pose2D(0.f, theFieldDimensions.xPosOwnGroundline, 0.f), parameters);
        sampleSetHasBeenResetted = true;
      }
    }
  }
  // If a penalty is over, reset samples to reenter positions
  else if(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2D pose = sampleGenerator.getTemplateAtReenterPosition();
      samples->at(i).init(pose, parameters);
    }
    sampleSetHasBeenResetted = true;
  }
  // I am clearly in the opponent's half and will be placed manually
  else if(theGameInfo.state == STATE_SET && propagatedRobotPose.translation.x > 100.f)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2D pose = sampleGenerator.getTemplateAtManualPlacementPosition(theRobotInfo.number);
      samples->at(i).init(pose, parameters);
    }
    sampleSetHasBeenResetted = true;
  }
  // I am a keeper and not in my penalty area in SET
  else if(theGameInfo.state == STATE_SET && theRobotInfo.number == 1 &&
    propagatedRobotPose.translation.x > theFieldDimensions.xPosOwnPenaltyArea &&
    std::abs(propagatedRobotPose.translation.y) > theFieldDimensions.yPosLeftPenaltyArea)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2D pose = sampleGenerator.getTemplateAtManualPlacementPosition(theRobotInfo.number);
      samples->at(i).init(pose, parameters);
    }
    sampleSetHasBeenResetted = true;
  }
  // Normal game is about to start: We start on the sidelines looking at our goal: (this is for checking in TeamCom)
  else if(lastGameState != STATE_INITIAL && theGameInfo.state == STATE_INITIAL)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2D pose = sampleGenerator.getTemplateAtWalkInPosition();
      samples->at(i).init(pose, parameters);
    }
    sampleSetHasBeenResetted = true;
  }
  // Normal game really starts: We start on the sidelines looking at our goal: (this is for actual setup)
  else if(lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2D pose = sampleGenerator.getTemplateAtWalkInPosition();
      samples->at(i).init(pose, parameters);
    }
    sampleSetHasBeenResetted = true;
  }
  // In SET state, a robot cannot be mirrored => reset flags
  if(lastGameState == STATE_SET && theGameInfo.state != STATE_SET)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      samples->at(i).setMirrored(false);
    }
  }
  // Update members
  lastGameState = theGameInfo.state;
  lastPenalty   = theRobotInfo.penalty;
}


void SelfLocator::computeMirrorLikelihoodAndAdaptMirrorFlags()
{
  // Count the number of samples that appear to be mirrored:
  int mirroredSamples(0);
  for(int i = 0; i < parameters.numberOfSamples; ++i)
  {
    if(samples->at(i).isMirrored())
      mirroredSamples++;
  }
  // If most samples are mirrored, consider them as "normal" and
  // flip all samples in the sample set
  if(mirroredSamples > samples->size() / 2)
  {
    for(int i = 0; i < parameters.numberOfSamples; ++i)
    {
      const bool oldMirror = samples->at(i).isMirrored();
      samples->at(i).setMirrored(!oldMirror);
    }
    mirroredSamples = samples->size() - mirroredSamples;
  }
  // Compute likelihood
  mirrorLikelihood = static_cast<float>(mirroredSamples) / samples->size();
}


void SelfLocator::computeSampleValidities()
{
  for(int i=0; i < parameters.numberOfSamples; ++i)
  {
    samples->at(i).computeValidity(theFieldDimensions);
  }
}


UKFSample& SelfLocator::getMostValidSample()
{
  UKFSample& returnSample = samples->at(0);
  float maxValidity = -1.f;
  float minVariance = 0.f; // Initial value does not matter
  for(int i=0; i < parameters.numberOfSamples; ++i)
  {
    if(samples->at(i).isMirrored())
      continue;
    const float val = samples->at(i).validity;
    if(val > maxValidity)
    {
      maxValidity = val;
      minVariance = samples->at(i).getVarianceWeighting();
      returnSample = samples->at(i);
    }
    else if(val == maxValidity)
    {
      float variance = samples->at(i).getVarianceWeighting();
      if(variance < minVariance)
      {
        maxValidity = val;
        minVariance = variance;
        returnSample = samples->at(i);
      }
    }
  }
  return returnSample;
}


bool SelfLocator::isMirrorCloser(const Pose2D& samplePose, const Pose2D& robotPose) const
{
  const Vector2<>& translation = samplePose.translation;
  const Vector2<> rotationWeight(std::max(parameters.useRotationThreshold - std::min(translation.abs(), robotPose.translation.abs()), 0.f), 0);
  const Vector2<> rotation = Pose2D(samplePose.rotation) * rotationWeight;
  const Vector2<> robotRotation = Pose2D(robotPose.rotation) * rotationWeight;

  return (robotPose.translation - translation).abs() + (robotRotation - rotation).abs() >
         (robotPose.translation + translation).abs() + (robotRotation + rotation).abs();
}


void SelfLocator::domainSpecificSituationHandling()
{
  // Currently, this method only handles the case that the
  // keeper is turned by 180 degrees but assumes to stand correctly
  if(theRobotInfo.number != 1 || theGameInfo.state != STATE_PLAYING || theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
    return;
  // Save time of last seen opponent goal post
  for(unsigned int i=0; i<theGoalPercept.goalPosts.size(); ++i)
  {
    const GoalPost& post = theGoalPercept.goalPosts[i];
    if(post.positionOnField.abs() > theFieldDimensions.xPosOpponentGoal)
    {
      lastTimeFarGoalSeen = theFrameInfo.time;
      break;
    }
  }
}


void SelfLocator::draw()
{
  DECLARE_DEBUG_DRAWING("module:SelfLocator:samples", "drawingOnField"); // Draws all hypotheses/samples on the field
  COMPLEX_DRAWING("module:SelfLocator:samples",
    for(int i = 0; i < parameters.numberOfSamples; ++i) \
      samples->at(i).draw();
  );

  DECLARE_DEBUG_DRAWING("module:SelfLocator:simples", "drawingOnField"); // Draws all hypotheses/samples on the field in a simple way
  COMPLEX_DRAWING("module:SelfLocator:simples",
    for(int i = 0; i < parameters.numberOfSamples; ++i) \
      samples->at(i).draw(true);
  );

  DECLARE_DEBUG_DRAWING("module:SelfLocator:templates", "drawingOnField"); // Draws all available templates
  COMPLEX_DRAWING("module:SelfLocator:templates", sampleGenerator.draw(););
}


MAKE_MODULE(SelfLocator, Modeling)
