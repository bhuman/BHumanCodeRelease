/**
* @file RealisticBallChallengeSelfLocator.cpp
*
* Implements a class that performs self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "SelfLocator.h"
#include "RealisticBallChallengeSelfLocator.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>

using namespace std;

RealisticBallChallengeSelfLocator::RealisticBallChallengeSelfLocator() : fieldModel(theFieldDimensions, *((SelfLocator*)this), theCameraMatrix),
  sampleGenerator(*((SelfLocator*)this), theGoalPercept, theLinePercept, theFrameInfo,
                  theFieldDimensions, theOdometryData, theMotionInfo, theRobotInfo, verifiedCenterCircle, verifiedPenaltyMark),
  lastPenalty(-1), lastGameState(-1),
  lastTimeFarGoalSeen(0), lastTimeKeeperJumped(0), lastTimeJumpSound(0), timeOfLastReturnFromPenalty(0),
  nextSampleNumber(0), idOfLastBestSample(-1)
{
  // Set up subcomponent for sample resetting
  sampleGenerator.init();

  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFSample>(numberOfSamples);
  for(int i=0; i<samples->size(); ++i)
  {
    Pose2f pose = Pose2f(-pi_2, Vector2f(0.f, theFieldDimensions.yPosLeftSideline));
    samples->at(i).init(pose, *((SelfLocator*)this), nextSampleNumber++);
  }
}

RealisticBallChallengeSelfLocator::~RealisticBallChallengeSelfLocator()
{
  delete samples;
}

void RealisticBallChallengeSelfLocator::update(RobotPose& robotPose)
{
#ifndef NDEBUG
  ASSERT(!std::isnan(theOdometer.distanceWalked));
  ASSERT(!std::isnan(static_cast<float>(theOdometer.odometryOffset.rotation)));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.x()));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.y()));
  ASSERT(!std::isnan(theOwnSideModel.largestXPossible));
  for(const GoalPost& p : theGoalPercept.goalPosts)
  {
    ASSERT(!std::isnan(p.positionOnField.x()));
    ASSERT(!std::isnan(p.positionOnField.y()));
  }
  for(const LinePercept::Line& l : theLinePercept.lines)
  {
    ASSERT(!std::isnan(l.alpha));
    ASSERT(!std::isnan(l.d));
    ASSERT(!std::isnan(l.endInImage.x()));
    ASSERT(!std::isnan(l.endInImage.y()));
    ASSERT(!std::isnan(l.startInImage.x()));
    ASSERT(!std::isnan(l.startInImage.y()));
    ASSERT(!std::isnan(l.first.x()));
    ASSERT(!std::isnan(l.last.y()));
    ASSERT(!std::isnan(l.startInImage.x()));
    ASSERT(!std::isnan(l.startInImage.y()));
  }
  if(theLinePercept.circle.found)
  {
    ASSERT(!std::isnan(theLinePercept.circle.pos.x()));
    ASSERT(!std::isnan(theLinePercept.circle.pos.y()));
  }
  if(thePenaltyMarkPercept.timeLastSeen == theFrameInfo.time && theFrameInfo.time != 0)
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

  Pose2f propagatedRobotPose = robotPose + theOdometer.odometryOffset;
  
  /* Keep the sample generator up to date:
   *  - new goal perceptions are buffered
   *  - old goal perceptions are deleted
   *  - the pose is needed for making guesses about unknown goal post assignment
   */
  if((theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) &&
     theRobotInfo.penalty == PENALTY_NONE)
  {
    verifiedCenterCircle.updateCenterCircle(theLinePercept, theFrameInfo, theOdometryData);
    verifiedPenaltyMark.updatePenaltyMark(thePenaltyMarkPercept, theFrameInfo, theOdometryData);
    sampleGenerator.bufferNewPerceptions(propagatedRobotPose);
  }

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
  */
  sensorUpdate();

  /* After motion update and sensor update: Maintain some variables
  *  - sample validity
  */
  if(lastTimeKeeperJumped == 0 || theFrameInfo.getTimeSince(lastTimeKeeperJumped) > goalieJumpTimeout)
  {
    computeSampleValidities();
  }

  /* Handle mirror information from SideConfidence
   */
  handleSideConfidence();

  /* Detect mirrored samples and put them back
   * - IS THIS USEFUL ???!?!?
   */
  if(theGameInfo.state == STATE_PLAYING && theGameInfo.secondaryState != STATE2_PENALTYSHOOT &&
    !theSideConfidence.mirror && !sampleSetHasBeenResetted)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      const Pose2f samplePose =  samples->at(i).getPose();
      if(sampleGenerator.isMirrorCloser(samplePose, robotPose))
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
  DEBUG_RESPONSE("module:SelfLocator:templates_only")
  {
    if(sampleGenerator.templatesAvailable())
    {
      for(int i = 0; i < numberOfSamples; ++i)
      {
        UKFSample newSample;
        if(theOwnSideModel.stillInOwnSide)
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::OWN_HALF, robotPose, lastRobotPose), *((SelfLocator*)this), nextSampleNumber++);
        else
          newSample.init(sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, robotPose, lastRobotPose), *((SelfLocator*)this), nextSampleNumber++);
        samples->at(i) = newSample;
      }
    }
  }
  
  if((lastRobotPose.translation - robotPose.translation).norm() > positionJumpNotificationDistance &&
     theGameInfo.state == STATE_PLAYING && !theSideConfidence.mirror)
  {
    if(theFrameInfo.getTimeSince(lastTimeJumpSound) > 1337)
    {
      SystemCall::playSound("jump.wav");
      lastTimeJumpSound = theFrameInfo.time;
    }
    ANNOTATION("RealisticBallChallengeSelfLocator", "Robot position has jumped!");
  }

  lastRobotPose = robotPose;
  MODIFY("representation:RobotPose", robotPose);

  draw(robotPose);
  DEBUG_DRAWING("origin:Odometry", "drawingOnField")
  {
    Pose2f origin = robotPose + theOdometryData.inverse();
    ORIGIN("origin:Odometry", origin.translation.x(), origin.translation.y(), origin.rotation);
  }

  ASSERT(!std::isnan(robotPose.translation.x()));
  ASSERT(!std::isnan(robotPose.translation.y()));
  ASSERT(!std::isnan(static_cast<float>(robotPose.rotation)));
}

void RealisticBallChallengeSelfLocator::computeModel(RobotPose& robotPose)
{
  UKFSample& result = getMostValidSample();
  Pose2f resultPose = result.getPose();
  // Override side information for testing on one side of a field only
  if((alwaysAssumeOpponentHalf && resultPose.translation.x() < 0) ||
     (Global::getSettings().isGoalkeeper && theGameInfo.state == STATE_PLAYING && resultPose.translation.x() > 0))
  {
    resultPose = Pose2f(pi) + resultPose;
  }
  robotPose.translation = resultPose.translation;
  robotPose.rotation = resultPose.rotation;
  Matrix3f cov = result.getCov();
  robotPose.deviation = sqrt(std::max(cov(0, 0), cov(1, 1)));
  robotPose.validity = result.computeValidity(theFieldDimensions);
  robotPose.covariance = cov;
  if(robotPose.validity >= validityThreshold)
    robotPose.validity = 1.f;
  else
    robotPose.validity *= (1.f / validityThreshold);
  idOfLastBestSample = result.id;
}

void RealisticBallChallengeSelfLocator::motionUpdate()
{
  const float transNoise = translationNoise;
  const float rotNoise = rotationNoise;
  const float transX = theOdometer.odometryOffset.translation.x();
  const float transY = theOdometer.odometryOffset.translation.y();
  const float dist = theOdometer.odometryOffset.translation.norm();
  const float angle = abs(theOdometer.odometryOffset.rotation);

  // precalculate rotational error that has to be adapted to all samples
  const float rotError = max(rotNoise, max(dist * movedDistWeight, angle * movedAngleWeight));

  // precalculate translational error that has to be adapted to all samples
  const float transXError = max(transNoise,
                                    max(abs(transX * majorDirTransWeight),
                                        abs(transY * minorDirTransWeight)));
  const float transYError = max(transNoise,
                                    max(abs(transY * majorDirTransWeight),
                                        abs(transX * minorDirTransWeight)));

  // update samples
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Vector2f transOffset( (transX - transXError) + (2 * transXError) * randomFloat(),
                                 (transY - transYError) + (2 * transYError) * randomFloat());
    const float rotationOffset = theOdometer.odometryOffset.rotation + (randomFloat() * 2 - 1) * rotError;

    samples->at(i).motionUpdate(Pose2f(rotationOffset, transOffset), *((SelfLocator*)this));
  }
}

void RealisticBallChallengeSelfLocator::sensorUpdate()
{
  if(!(theMotionRequest.motion == MotionRequest::walk || theMotionRequest.motion == MotionRequest::stand ||
      (theMotionRequest.motion == MotionRequest::specialAction &&
       theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh)))
    return;
  
  // Integrate lines
  if(theLinePercept.lines.size() || theLinePercept.circle.found)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      samples->at(i).updateByLinePercept(theLinePercept, fieldModel, *((SelfLocator*)this), theFieldDimensions, theMotionInfo, theCameraMatrix);
    }
  }
  // Integrate penalty mark
  if(thePenaltyMarkPercept.timeLastSeen == theFrameInfo.time && theFrameInfo.time != 0)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      samples->at(i).updateByPenaltyMarkPercept(thePenaltyMarkPercept, fieldModel, *((SelfLocator*)this), theMotionInfo, theCameraMatrix);
    }
  }
  // Integrate goal posts:
  // (if there is any invalid goalpost, discard them all)
  bool postsAreOK = true;
  bool goalieInPenaltyArea = Global::getSettings().isGoalkeeper &&
                             theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea &&
                             abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea;
  for(auto checkPost : theGoalPercept.goalPosts)
  {
    if(!verifiedCenterCircle.isGoalpostCompatibleToCenterCircle(checkPost.positionOnField, theOdometryData))
    {
      postsAreOK = false;
      ANNOTATION("RealisticBallChallengeSelfLocator", "Goalpost has been discarded as it was too close to the center circle :-(");
      break;
    }
    if(!verifiedPenaltyMark.isGoalpostCompatibleToPenaltyMark(checkPost.positionOnField, theOdometryData))
    {
      postsAreOK = false;
      ANNOTATION("RealisticBallChallengeSelfLocator", "Goalpost has been discarded as it was too close to the penalty mark :-(");
      break;
    }
    if(goalieInPenaltyArea && checkPost.positionOnField.norm() > 3000.f)
    {
      postsAreOK = false;
      break;
    }
  }
  if(theGoalPercept.goalPosts.size() && postsAreOK)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      samples->at(i).updateByGoalPercept(theGoalPercept, fieldModel, *((SelfLocator*)this), theMotionInfo, theCameraMatrix);
    }
  }
}

void RealisticBallChallengeSelfLocator::sensorResetting(const RobotPose& robotPose)
{
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT) // Don't replace samples in penalty shootout
    return;
  if(theSideConfidence.mirror)                          // Don't replace samples in mirror cycle
    return;
  if(timeOfLastReturnFromPenalty != 0 && theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 20000)
    return;
  if(sampleGenerator.templatesAvailable())
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).computeValidity(theFieldDimensions) == 0.f)
      {
        UKFSample newSample;
        newSample.init(sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, robotPose, lastRobotPose), *((SelfLocator*)this), nextSampleNumber++);
        samples->at(i) = newSample;
      }
    }
  }
}

void RealisticBallChallengeSelfLocator::resampling()
{
  float totalWeighting(0.f);

  // Check, if resampling should performed based on validity (currently only when goal posts are seen):
  if(theGoalPercept.goalPosts.size())
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      samples->at(i).computeWeightingBasedOnValidity(theFieldDimensions, *((SelfLocator*)this));
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
    int replicationCount(0);
    while(currentSum > nextPos && j < numberOfSamples)
    {
      samples->at(j) = oldSet[i];
      if(replicationCount) // An old sample becomes copied multiple times: we need new identifier for the new instances
        samples->at(j).id = nextSampleNumber++;
      replicationCount++;
      j++;
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }

  // fill up missing samples (could happen in rare cases) with new poses:
  for(; j < numberOfSamples; ++j)
  {
    const Pose2f pose = sampleGenerator.getTemplate(TemplateGenerator::CONSIDER_POSE, lastRobotPose, lastRobotPose);
    samples->at(j).init(pose, *((SelfLocator*)this), nextSampleNumber++);
  }
  ASSERT(allSamplesIDsAreUnique());
}

void RealisticBallChallengeSelfLocator::handleSideConfidence()
{
  return;
}

void RealisticBallChallengeSelfLocator::handleGameStateChanges(const Pose2f& propagatedRobotPose)
{
  // If a penalty is over, reset samples to reenter positions (should not happen)
  if(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2f pose = Pose2f(-pi_2, Vector2f(0.f, theFieldDimensions.yPosLeftSideline));
      samples->at(i).init(pose, *((SelfLocator*)this), nextSampleNumber++);
    }
    sampleSetHasBeenResetted = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
    sampleGenerator.init();
    verifiedCenterCircle.reset();
    verifiedPenaltyMark.reset();
  }
  // Challenge is starting
  else if(theGameInfo.state == STATE_PLAYING && lastGameState != STATE_PLAYING)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2f pose = Pose2f(-pi_2, Vector2f(0.f, theFieldDimensions.yPosLeftSideline));
      samples->at(i).init(pose, *((SelfLocator*)this), nextSampleNumber++);
    }
    sampleSetHasBeenResetted = true;
    sampleGenerator.init();
    verifiedCenterCircle.reset();
    verifiedPenaltyMark.reset();
  }
  // just for debugging, not important
  else if(theGameInfo.state == STATE_SET && lastGameState != STATE_SET)
  {
    for(int i=0; i<samples->size(); ++i)
    {
      Pose2f pose = Pose2f(-pi_2, Vector2f(0.f, theFieldDimensions.yPosLeftSideline));
      samples->at(i).init(pose, *((SelfLocator*)this), nextSampleNumber++);
    }
    sampleSetHasBeenResetted = true;
    sampleGenerator.init();
    verifiedCenterCircle.reset();
    verifiedPenaltyMark.reset();
  }
  // Update members
  lastGameState = theGameInfo.state;
  lastPenalty   = theRobotInfo.penalty;
}

void RealisticBallChallengeSelfLocator::computeSampleValidities()
{
  for(int i=0; i < numberOfSamples; ++i)
  {
    samples->at(i).computeValidity(theFieldDimensions);
  }
}

UKFSample& RealisticBallChallengeSelfLocator::getMostValidSample()
{
  float validityOfLastBestSample = -1.f;
  UKFSample* lastBestSample = 0;
  if(idOfLastBestSample != -1)
  {
    for(int i=0; i < numberOfSamples; ++i)
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
  for(int i=0; i < numberOfSamples; ++i)
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

void RealisticBallChallengeSelfLocator::domainSpecificSituationHandling()
{
  return;
  // Currently, this method only handles the case that the
  // keeper is turned by 180 degrees but assumes to stand correctly
  if(!Global::getSettings().isGoalkeeper || theGameInfo.state != STATE_PLAYING || theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
    return;
  // Save time of last seen opponent goal post
  for(unsigned int i=0; i<theGoalPercept.goalPosts.size(); ++i)
  {
    const GoalPost& post = theGoalPercept.goalPosts[i];
    if(post.positionOnField.norm() > theFieldDimensions.xPosOpponentGoal)
    {
      lastTimeFarGoalSeen = theFrameInfo.time;
      break;
    }
  }
  // The robot is in its penalty area and assumes to look at the opponent half
  // and guards its goal.
  if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea &&
    abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea &&
    abs(theRobotPose.rotation) < 45_deg &&
    Global::getSettings().isGoalkeeper)
  {
    // Calculate distance to furthest point on field boundary
    float maxDistance = 0.f;
    for(auto& p : theFieldBoundary.boundaryOnField)
    {
      if(p.norm() > maxDistance)
        maxDistance = p.norm();
    }
    // It has not seen important stuff for a long time but the field border appears
    // to be close. This means that it is probably twisted:
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(lastTimeFarGoalSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(theLinePercept.circle.lastSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(thePenaltyMarkPercept.timeLastSeen) > goalieNoPerceptsThreshold &&
       maxDistance < goalieFieldBorderDistanceThreshold &&
       maxDistance > 400.f)
    {
      for(int i = 0; i < numberOfSamples; ++i)
        samples->at(i).twist();
    }
  }
}

void RealisticBallChallengeSelfLocator::draw(const RobotPose& robotPose)
{
  DECLARE_DEBUG_DRAWING("module:SelfLocator:samples", "drawingOnField"); // Draws all hypotheses/samples on the field
  COMPLEX_DRAWING("module:SelfLocator:samples")
    for(int i = 0; i < numberOfSamples; ++i) \
      samples->at(i).draw();

  DECLARE_DEBUG_DRAWING("module:SelfLocator:simples", "drawingOnField"); // Draws all hypotheses/samples on the field in a simple way
  COMPLEX_DRAWING("module:SelfLocator:simples")
    for(int i = 0; i < numberOfSamples; ++i) \
      samples->at(i).draw(true);

  DECLARE_DEBUG_DRAWING("module:SelfLocator:templates", "drawingOnField"); // Draws all available templates
  COMPLEX_DRAWING("module:SelfLocator:templates") sampleGenerator.draw();
  DECLARE_DEBUG_DRAWING("module:SelfLocator:templatesWithCenterCircles", "drawingOnField"); // Draws all available templates that have center circle information
  COMPLEX_DRAWING("module:SelfLocator:templatesWithCenterCircles") sampleGenerator.draw();
  DECLARE_DEBUG_DRAWING("module:SelfLocator:templatesWithLines", "drawingOnField"); // Draws all available templates that have line information
  COMPLEX_DRAWING("module:SelfLocator:templatesWithLines") sampleGenerator.draw();

  DECLARE_DEBUG_DRAWING("module:SelfLocator:verifiedCenterCircle", "drawingOnField");
  COMPLEX_DRAWING("module:SelfLocator:verifiedCenterCircle") verifiedCenterCircle.draw(theOdometryData);
  DECLARE_DEBUG_DRAWING("module:SelfLocator:verifiedPenaltyMark", "drawingOnField");
  COMPLEX_DRAWING("module:SelfLocator:verifiedPenaltyMark") verifiedPenaltyMark.draw(theOdometryData);
  
  DECLARE_DEBUG_DRAWING("module:SelfLocator:isMirrorCloser", "drawingOnField"); // Visualizes effect of current function's parameters
  COMPLEX_DRAWING("module:SelfLocator:isMirrorCloser")
  {
    const float stepSize = 500.f;
    const float degStepSize = 30.f;
    const float length = stepSize / 3.f;
    for(float x=theFieldDimensions.xPosOwnFieldBorder; x<=theFieldDimensions.xPosOpponentFieldBorder; x+=stepSize)
    {
      for(float y=theFieldDimensions.yPosRightFieldBorder; y<=theFieldDimensions.yPosLeftFieldBorder; y+=stepSize)
      {
        for(float rot=0.f; rot < 360.f; rot+=degStepSize)
        {
          Pose2f samplePose(Angle::fromDegrees(rot), x, y);
          ColorRGBA col = sampleGenerator.isMirrorCloser(samplePose, robotPose) ? ColorRGBA(255,0,0) : ColorRGBA(0,0,0);
          Vector2f vec(length,0.f);
          vec.rotate(Angle::fromDegrees(rot));
          LINE("module:SelfLocator:isMirrorCloser", x, y, x+vec.x(), y+vec.y(), 20, Drawings::solidPen, col);
        }
      }
    }
  }

  DECLARE_DEBUG_DRAWING("module:SelfLocator:fieldOfView", "drawingOnField");
  COMPLEX_DRAWING("module:SelfLocator:fieldOfView")
  {
    std::vector<Vector2f> p;
    Geometry::computeFieldOfViewInFieldCoordinates(robotPose, theCameraMatrix, theCameraInfo, theFieldDimensions, p);
    POLYGON("module:SelfLocator:fieldOfView", 4, p, 20, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, ColorRGBA(255, 255, 255, 25));
  }

  sampleGenerator.plot();
}

bool RealisticBallChallengeSelfLocator::allSamplesIDsAreUnique()
{
  for(int i=0; i < numberOfSamples - 1; ++i)
  {
    for(int j=i+1; j<numberOfSamples; ++j)
    {
      if(samples->at(i).id == samples->at(j).id)
        return false;
    }
  }
  return true;
}

MAKE_MODULE(RealisticBallChallengeSelfLocator, modeling)
