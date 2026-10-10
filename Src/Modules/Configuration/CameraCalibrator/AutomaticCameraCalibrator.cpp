/**
 * @file AutomaticCameraCalibrator.cpp
 *
 * This file implements a module that provides an automatic camera calibration
 * based on the penalty area.
 *
 * @author Arne Hasselbring
 */

#include "AutomaticCameraCalibrator.h"
#include "Math/Random.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(AutomaticCameraCalibrator);

AutomaticCameraCalibrator::AutomaticCameraCalibrator() :
  functor(*this),
  parallelDistRange(theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.fieldLinesWidth,
                    theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentGoalArea + theFieldDimensions.fieldLinesWidth)
{
  // Load the cameraCalibration from disk, if it exists.
  loadModuleParameters(const_cast<CameraCalibration&>(theCameraCalibration), "CameraCalibration", nullptr);
}

float AutomaticCameraCalibrator::calculateAngle(const Vector2f& lineAFirst, const Vector2f& lineASecond, const Vector2f& lineBFirst, const Vector2f& lineBSecond)
{
  const float dot = std::max(-1.f, std::min(static_cast<float>((lineAFirst - lineASecond).normalized().dot((lineBFirst - lineBSecond).normalized())), 1.f));
  return std::acos(dot);
}

void AutomaticCameraCalibrator::update(CameraCalibration& cameraCalibration)
{
  DECLARE_DEBUG_DRAWING("module:AutomaticCameraCalibrator:fieldLines", "drawingOnImage");

  nextCameraCalibration = theCameraCalibration;
  updateSampleConfiguration();

  // Calibration start requested.
  if(state == CameraCalibrationStatus::State::idle && theCalibrationRequest.targetState == CameraCalibrationStatus::State::recordSamples)
  {
    optimizer = nullptr;
    successiveConvergences = 0;
    optimizationSteps = 0;
    samples.resize(theCalibrationRequest.totalNumOfSamples);
    std::for_each(samples.begin(), samples.end(), [](std::unique_ptr<Sample>& sample) { sample.reset(); });
    nextCameraCalibration = theCameraCalibration;

    state = CameraCalibrationStatus::State::recordSamples;
    inStateSince = theFrameInfo.time;
  }

  // TODO: It would be nice to trigger this action with the request.
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:converge")
  {
    if(optimizer && lowestDelta != std::numeric_limits<float>::max())
    {
      unpack(lowestDeltaParameters, nextCameraCalibration);
      OUTPUT_TEXT("AutomaticCameraCalibrator: converged!");
      resetOptimization(true);
    }
  }

  if(state == CameraCalibrationStatus::State::recordSamples && currentSampleConfiguration &&
     theCalibrationRequest.sampleConfigurationRequest)
  {
    recordSamples();
  }

  if(theCalibrationRequest.targetState == CameraCalibrationStatus::State::optimize && state != CameraCalibrationStatus::State::optimize)
  {
    state = CameraCalibrationStatus::State::optimize;
    inStateSince = theFrameInfo.time;
  }

  if(state == CameraCalibrationStatus::State::optimize)
    optimize();

  cameraCalibration = nextCameraCalibration;

  COMPLEX_DRAWING("module:AutomaticCameraCalibrator:fieldLines")
    drawFieldLines();

  if(currentSampleConfiguration)
  {
    if(theFrameInfo.getTimeSince(lastCameraInfoTimestamp) > maxTimeBetweenExecutionCycle)
      noCameraTimeoutSinceTimestamp = theFrameInfo.time;
    lastCameraInfoTimestamp = theFrameInfo.time;
  }
}

void AutomaticCameraCalibrator::update(CameraCalibrationStatus& cameraCalibrationStatus)
{
  cameraCalibrationStatus.state = state;
  cameraCalibrationStatus.inStateSince = inStateSince;

  const SampleConfigurationStatus oldStatus = cameraCalibrationStatus.sampleConfigurationStatus;
  cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::none;

  if(state == CameraCalibrationStatus::State::idle)
  {
    cameraCalibrationStatus.sampleIndex = 0;
    return;
  }

  if(theCalibrationRequest.sampleConfigurationRequest)
  {
    cameraCalibrationStatus.sampleConfigurationStatus = allRequiredFeaturesVisible ? SampleConfigurationStatus::visible : SampleConfigurationStatus::notVisible;
    if(theFrameInfo.getTimeSince(noCameraTimeoutSinceTimestamp) < minTimeSinceNoCameraTimeout)
      cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::waiting;
    else if(allRequiredFeaturesVisible && theCalibrationRequest.sampleConfigurationRequest->doRecord)
      cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::recording;
    updateSampleConfiguration();
    if(currentSampleConfiguration && currentSampleConfiguration->samplesExist(samples))
    {
      if(cameraCalibrationStatus.sampleConfigurationStatus != SampleConfigurationStatus::finished)
        cameraCalibrationStatus.sampleIndex++;
      cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::finished;
    }
  }
  if(oldStatus != cameraCalibrationStatus.sampleConfigurationStatus)
    cameraCalibrationStatus.inStatusSince = theFrameInfo.time;
}

void AutomaticCameraCalibrator::update(CameraResolutionRequest&)
{
  // Nothing to do for booster robots
}

#define ADD_SAMPLE(sampleType, SampleName, first, second) \
  if(currentSampleConfiguration->needToRecord(samples, sampleType)) \
  { \
    auto s = std::make_unique<SampleName>(*this, theTorsoMatrix, theJointAngles.angles[Joints::headYaw], theJointAngles.angles[Joints::headPitch], theCameraInfo, first, second); \
    currentSampleConfiguration->record(samples, sampleType, std::move(s)); \
  }

#define INCREASE_RANGE(RangeName, discarded, found, numDiscarded, range) \
  if(discarded && !found) \
    ++numDiscarded; \
  if(numDiscarded >= discardsUntilIncrease) \
  { \
    numDiscarded = 0; \
    range = Rangef(range.min-increase, range.max+increase); \
    OUTPUT_TEXT(RangeName << " - Increased range"); \
  }

void AutomaticCameraCalibrator::recordSamples()
{
  if(theFrameInfo.getTimeSince(noCameraTimeoutSinceTimestamp) < minTimeSinceNoCameraTimeout) // Wait for camera finishing resetting
    return;

  const LinesPercept::Line& customLineI = theCustomCalibrationLines.lines[CustomCalibrationLines::closeGoalAreaConnectingLine];
  const LinesPercept::Line& customLineJ = theCustomCalibrationLines.lines[CustomCalibrationLines::innerGoalAreaParallelLine];
  const LinesPercept::Line& customLineK = theCustomCalibrationLines.lines[CustomCalibrationLines::goalLineGoalAreaParallelLine];
  const LinesPercept::Line& customLineL = theCustomCalibrationLines.lines[CustomCalibrationLines::farGoalAreaConnectingLine];
  const bool allCustomLinesSet = !customLineI.isEmpty() && !customLineJ.isEmpty() && !customLineK.isEmpty() && !customLineL.isEmpty();
  allRequiredFeaturesVisible = false;
  if((currentSampleConfiguration->needToRecord(samples, cornerAngle) || currentSampleConfiguration->needToRecord(samples, parallelAngle) || currentSampleConfiguration->needToRecord(samples, parallelLinesDistance)))
  {
    bool discardedParallelLines = false, foundParallelLines = false;

    if(!theCalibrationRequest.sampleConfigurationRequest->doRecord || !allCustomLinesSet) return;
    const LinesPercept::Line& lineI = customLineI;
    const LinesPercept::Line& lineJ = customLineJ;
    const LinesPercept::Line& lineK = customLineK;

    // Sort image points based on the x-coordinate. Otherwise the angles might be wrong
    Vector2f iFirstImg = lineI.firstImg.cast<float>();
    Vector2f iLastImg = lineI.lastImg.cast<float>();
    Vector2f jFirstImg = lineJ.firstImg.cast<float>();
    Vector2f jLastImg = lineJ.lastImg.cast<float>();
    Vector2f kFirstImg = lineK.firstImg.cast<float>();
    Vector2f kLastImg = lineK.lastImg.cast<float>();

    auto swapImgPoints = [](Vector2f& vec1, Vector2f& vec2)
    {
      if(vec1.x() > vec2.x())
      {
        const Vector2f temp1 = vec1;
        vec1 = vec2;
        vec2 = temp1;
      }
    };

    swapImgPoints(iFirstImg, iLastImg);
    swapImgPoints(jFirstImg, jLastImg);
    swapImgPoints(kFirstImg, kLastImg);

    // Fit lines through the start/end points of the lines
    Line cLine1 = { iFirstImg, iLastImg, customLineI.firstField, customLineI.lastField };
    Line cLine2 = { jFirstImg, jLastImg, customLineJ.firstField, customLineJ.lastField };
    Line cLine3 = { kFirstImg, kLastImg, customLineK.firstField, customLineK.lastField };

    // Check if we see the the other short connecting line of the goal area
    bool sampleFinished = !currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort);
    allRequiredFeaturesVisible |= !currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort);

    const LinesPercept::Line& lineL = customLineL;

    Vector2f lFirstImg = lineL.firstImg.cast<float>();
    Vector2f lLastImg = lineL.lastImg.cast<float>();

    swapImgPoints(lFirstImg, lLastImg);

    Line cLine4 = { lFirstImg, lLastImg, customLineL.firstField, customLineL.lastField };

    allRequiredFeaturesVisible = true;

    sampleFinished = true;
    const Geometry::Line line1(cLine1.aOnField, (cLine1.bOnField - cLine1.aOnField).normalized());
    ADD_SAMPLE(parallelLinesDistanceShort, ParallelLinesDistanceShortSample, cLine1, cLine4)

    if(sampleFinished)
    {
      if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) return;

      ADD_SAMPLE(cornerAngle, CornerAngleSample, cLine1, ((cLine2.bOnField - cLine2.aOnField).squaredNorm() <
                                                          (cLine3.bOnField - cLine3.aOnField).squaredNorm() ? cLine2 : cLine3))
      ADD_SAMPLE(parallelAngle, ParallelAngleSample, cLine2, cLine3)
      ADD_SAMPLE(parallelLinesDistance, ParallelLinesDistanceSample, cLine2, cLine3)
    }
    else
      discardedParallelLines = true;

    if(theCalibrationRequest.sampleConfigurationRequest->doRecord)
    {
      INCREASE_RANGE("ParallelDisRange", discardedParallelLines, foundParallelLines, numOfDiscardedParallelLines, parallelDistRange)
    }
  }

  // Make sure the head movement is registered.
  if(!currentSampleConfiguration->needToRecord(samples, cornerAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelLinesDistance) &&
     !currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort))
  {
    numOfDiscardedParallelLines = numOfDiscardedGoalAreaLines = numOfDiscardedGoalLines = 0;
    allRequiredFeaturesVisible = true;
  }
}

void AutomaticCameraCalibrator::optimize()
{
  if(!optimizer)
  {
    optimizer = std::make_unique<GaussNewtonOptimizer<numOfParameterTranslations>>(functor);
    optimizationParameters = pack(theCameraCalibration);
    successiveConvergences = 0;
  }
  else
  {
    const float delta = optimizer->iterate(optimizationParameters, Parameters::Constant(0.0001f));
    if(!std::isfinite(delta))
    {
      resetOptimization(false);
      return;
    }

    for(size_t i = 0; i < functor.getNumOfMeasurements(); ++i)
    {
      if(functor(optimizationParameters, i) >= notValidError)
      {
        resetOptimization(false);
        return;
      }
    }

    OUTPUT_TEXT("AutomaticCameraCalibrator: delta = " << delta << "\n");
    if(std::abs(delta) < lowestDelta)
    {
      lowestDelta = std::abs(delta);
      lowestDeltaParameters = optimizationParameters;
    }
    ++optimizationSteps;
    if(std::abs(delta) < (terminationCriterion * std::max(1u, optimizationSteps / 500 * 50)))
      ++successiveConvergences;
    else
      successiveConvergences = 0;
    if(successiveConvergences > 0)
    {
      float error = 0;
      for(size_t i = 0; i < samples.size(); i++)
        error += functor(optimizationParameters, i);
      error /= static_cast<float>(samples.size());
      if(successiveConvergences == 1 || error < lowestError)
      {
        lowestError = error;
        lowestErrorParameters = optimizationParameters;
      }
    }
    if(successiveConvergences >= minSuccessiveConvergences)
    {
      OUTPUT_TEXT("AutomaticCameraCalibrator: converged!");
      unpack(lowestErrorParameters, nextCameraCalibration);
      resetOptimization(true);
    }
    else
      unpack(optimizationParameters, nextCameraCalibration);
  }
}

void AutomaticCameraCalibrator::resetOptimization(const bool finished)
{
  if(finished)
    state = CameraCalibrationStatus::State::idle;
  else
  {
    OUTPUT_TEXT("Restart optimize! An optimization error occurred!");
    FOREACH_ENUM(ParameterTranslation, parameter)
      optimizationParameters(parameter) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    unpack(optimizationParameters, nextCameraCalibration);
  }

  optimizer = nullptr;
  lowestDelta = std::numeric_limits<float>::max();
  lowestDeltaParameters = Parameters();
  optimizationSteps = 0;
}

AutomaticCameraCalibrator::Parameters AutomaticCameraCalibrator::pack(const CameraCalibration& cameraCalibration) const
{
  Parameters params;
  FOREACH_ENUM(ParameterTranslation, parameter)
    params(parameter) = parameter < firstBodyCorrection
                        ? cameraCalibration.cameraRotationCorrection(parameter)
                        : cameraCalibration.bodyRotationCorrection(parameter - firstBodyCorrection);

  return params;
}

void AutomaticCameraCalibrator::unpack(const Parameters& params, CameraCalibration& cameraCalibration) const
{
  FOREACH_ENUM(ParameterTranslation, parameter)
    (parameter < firstBodyCorrection
      ? cameraCalibration.cameraRotationCorrection(parameter)
      : cameraCalibration.bodyRotationCorrection(parameter - firstBodyCorrection)) = static_cast<float>(std::fmod(params(parameter), 360_deg));
}

void AutomaticCameraCalibrator::updateSampleConfiguration()
{
  if(!theCalibrationRequest.sampleConfigurationRequest)
  {
    return;
  }
  if(static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index) <= lastSampleConfigurationIndex)
    return;

  const auto& request = theCalibrationRequest.sampleConfigurationRequest;

  currentSampleConfiguration = std::make_unique<SampleConfiguration>();
  currentSampleConfiguration->headPan = request->headPan;
  currentSampleConfiguration->headTilt = request->headTilt;
  currentSampleConfiguration->sampleTypes = request->sampleTypes;
  currentSampleConfiguration->sampleIndexBase = numOfSamples;
  FOREACH_ENUM(SampleType, sampleType)
    if(request->sampleTypes & bit(sampleType))
      ++numOfSamples;

  lastSampleConfigurationIndex = static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index);
}

#define CHECK_LINE_PROJECTION(line, camMat, camInf) \
  (Transformation::imageToRobot(line.aInImage, camMat, camInf, line.aOnField) && \
   Transformation::imageToRobot(line.bInImage, camMat, camInf, line.bOnField))

#define CHECK_PROJECTION_2_LINES(SampleName, line1, line2) \
  if(!CHECK_LINE_PROJECTION(line1, cameraMatrix, cameraInfo) || \
     !CHECK_LINE_PROJECTION(line2, cameraMatrix, cameraInfo)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

#define CHECK_PROJECTION_LINE_PENALTY(SampleName, line, p, p2) \
  if(!CHECK_LINE_PROJECTION(line, cameraMatrix, cameraInfo) || \
     !Transformation::imageToRobot(p, cameraMatrix, cameraInfo, p2)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

float AutomaticCameraCalibrator::Sample::computeError(const CameraCalibration& cameraCalibration) const
{
  const RobotCameraMatrix robotCameraMatrix(calibrator.theRobotDimensions, headYaw, headPitch, cameraCalibration);
  const CameraMatrix cameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
  return computeError(cameraMatrix);
}

float AutomaticCameraCalibrator::CornerAngleSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("CornerAngleSample", cLine1, cLine2)

  const float cornerAngle = calculateAngle(cLine1.aOnField, cLine1.bOnField, cLine2.aOnField, cLine2.bOnField);
  const float cornerAngleError = std::abs(90_deg - cornerAngle);
  OUTPUT_TEXT("Angle 90: " << cornerAngle << ", error: " << cornerAngleError);
  return cornerAngleError / calibrator.angleErrorDivisor;
}

float AutomaticCameraCalibrator::ParallelAngleSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("ParallelAngleSample", cLine1, cLine2)

  const float parallelAngle = calculateAngle(cLine1.aOnField, cLine1.bOnField, cLine2.aOnField, cLine2.bOnField);
  const float parallelAngleError = std::min(parallelAngle, 180_deg - parallelAngle);
  OUTPUT_TEXT("Angle 180: " << parallelAngle << ", error: " << parallelAngleError);
  return parallelAngleError / calibrator.angleErrorDivisor;
}

float AutomaticCameraCalibrator::ParallelLinesDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("ParallelLinesDistanceSample", cLine1, cLine2)

  const Geometry::Line line1(cLine1.aOnField, (cLine1.bOnField - cLine1.aOnField).normalized());
  const Geometry::Line line2(cLine2.aOnField, (cLine2.bOnField - cLine2.aOnField).normalized());
  const float distance1 = Geometry::getDistanceToLineSigned(line1, 0.5f * (cLine2.aOnField + cLine2.bOnField));
  const float distance2 = Geometry::getDistanceToLineSigned(line2, 0.5f * (cLine1.aOnField + cLine1.bOnField));

  const float combinedOffset = distance1 > 0 ? cLine1.offset - cLine2.offset : cLine2.offset - cLine1.offset;
  const float optimalDistance = calibrator.theFieldDimensions.xPosOpponentGoalLine - calibrator.theFieldDimensions.xPosOpponentGoalArea + combinedOffset;

  const float lineDistanceError = std::max(std::abs(std::abs(distance1) - optimalDistance), std::abs(std::abs(distance2) - optimalDistance));
  OUTPUT_TEXT("LineDistanceError: " << lineDistanceError);
  return lineDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibrator::ParallelLinesDistanceShortSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("ParallelLinesDistanceShortSample", cLine1, cLine2)

  const Geometry::Line line1(cLine1.aOnField, (cLine1.bOnField - cLine1.aOnField).normalized());
  const Geometry::Line line2(cLine2.aOnField, (cLine2.bOnField - cLine2.aOnField).normalized());
  const float distance1 = Geometry::getDistanceToLineSigned(line1, 0.5f * (cLine2.aOnField + cLine2.bOnField));
  const float distance2 = Geometry::getDistanceToLineSigned(line2, 0.5f * (cLine1.aOnField + cLine1.bOnField));

  const float combinedOffset = distance1 > 0 ? cLine1.offset - cLine2.offset : cLine2.offset - cLine1.offset;
  const float optimalDistance = calibrator.theFieldDimensions.yPosLeftGoalArea - calibrator.theFieldDimensions.yPosRightGoalArea + combinedOffset;

  const float lineDistanceError = (std::abs(std::abs(distance1) - optimalDistance) + std::abs(std::abs(distance2) - optimalDistance)) * 0.5f;
  OUTPUT_TEXT("LineDistanceShortError: " << lineDistanceError);
  return lineDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibrator::Functor::operator()(const Parameters& params, size_t measurement) const
{
  if(calibrator.samples[measurement] == nullptr)
    return 0.f;
  CameraCalibration cameraCalibration = calibrator.nextCameraCalibration;
  calibrator.unpack(params, cameraCalibration);
  return calibrator.samples[measurement]->computeError(cameraCalibration);
}

bool AutomaticCameraCalibrator::SampleConfiguration::needToRecord(const std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType) const
{
  if(!(sampleTypes & bit(sampleType)))
    return false;
  size_t sampleIndex = sampleIndexBase;
  FOREACH_ENUM(SampleType, x, sampleType)
  {
    if(sampleTypes & bit(x))
      ++sampleIndex;
  }
  ASSERT(sampleIndex < samples.size());
  return !samples[sampleIndex];
}

void AutomaticCameraCalibrator::SampleConfiguration::record(std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType, std::unique_ptr<Sample> sample) const
{
  ASSERT(sampleTypes & bit(sampleType));
  size_t sampleIndex = sampleIndexBase;
  FOREACH_ENUM(SampleType, x, sampleType)
  {
    if(sampleTypes & bit(x))
      ++sampleIndex;
  }
  ASSERT(sampleIndex < samples.size());
  samples[sampleIndex] = std::move(sample);
}

bool AutomaticCameraCalibrator::SampleConfiguration::samplesExist(const std::vector<std::unique_ptr<Sample>>& samples) const
{
  size_t sampleIndex = sampleIndexBase;
  FOREACH_ENUM(SampleType, sampleType)
  {
    if(sampleTypes & bit(sampleType))
    {
      if(!samples[sampleIndex])
        return false;
      ++sampleIndex;
    }
  }
  return true;
}

void AutomaticCameraCalibrator::drawFieldLines() const
{
  const Pose2f robotPoseInv = validationRobotPose.inverse();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from), theCameraMatrix, theCameraInfo, lineInImage))
      LINE("module:AutomaticCameraCalibrator:fieldLines", lineInImage.base.x(), lineInImage.base.y(), (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1, Drawings::solidPen, ColorRGBA::black);
  }
}

bool AutomaticCameraCalibrator::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
{
  const float& f = cameraInfo.focalLength;
  const Pose3f cameraMatrixInv = cameraMatrix.inverse();

  const Vector2f& p1 = lineOnField.base;
  const Vector2f p2 = p1 + lineOnField.direction;
  Vector3f p1Camera = cameraMatrixInv * Vector3f(p1.x(), p1.y(), 0), p2Camera = cameraMatrixInv * Vector3f(p2.x(), p2.y(), 0);

  const bool p1Behind = p1Camera.x() < cameraInfo.focalLength, p2Behind = p2Camera.x() < cameraInfo.focalLength;
  if(p1Behind && p2Behind)
    return false;
  else if(!p1Behind && !p2Behind)
  {
    p1Camera /= (p1Camera.x() / f);
    p2Camera /= (p2Camera.x() / f);
  }
  else
  {
    const Vector3f direction = p1Camera - p2Camera;
    const float scale = (f - p1Camera.x()) / direction.x();
    const Vector3f intersection = p1Camera + direction * scale;
    if(p1Behind)
    {
      p1Camera = intersection;
      p2Camera /= (p2Camera.x() / f);
    }
    else
    {
      p2Camera = intersection;
      p1Camera /= (p1Camera.x() / f);
    }
  }
  lineInImage.base = Vector2f(cameraInfo.opticalCenter.x() - p1Camera.y(), cameraInfo.opticalCenter.y() - p1Camera.z());
  lineInImage.direction = Vector2f(cameraInfo.opticalCenter.x() - p2Camera.y(), cameraInfo.opticalCenter.y() - p2Camera.z()) - lineInImage.base;
  return true;
}
