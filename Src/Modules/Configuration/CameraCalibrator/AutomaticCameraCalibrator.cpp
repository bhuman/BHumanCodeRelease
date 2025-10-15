/**
 * @file AutomaticCameraCalibrator.cpp
 *
 * This file implements a module that provides an automatic camera calibration
 * based on the penalty area.
 *
 * @author Arne Hasselbring
 */

#include "AutomaticCameraCalibrator.h"
#include "Platform/SystemCall.h"
#include "Debugging/Annotation.h"
#include "Math/Random.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(AutomaticCameraCalibrator);

AutomaticCameraCalibrator::AutomaticCameraCalibrator() :
  functor(*this)
{
  parallelDisRangeLower = parallelDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.fieldLinesWidth,
                                                         theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentGoalArea + theFieldDimensions.fieldLinesWidth);
  goalAreaDisRangeLower = goalAreaDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.fieldLinesWidth,
                                                         theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyMark + theFieldDimensions.fieldLinesWidth);
  goalLineDisRangeLower = goalLineDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.fieldLinesWidth,
                                                         theFieldDimensions.xPosOpponentGoalLine - theFieldDimensions.xPosOpponentPenaltyMark + theFieldDimensions.fieldLinesWidth);

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
  DEBUG_DRAWING("module:AutomaticCameraCalibrator:fieldLines", "drawingOnImage")
    THREAD("module:AutomaticCameraCalibrator:fieldLines", theCameraInfo.getThreadName());

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

  // Abort requested.
  if(theCalibrationRequest.targetState == CameraCalibrationStatus::State::idle && state != CameraCalibrationStatus::State::idle)
  {
    samples.clear();
    currentSampleConfiguration = nullptr;
    state = CameraCalibrationStatus::State::idle;
    inStateSince = theFrameInfo.time;
    numOfSamples = 0;
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
     theCalibrationRequest.sampleConfigurationRequest && theOptionalECImage.image)
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

  if(currentSampleConfiguration && currentSampleConfiguration->camera == theCameraInfo.camera)
  {
    if(theFrameInfo.getTimeSince(lastCameraInfoTimestamp) > maxTimeBetweenExecutionCycle)
      noCameraTimeoutSinceTimestamp = theFrameInfo.time;
    lastCameraInfoTimestamp = theFrameInfo.time;
  }
}

void AutomaticCameraCalibrator::update(CameraCalibrationStatus& cameraCalibrationStatus)
{
  if(state == CameraCalibrationStatus::State::idle)
    cameraCalibrationStatus.sampleIndex = 0;

  cameraCalibrationStatus.state = state;
  cameraCalibrationStatus.inStateSince = inStateSince;

  const SampleConfigurationStatus oldStatus = cameraCalibrationStatus.sampleConfigurationStatus;
  cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::none;
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

void AutomaticCameraCalibrator::update(CameraResolutionRequest& cameraResolutionRequest)
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
  {
    if(state == CameraCalibrationStatus::State::idle || state == CameraCalibrationStatus::State::optimize)
    {
      cameraResolutionRequest.resolutions[CameraInfo::lower] = CameraResolutionRequest::Resolutions::defaultRes;
      cameraResolutionRequest.resolutions[CameraInfo::upper] = CameraResolutionRequest::Resolutions::defaultRes;
    }
    else
    {
      if(currentSampleConfiguration && currentSampleConfiguration->camera == CameraInfo::lower)
        cameraResolutionRequest.resolutions[CameraInfo::lower] = resRequest[CameraInfo::lower];
      else
        cameraResolutionRequest.resolutions[CameraInfo::lower] = CameraResolutionRequest::Resolutions::defaultRes;
      if(!currentSampleConfiguration || currentSampleConfiguration->camera == CameraInfo::upper)
        cameraResolutionRequest.resolutions[CameraInfo::upper] = resRequest[CameraInfo::upper];
      else
        cameraResolutionRequest.resolutions[CameraInfo::upper] = CameraResolutionRequest::Resolutions::defaultRes;
    }
  }
}

#define ADD_SAMPLE(sampleType, SampleName, first, second) \
  if(currentSampleConfiguration->needToRecord(samples, sampleType)) \
  { \
    auto s = std::make_unique<SampleName>(*this, theTorsoMatrix, theRobotModel, theCameraInfo, theImageCoordinateSystem, first, second); \
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
  if(theCameraInfo.camera != currentSampleConfiguration->camera ||
     theFrameInfo.getTimeSince(noCameraTimeoutSinceTimestamp) < minTimeSinceNoCameraTimeout) // Wait for camera finishing resetting
    return;

  const LinesPercept::Line& customLineI = theCustomCalibrationLines.lines[CustomCalibrationLines::closeGoalAreaConnectingLine];
  const LinesPercept::Line& customLineJ = theCustomCalibrationLines.lines[CustomCalibrationLines::outerGoalAreaParallelLine];
  const LinesPercept::Line& customLineK = theCustomCalibrationLines.lines[CustomCalibrationLines::innerGoalAreaParallelLine];
  const LinesPercept::Line& customLineL = theCustomCalibrationLines.lines[CustomCalibrationLines::farGoalAreaConnectingLine];
  const bool allCustomLinesSet = !customLineI.isEmpty() && !customLineJ.isEmpty() && !customLineK.isEmpty() && !customLineL.isEmpty();

  allRequiredFeaturesVisible = false;
  if(((theLinesPercept.lines.size() >= 3 && theLinesPercept.lines.size() <= 9) || allCustomLinesSet) &&
     (currentSampleConfiguration->needToRecord(samples, cornerAngle) || currentSampleConfiguration->needToRecord(samples, parallelAngle) || currentSampleConfiguration->needToRecord(samples, parallelLinesDistance)) &&
     !(currentSampleConfiguration->needToRecord(samples, goalAreaDistance) || currentSampleConfiguration->needToRecord(samples, goalLineDistance)))
  {
    bool discardedParallelLines = false, foundParallelLines = false;
    Rangef& parallelLinesRange = currentSampleConfiguration->camera == CameraInfo::upper ? parallelDisRangeUpper : parallelDisRangeLower;
    for(size_t i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      for(size_t j = 0; j < theLinesPercept.lines.size(); ++j)
      {
        if(i == j || foundParallelLines || allRequiredFeaturesVisible)
          continue;
        for(size_t k = j + 1; k < theLinesPercept.lines.size(); ++k)
        {
          if(i == k || foundParallelLines || allRequiredFeaturesVisible)
            continue;

          const LinesPercept::Line& lineI = !customLineI.isEmpty() ? customLineI : theLinesPercept.lines[i];
          const LinesPercept::Line& lineJ = !customLineJ.isEmpty() ? customLineJ : theLinesPercept.lines[j];
          const LinesPercept::Line& lineK = !customLineK.isEmpty() ? customLineK : theLinesPercept.lines[k];

          // i is the index of the "short" connecting line, j and k are the indices for the orthogonal lines (goal line and front goal area line).
          // It is checked whether the line i has one end in one line and the other end in the other line.
          // TODO: This should be done in image coordinates because otherwise the camera would have to be calibrated. But still works.
          const float distInImage1 = std::min(Geometry::getDistanceToEdge(lineJ.line, lineI.firstField.cast<float>()),
                                              Geometry::getDistanceToEdge(lineJ.line, lineI.lastField.cast<float>()));
          if(distInImage1 > 100.f)
            continue;
          const float distInImage2 = std::min(Geometry::getDistanceToEdge(lineK.line, lineI.firstField.cast<float>()),
                                              Geometry::getDistanceToEdge(lineK.line, lineI.lastField.cast<float>()));
          if(distInImage2 > 100.f)
            continue;

          // Make sure that the short connecting line is the closest as seen from the robot.
          const float squaredDistanceToShortLineStart = lineI.firstField.squaredNorm();
          const float squaredDistanceToShortLineEnd = lineI.lastField.squaredNorm();
          const float squaredDistanceToShortLine = squaredDistanceToShortLineStart < squaredDistanceToShortLineEnd ? squaredDistanceToShortLineStart : squaredDistanceToShortLineEnd;

          if(((lineJ.firstField + lineJ.lastField) * 0.5f).squaredNorm() < squaredDistanceToShortLine ||
             ((lineK.firstField + lineK.lastField) * 0.5f).squaredNorm() < squaredDistanceToShortLine)
            continue;

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

          // Roughly check whether the angles are reasonable (in image coordinates)
          const Angle angleIJ = calculateAngle(iFirstImg, iLastImg,
                                               jFirstImg, jLastImg);
          const Angle angleIK = calculateAngle(iFirstImg, iLastImg,
                                               kFirstImg, kLastImg);
          const Angle angleJK = calculateAngle(jFirstImg, jLastImg,
                                               kFirstImg, kLastImg);
          if(angleIJ < 20_deg || angleIJ > 160_deg || angleIK < 20_deg || angleIK > 160_deg || angleJK > 40_deg)
            continue;

          const float iImgLengthSqr = (iFirstImg - iLastImg).squaredNorm();
          if(iImgLengthSqr > (kFirstImg - kLastImg).squaredNorm() || iImgLengthSqr > (jFirstImg - jLastImg).squaredNorm())
            continue;

          // Fit lines through the start/end points of the lines
          LineCorrector::Line cLine1 = { iFirstImg, iLastImg, !customLineI.isEmpty() ? customLineI.firstField : Vector2f(), !customLineI.isEmpty() ? customLineI.lastField : Vector2f()};
          LineCorrector::Line cLine2 = { jFirstImg, jLastImg, !customLineJ.isEmpty() ? customLineJ.firstField : Vector2f(), !customLineJ.isEmpty() ? customLineJ.lastField : Vector2f() };
          LineCorrector::Line cLine3 = { kFirstImg, kLastImg, !customLineK.isEmpty() ? customLineK.firstField : Vector2f(), !customLineK.isEmpty() ? customLineK.lastField : Vector2f() };
          if((customLineI.isEmpty() && !theLineCorrector.fitLine(cLine1)) || (customLineJ.isEmpty() && !theLineCorrector.fitLine(cLine2)) || (customLineK.isEmpty() && !theLineCorrector.fitLine(cLine3)))
            continue;

          const Geometry::Line line2(cLine2.aOnField, (cLine2.bOnField - cLine2.aOnField).normalized());
          const float distance = Geometry::getDistanceToLineSigned(line2, (cLine3.aOnField + cLine3.bOnField) * 0.5f);
          const float combinedOffset = distance > 0 ? cLine2.offset - cLine3.offset : cLine3.offset - cLine2.offset;

          foundParallelLines = parallelLinesRange.isInside(std::abs(distance) - combinedOffset);

          // Check if we see the the other short connecting line of the goal area
          bool sampleFinished = !currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort);
          allRequiredFeaturesVisible |= !currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort);
          if((theLinesPercept.lines.size() >= 4 || !customLineL.isEmpty()) && currentSampleConfiguration->needToRecord(samples, parallelLinesDistanceShort))
          {
            for(size_t l = 0; l < theLinesPercept.lines.size(); ++l)
            {
              if(l == i || l == j || l == k || allRequiredFeaturesVisible)
                continue;

              const LinesPercept::Line& lineL = !customLineL.isEmpty() ? customLineL : theLinesPercept.lines[l];

              const float distInImage1 = std::min(Geometry::getDistanceToEdge(lineJ.line, lineL.firstField.cast<float>()),
                                                  Geometry::getDistanceToEdge(lineJ.line, lineL.lastField.cast<float>()));
              if(distInImage1 > 100.f)
                continue;
              const float distInImage2 = std::min(Geometry::getDistanceToEdge(lineK.line, lineL.firstField.cast<float>()),
                                                  Geometry::getDistanceToEdge(lineK.line, lineL.lastField.cast<float>()));
              if(distInImage2 > 100.f)
                continue;

              const float squaredDistanceToOtherShortLine = (lineI.firstField - lineI.lastField).squaredNorm();

              if(((lineJ.firstField + lineJ.lastField) * 0.5f).squaredNorm() < squaredDistanceToOtherShortLine ||
                 ((lineK.firstField + lineK.lastField) * 0.5f).squaredNorm() < squaredDistanceToOtherShortLine)
                continue;

              Vector2f lFirstImg = lineL.firstImg.cast<float>();
              Vector2f lLastImg = lineL.lastImg.cast<float>();

              swapImgPoints(lFirstImg, lLastImg);

              const Angle angleLJ = calculateAngle(lFirstImg, lLastImg,
                                                   jFirstImg, jLastImg);
              const Angle angleLK = calculateAngle(lFirstImg, lLastImg,
                                                   kFirstImg, kLastImg);

              if(angleLJ < 15_deg || angleLJ > 160_deg || angleLK < 15_deg || angleLK > 160_deg)
                continue;

              const float lImgLengthSqr = (lFirstImg - lLastImg).squaredNorm();
              if(lImgLengthSqr > (kFirstImg - kLastImg).squaredNorm() || lImgLengthSqr > (jFirstImg - jLastImg).squaredNorm())
                continue;

              LineCorrector::Line cLine4 = { lFirstImg, lLastImg, !customLineL.isEmpty() ? customLineL.firstField : Vector2f(), !customLineL.isEmpty() ? customLineL.lastField : Vector2f() };

              if(customLineL.isEmpty() && !theLineCorrector.fitLine(cLine4))
                continue;

              allRequiredFeaturesVisible = true;
              if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) continue;

              sampleFinished = true;
              const Geometry::Line line1(cLine1.aOnField, (cLine1.bOnField - cLine1.aOnField).normalized());
              const float distanceOther = Geometry::getDistanceToLineSigned(line1, (cLine4.aOnField + cLine4.bOnField) * 0.5f);
              const float combinedOffsetOther = distanceOther > 0 ? cLine1.offset - cLine4.offset : cLine4.offset - cLine1.offset;
              OUTPUT_TEXT("ParallelLinesDistance Short: " << std::abs(distanceOther) << ", CombinedOffset: " << combinedOffsetOther);
              ADD_SAMPLE(parallelLinesDistanceShort, ParallelLinesDistanceShortSample, cLine1, cLine4)
              ANNOTATION("AutomaticCameraCalibrator", "Sample Recorded (l): " << std::to_string(l));
              break;
            }
          }
          if(sampleFinished)
          {
            if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) continue;

            OUTPUT_TEXT("ParallelLinesDistance: " << std::abs(distance) << ", CombinedOffset: " << combinedOffset);
            ANNOTATION("AutomaticCameraCalibrator", "Sample Recorded (i, j, k): " << std::to_string(i) << " " << std::to_string(j) << " " << std::to_string(k));

            ADD_SAMPLE(cornerAngle, CornerAngleSample, cLine1, ((cLine2.bOnField - cLine2.aOnField).squaredNorm() <
                                                                (cLine3.bOnField - cLine3.aOnField).squaredNorm() ? cLine2 : cLine3))
            ADD_SAMPLE(parallelAngle, ParallelAngleSample, cLine2, cLine3)
            ADD_SAMPLE(parallelLinesDistance, ParallelLinesDistanceSample, cLine2, cLine3)
          }
          else
            discardedParallelLines = true;
        }
      }
    }
    if(theCalibrationRequest.sampleConfigurationRequest->doRecord)
    {
      INCREASE_RANGE("ParallelDisRange", discardedParallelLines, foundParallelLines, numOfDiscardedParallelLines, parallelLinesRange)
    }
  }

  if(thePenaltyMarkPercept.wasSeen && theLinesPercept.lines.size() >= 2 && theLinesPercept.lines.size() <= 8 &&
     (currentSampleConfiguration->needToRecord(samples, goalAreaDistance) || currentSampleConfiguration->needToRecord(samples, goalLineDistance)))
  {
    bool discardedGoalAreaLine = false, foundGoalAreaLine = false, discardedGoalLine = false, foundGoalLine = false;
    Rangef& goalAreaDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? goalAreaDisRangeUpper : goalAreaDisRangeLower;
    Rangef& goalLineDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? goalLineDisRangeUpper : goalLineDisRangeLower;

    for(size_t i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      for(size_t j = i + 1; j < theLinesPercept.lines.size(); ++j)
      {
        const LinesPercept::Line& lineI = !customLineI.isEmpty() ? customLineI : theLinesPercept.lines[i];
        const LinesPercept::Line& lineJ = !customLineJ.isEmpty() ? customLineJ : theLinesPercept.lines[j];

        // Heuristic: Goal line and front goal area line should span at least half the image width.
        if(std::abs(lineI.firstImg.x() - lineI.lastImg.x()) < theCameraInfo.width / 2 ||
           std::abs(lineJ.firstImg.x() - lineJ.lastImg.x()) < theCameraInfo.width / 2)
          continue;
        // Make sure the lines dont intersect
        if(Geometry::isPointLeftOfLine(lineI.firstField, lineI.lastField, lineJ.firstField) !=
           Geometry::isPointLeftOfLine(lineI.firstField, lineI.lastField, lineJ.lastField))
          continue;
        // Both lines should be behind the penalty mark (filter the front penalty area line)
        const float squaredDistanceToFirst = ((lineI.firstField + lineI.lastField) * 0.5f).squaredNorm();
        const float squaredDistanceToSecond = ((lineJ.firstField + lineJ.lastField) * 0.5f).squaredNorm();
        if(std::min(squaredDistanceToFirst, squaredDistanceToSecond) < thePenaltyMarkPercept.positionOnField.squaredNorm())
          continue;

        allRequiredFeaturesVisible = true;
        if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) continue;

        LineCorrector::Line cLineGoalArea = { lineI.firstImg.cast<float>(), lineI.lastImg.cast<float>(), Vector2f(), Vector2f()};
        LineCorrector::Line cLineGoalLine = { lineJ.firstImg.cast<float>(), lineJ.lastImg.cast<float>(), Vector2f(), Vector2f()};
        // Use the farther line as the goal line
        if(squaredDistanceToFirst > squaredDistanceToSecond)
          std::swap(cLineGoalArea, cLineGoalLine);
        if(!theLineCorrector.fitLine(cLineGoalArea) || !theLineCorrector.fitLine(cLineGoalLine))
          continue;
        float goalAreaLineDistance = Geometry::getDistanceToLine(Geometry::Line(cLineGoalArea.aOnField, (cLineGoalArea.bOnField - cLineGoalArea.aOnField).normalized()), thePenaltyMarkPercept.positionOnField);
        float goalLineDistance = Geometry::getDistanceToLine(Geometry::Line(cLineGoalLine.aOnField, (cLineGoalLine.bOnField - cLineGoalLine.aOnField).normalized()), thePenaltyMarkPercept.positionOnField);

        // Check if the found lines have a valid distance from the penalty spot.
        bool goalAreaLineDistanceValid, goalLineDistanceValid;
        (goalAreaLineDistanceValid = goalAreaDisRange.isInside(goalAreaLineDistance - cLineGoalArea.offset)) ? foundGoalAreaLine = true : discardedGoalAreaLine = true;
        (goalLineDistanceValid = goalLineDisRange.isInside(goalLineDistance - cLineGoalLine.offset)) ? foundGoalLine = true : discardedGoalLine = true;
        if(!goalAreaLineDistanceValid || !goalLineDistanceValid)
          continue;

        OUTPUT_TEXT("GoalAreaLineDistance: " << goalAreaLineDistance << ", Offset: " << cLineGoalArea.offset);
        OUTPUT_TEXT("GoalLineDistance: " << goalLineDistance << ", Offset: " << cLineGoalLine.offset);
        ANNOTATION("AutomaticCameraCalibrator", "Sample Recorded: " << std::to_string(i) << " " << std::to_string(j));

        ADD_SAMPLE(goalAreaDistance, GoalAreaDistanceSample, thePenaltyMarkPercept.positionInImage, cLineGoalArea)
        ADD_SAMPLE(SampleType::goalLineDistance, GoalLineDistanceSample, thePenaltyMarkPercept.positionInImage, cLineGoalLine)
        ADD_SAMPLE(parallelAngle, ParallelAngleSample, cLineGoalArea, cLineGoalLine)
        ADD_SAMPLE(parallelLinesDistance, ParallelLinesDistanceSample, cLineGoalArea, cLineGoalLine)
      }
    }
    if(theCalibrationRequest.sampleConfigurationRequest->doRecord)
    {
      INCREASE_RANGE("PenaltyLineRange", discardedGoalAreaLine, foundGoalAreaLine, numOfDiscardedGoalAreaLines, goalAreaDisRange)
      INCREASE_RANGE("GoalLineRange", discardedGoalLine, foundGoalLine, numOfDiscardedGoalLines, goalLineDisRange)
    }
  }

  // Make sure the head movement is registered.
  if(!currentSampleConfiguration->needToRecord(samples, cornerAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelLinesDistance) &&
     !currentSampleConfiguration->needToRecord(samples, goalAreaDistance) &&
     !currentSampleConfiguration->needToRecord(samples, goalLineDistance) &&
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
    optimizationParameters(lowerCameraRollCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    optimizationParameters(lowerCameraTiltCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    optimizationParameters(upperCameraRollCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    optimizationParameters(upperCameraTiltCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    optimizationParameters(bodyRollCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
    optimizationParameters(bodyTiltCorrection) = Random::uniform<float>(-0.5_deg, 0.5_deg);
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
  params(lowerCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x();
  params(lowerCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y();

  params(upperCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x();
  params(upperCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y();

  params(bodyRollCorrection) = cameraCalibration.bodyRotationCorrection.x();
  params(bodyTiltCorrection) = cameraCalibration.bodyRotationCorrection.y();

  return params;
}

void AutomaticCameraCalibrator::unpack(const Parameters& params, CameraCalibration& cameraCalibration) const
{
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x() = static_cast<float>(std::fmod(params(lowerCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() = static_cast<float>(std::fmod(params(lowerCameraTiltCorrection), 360_deg));

  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x() = static_cast<float>(std::fmod(params(upperCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y() = static_cast<float>(std::fmod(params(upperCameraTiltCorrection), 360_deg));

  cameraCalibration.bodyRotationCorrection.x() = static_cast<float>(std::fmod(params(bodyRollCorrection), 360_deg));
  cameraCalibration.bodyRotationCorrection.y() = static_cast<float>(std::fmod(params(bodyTiltCorrection), 360_deg));
}

void AutomaticCameraCalibrator::updateSampleConfiguration()
{
  if(!theCalibrationRequest.sampleConfigurationRequest)
  {
    return;
  }
  if(static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index) == lastSampleConfigurationIndex)
    return;

  const auto& request = theCalibrationRequest.sampleConfigurationRequest;

  currentSampleConfiguration = std::make_unique<SampleConfiguration>();
  currentSampleConfiguration->camera = request->camera;
  currentSampleConfiguration->headPan = request->headPan;
  currentSampleConfiguration->headTilt = request->headTilt;
  currentSampleConfiguration->sampleTypes = request->sampleTypes;
  currentSampleConfiguration->sampleIndexBase = numOfSamples;
  FOREACH_ENUM(SampleType, sampleType)
    if(request->sampleTypes & bit(sampleType))
      ++numOfSamples;

  lastSampleConfigurationIndex = static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index);
}

#define CHECK_LINE_PROJECTION(line, coordSys, camMat, camInf) \
  (Transformation::imageToRobot(coordSys.toCorrected(line.aInImage), camMat, camInf, line.aOnField) && \
   Transformation::imageToRobot(coordSys.toCorrected(line.bInImage), camMat, camInf, line.bOnField))

#define CHECK_PROJECTION_2_LINES(SampleName, line1, line2) \
  if(!CHECK_LINE_PROJECTION(line1, coordSys, cameraMatrix, cameraInfo) || \
     !CHECK_LINE_PROJECTION(line2, coordSys, cameraMatrix, cameraInfo)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

#define CHECK_PROJECTION_LINE_PENALTY(SampleName, line, p, p2) \
  if(!CHECK_LINE_PROJECTION(line, coordSys, cameraMatrix, cameraInfo) || \
     !Transformation::imageToRobot(coordSys.toCorrected(p), cameraMatrix, cameraInfo, p2)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

float AutomaticCameraCalibrator::Sample::computeError(const CameraCalibration& cameraCalibration) const
{
  const RobotCameraMatrix robotCameraMatrix(calibrator.theRobotDimensions, robotModel.limbs[Limbs::head], cameraCalibration, cameraInfo.camera);
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

float AutomaticCameraCalibrator::GoalAreaDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("GoalAreaDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField)

  const Geometry::Line line(cLine.aOnField, (cLine.bOnField - cLine.aOnField).normalized());
  const float goalAreaDistance = Geometry::getDistanceToLine(line, penaltyMarkOnField);
  const float goalAreaDistanceError = std::abs(goalAreaDistance - (calibrator.theFieldDimensions.xPosOpponentGoalArea -
                                               calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("PenaltyDistanceError: " << goalAreaDistanceError);
  return goalAreaDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibrator::GoalLineDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("GoalLineDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField)

  const Geometry::Line line(cLine.aOnField, (cLine.bOnField - cLine.aOnField).normalized());
  const float goalLineDistance = Geometry::getDistanceToLine(line, penaltyMarkOnField);
  const float goalLineDistanceError = std::abs(goalLineDistance - (calibrator.theFieldDimensions.xPosOpponentGoalLine -
                                               calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("GoalLineDistanceError: " << goalLineDistanceError);
  return goalLineDistanceError / calibrator.distanceErrorDivisor;
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
