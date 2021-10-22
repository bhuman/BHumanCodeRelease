/**
 * @file ExpAutomaticCameraCalibrator.cpp
 *
 * This file implements a module that provides an automatic camera calibration
 * based on the penalty area.
 *
 * @author Arne Hasselbring
 */

#include "ExpAutomaticCameraCalibrator.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(ExpAutomaticCameraCalibrator, infrastructure);

ExpAutomaticCameraCalibrator::ExpAutomaticCameraCalibrator() :
  state(State::idle),
  functor(*this)
{
  inStateSince = 0;
  lastSampleConfigurationIndex = -1;

  currentSampleConfiguration = nullptr;

  createLookUpTables();
  parallelDisRangeLower = parallelDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.fieldLinesWidth,
                                                         theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentGoalArea + theFieldDimensions.fieldLinesWidth);
  goalAreaDisRangeLower = goalAreaDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.fieldLinesWidth,
                                                         theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyMark + theFieldDimensions.fieldLinesWidth);
  groundLineDisRangeLower = groundLineDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.fieldLinesWidth,
                                                             theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentPenaltyMark + theFieldDimensions.fieldLinesWidth);

  // Load the cameraCalibration and cameraIntrinsics from disk, if it exists.
  loadModuleParameters(const_cast<CameraCalibration&>(theCameraCalibration), "CameraCalibration", nullptr);
  loadModuleParameters(const_cast<CameraIntrinsics&>(theCameraIntrinsics), "CameraIntrinsics", nullptr);
  nextCameraIntrinsics = theCameraIntrinsics;
  originalOpeningAngles = theCameraIntrinsics;
}

void ExpAutomaticCameraCalibrator::createLookUpTables()
{
  sinAngles.resize(numOfAngles);
  cosAngles.resize(numOfAngles);
  int index = 0;
  for(float deg = 0.f; index < numOfAngles; ++index, deg += 180_deg / numOfAngles)
  {
    cosAngles[index] = std::cos(deg);
    sinAngles[index] = std::sin(deg);
  }
}

float ExpAutomaticCameraCalibrator::calculateAngle(const Vector2f& lineAFirst, const Vector2f& lineASecond, const Vector2f& lineBFirst, const Vector2f& lineBSecond)
{
  const float dot = std::max(-1.f, std::min(static_cast<float>((lineAFirst - lineASecond).normalized().dot((lineBFirst - lineBSecond).normalized())), 1.f));
  return std::acos(dot);
}

void ExpAutomaticCameraCalibrator::update(CameraCalibration& cameraCalibration)
{
  DEBUG_DRAWING("module:ExpAutomaticCameraCalibrator:fieldLines", "drawingOnImage")
    THREAD("module:ExpAutomaticCameraCalibrator:fieldLines", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
  DEBUG_DRAWING("module:ExpAutomaticCameraCalibrator:correctedLines", "drawingOnImage")
    THREAD("module:ExpAutomaticCameraCalibrator:correctedLines", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");

  nextCameraCalibration = theCameraCalibration;
  updateSampleConfiguration();

  // Calibration start requested.
  if(state == State::idle && theCalibrationRequest.targetState == State::recordSamples)
  {
    optimizer = nullptr;
    successiveConvergences = 0;
    optimizationSteps = 0;
    samples.clear();
    nextCameraCalibration = theCameraCalibration;
    nextCameraIntrinsics = theCameraIntrinsics;

    state = State::recordSamples;
    inStateSince = theFrameInfo.time;
  }

  // Abort requested.
  if(theCalibrationRequest.targetState == State::idle && state != State::idle)
  {
    state = State::idle;
    inStateSince = theFrameInfo.time;
  }

  // TODO: It would be nice to trigger this action with the request.
  DEBUG_RESPONSE_ONCE("module:ExpAutomaticCameraCalibrator:converge")
  {
    if(optimizer && lowestDelta != std::numeric_limits<float>::max())
    {
      unpack(lowestDeltaParameters, nextCameraCalibration, nextCameraIntrinsics);
      OUTPUT_TEXT("ExpAutomaticCameraCalibrator: converged!");
      resetOptimization(true);
    }
  }

  if(state == State::recordSamples && currentSampleConfiguration && theCalibrationRequest.sampleConfigurationRequest)
  {
    recordSamples();
  }

  if(theCalibrationRequest.targetState == State::optimize && state != State::optimize)
  {
    state = State::optimize;
    inStateSince = theFrameInfo.time;
  }

  if(state == State::optimize)
    optimize();

  cameraCalibration = nextCameraCalibration;

  COMPLEX_DRAWING("module:ExpAutomaticCameraCalibrator:fieldLines")
    drawFieldLines();
}

#define CHECK_LINE_PROJECTION(line, otherCoordSys, camMat, camInf) \
  (Transformation::imageToRobot(otherCoordSys.toCorrected(line.aInImage), camMat, camInf, line.aOnField) && \
   Transformation::imageToRobot(otherCoordSys.toCorrected(line.bInImage), camMat, camInf, line.bOnField))

bool ExpAutomaticCameraCalibrator::fitLine(CorrectedLine& cline)
{
  Vector2f& correctedStart = cline.aInImage;
  Vector2f& correctedEnd = cline.bInImage;
  if(correctedEnd.x() < correctedStart.x()) std::swap(correctedStart, correctedEnd);

  // Determine the size of the image section to be processed
  const Vector2i mid = ((correctedStart + correctedEnd) * 0.5f).cast<int>();
  const int sizeX = ((std::max(32, static_cast<int>(correctedEnd.x() - correctedStart.x())) + 15) / 16) * 16;
  const int sizeY = std::max(32, std::abs(static_cast<int>(correctedEnd.y() - correctedStart.y())));
  const int startX = mid.x() - sizeX / 2, startY = mid.y() - sizeY / 2;

  // Extract the image patch and calculate the Sobel image
  Sobel::Image1D grayImage(sizeX, sizeY, sizeof(Sobel::Image1D::PixelType));
  extractImagePatch(Vector2i(startX, startY), Vector2i(sizeX, sizeY), grayImage);
  Sobel::SobelImage sobelImage(grayImage.width, grayImage.height);
  Sobel::sobelSSE(grayImage, sobelImage);

  // Since we know the approximate angle of the straight line, we only consider angles in this sector
  Vector2f dirLine = (correctedEnd.cast<float>() - correctedStart.cast<float>()).normalized();
  dirLine.rotateLeft();
  const float angle = std::fmod(static_cast<float>(atan2f(dirLine.y(), dirLine.x()) + 180_deg), static_cast<float>(180_deg));
  const float minAngle = std::fmod(static_cast<float>(Angle::normalize(angle - 10_deg) + 180_deg), static_cast<float>(180_deg));
  const float maxAngle = std::fmod(static_cast<float>(Angle::normalize(angle + 10_deg) + 180_deg), static_cast<float>(180_deg));
  const int minIndex = static_cast<int>(minAngle * numOfAngles / 180_deg) % numOfAngles;
  const int maxIndex = static_cast<int>(maxAngle * numOfAngles / 180_deg) % numOfAngles;

  // Calculate the values in the hough space
  const int dMax = static_cast<int>(std::ceil(std::hypot(sobelImage.height, sobelImage.width)));
  std::vector<std::vector<int> > houghSpace(numOfAngles, std::vector<int>(2 * dMax + 1, 0));
  calcHoughSpace(sobelImage, minIndex, maxIndex, dMax, houghSpace);

  // Determine the local maxima in the hough space
  std::vector<Maximum> localMaxima;
  determineLocalMaxima(houghSpace, minIndex, maxIndex, localMaxima);

  if(localMaxima.size() > 1)
  {
    // Calculate the corrected start/end of the upper or lower edge
    std::sort(localMaxima.begin(), localMaxima.end(), [](const Maximum& a, const Maximum& b) { return a.maxAcc > b.maxAcc; });
    int angle = localMaxima[0].angleIndex, distance = localMaxima[0].distanceIndex - dMax;
    Vector2f pointOnLine = Vector2f(distance * cosAngles[angle], distance * sinAngles[angle]) + Vector2f(startX, startY);
    Vector2f n0 = Vector2f(cosAngles[angle], sinAngles[angle]);

    Eigen::Hyperplane<float, 2> optimalLine = Eigen::Hyperplane<float, 2>(n0, pointOnLine);
    Vector2f norm = std::abs(correctedStart.x() - correctedEnd.x()) < std::abs(correctedStart.y() - correctedEnd.y()) ? Vector2f(0, 1) : Vector2f(1, 0);
    Eigen::Hyperplane<float, 2> lineStart = Eigen::Hyperplane<float, 2>(norm, correctedStart);
    Eigen::Hyperplane<float, 2> lineEnd = Eigen::Hyperplane<float, 2>(norm, correctedEnd);
    cline.aInImage = optimalLine.intersection(lineStart);
    cline.bInImage = optimalLine.intersection(lineEnd);

    // Check if we found the upper or lower edge and set the offset accordingly
    for(unsigned int i = 1; i < localMaxima.size(); ++i)
    {
      int angle = localMaxima[i].angleIndex, distance = localMaxima[i].distanceIndex - dMax;
      pointOnLine = Vector2f(distance * cosAngles[angle], distance * sinAngles[angle]) + Vector2f(startX, startY);
      n0 = Vector2f(cosAngles[angle], sinAngles[angle]);
      Eigen::Hyperplane<float, 2> oppositeOptimalLine = Eigen::Hyperplane<float, 2>(n0, pointOnLine);
      Vector2f startOpposite = oppositeOptimalLine.intersection(lineStart), endOpposite = oppositeOptimalLine.intersection(lineEnd);

      float disInImage = optimalLine.signedDistance((startOpposite + endOpposite) * 0.5f);
      if((optimalLine.signedDistance(startOpposite) < 0.f) != (optimalLine.signedDistance(endOpposite) < 0.f) ||
         optimalLine.absDistance(startOpposite) < minDisImage || optimalLine.absDistance(endOpposite) < minDisImage)
        continue;
      cline.offset = disInImage > 0.f ? theFieldDimensions.fieldLinesWidth / 2.f : -theFieldDimensions.fieldLinesWidth / 2.f;
      return CHECK_LINE_PROJECTION(cline, theImageCoordinateSystem, theCameraMatrix, theCameraInfo);
    }
  }
  return false;
}

void ExpAutomaticCameraCalibrator::update(CameraCalibrationStatus& cameraCalibrationStatus)
{
  cameraCalibrationStatus.state = state;
  cameraCalibrationStatus.inStateSince = inStateSince;

  cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::none;
  if(theCalibrationRequest.sampleConfigurationRequest)
  {
    cameraCalibrationStatus.sampleConfigurationStatus = allRequiredFeaturesVisible ? SampleConfigurationStatus::visible : SampleConfigurationStatus::notVisible;
    if(allRequiredFeaturesVisible && theCalibrationRequest.sampleConfigurationRequest->doRecord)
      cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::recording;
    updateSampleConfiguration();
    if(currentSampleConfiguration && isSampleConfigurationRecorded(currentSampleConfiguration->index))
      cameraCalibrationStatus.sampleConfigurationStatus = SampleConfigurationStatus::finished;
  }
}

void ExpAutomaticCameraCalibrator::update(CameraResolutionRequest& cameraResolutionRequest)
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
  {
    if(state == State::idle)
    {
      cameraResolutionRequest.resolutions[CameraInfo::lower] = CameraResolutionRequest::Resolutions::defaultRes;
      cameraResolutionRequest.resolutions[CameraInfo::upper] = CameraResolutionRequest::Resolutions::defaultRes;
    }
    else
    {
      cameraResolutionRequest.resolutions[CameraInfo::lower] = resRequest[CameraInfo::lower];
      cameraResolutionRequest.resolutions[CameraInfo::upper] = resRequest[CameraInfo::upper];
    }
  }
}

void ExpAutomaticCameraCalibrator::update(CameraIntrinsics& cameraIntrinsics)
{
  cameraIntrinsics = nextCameraIntrinsics;
}

void ExpAutomaticCameraCalibrator::extractImagePatch(const Vector2i& start, const Vector2i& size, Sobel::Image1D& grayImage)
{
  const ECImage& theECImage = theCameraInfo.camera == CameraInfo::upper ? static_cast<const ECImage&>(theUpperECImage) : static_cast<const ECImage&>(theLowerECImage);
  int currentY = start.y();
  for(int y = 0; y < size.y(); ++y, ++currentY)
  {
    if(currentY >= theCameraInfo.height || currentY < 0)
      continue;

    int currentX = start.x();
    const PixelTypes::GrayscaledPixel* pix = theECImage.grayscaled[currentY];
    for(int x = 0; x < size.x(); ++x, ++currentX)
    {
      if(currentX < theCameraInfo.width && currentX >= 0)
        grayImage[y][x] = *(pix + currentX);
    }
  }
}

float ExpAutomaticCameraCalibrator::determineSobelThresh(const Sobel::SobelImage& sobelImage)
{
  int thresh = 0;
  for(unsigned int y = 1; y < sobelImage.height - 1; ++y)
    for(unsigned int x = 1; x < sobelImage.width - 1; ++x)
    {
      const Sobel::SobelPixel& pixel = sobelImage[y][x];
      const int value = pixel.x * pixel.x + pixel.y * pixel.y;
      if(value > thresh) thresh = value;
    }
  return sqr(std::sqrt(static_cast<float>(thresh)) * sobelThreshValue);
}

void ExpAutomaticCameraCalibrator::calcHoughSpace(const Sobel::SobelImage& sobelImage, const int minIndex, const int maxIndex, const int dMax, std::vector<std::vector<int> >& houghSpace)
{
  const float thresh = determineSobelThresh(sobelImage);
  for(unsigned int y = 1; y < sobelImage.height - 1; ++y)
    for(unsigned int x = 1; x < sobelImage.width - 1; ++x)
    {
      const Sobel::SobelPixel& pixel = sobelImage[y][x];
      if(pixel.x * pixel.x + pixel.y * pixel.y >= thresh)
      {
        for(int index = minIndex; index != maxIndex; ++index)
        {
          int d = static_cast<int>(std::ceil(x * cosAngles[index] + y * sinAngles[index]));
          ++houghSpace[index][d + dMax];
          if(minIndex > maxIndex && index == numOfAngles - 1)
            index = -1;
        }
      }
    }
}

void ExpAutomaticCameraCalibrator::determineLocalMaxima(const std::vector<std::vector<int> >& houghSpace, const int minIndex, const int maxIndex, std::vector<Maximum>& localMaxima)
{
  const int maxDisIndex = static_cast<int>(houghSpace[0].size());
  auto localMaximum = [&houghSpace, maxDisIndex](const int value, const int angleIndex, const int distanceIndex, const int numOfAngles) -> bool
  {
    for(int i = -1; i <= 1; ++i)
    {
      int index = ((angleIndex + i) + numOfAngles) % numOfAngles;
      for(int j = std::max(0, distanceIndex - 1); j <= std::min(maxDisIndex - 1, distanceIndex + 1); ++j)
      {
        if(index == angleIndex && j == distanceIndex)
          continue;
        if(houghSpace[index][j] > value)
          return false;
      }
    }
    return true;
  };

  for(int angleIndex = minIndex; angleIndex != maxIndex; ++angleIndex)
  {
    for(int distanceIndex = 0; distanceIndex < maxDisIndex; ++distanceIndex)
    {
      int value = houghSpace[angleIndex][distanceIndex];
      if(value != 0 && localMaximum(value, angleIndex, distanceIndex, numOfAngles))
        localMaxima.push_back({value, angleIndex, distanceIndex});
    }
    if(minIndex > maxIndex && angleIndex == numOfAngles - 1)
      angleIndex = -1;
  }
}

#define ADD_SAMPLE(recordedSamples, sampleType, SampleName, first, second) \
  if(currentSampleConfiguration->containsSampleType(sampleType)) \
  { \
    auto s = std::make_unique<SampleName>(*this, theTorsoMatrix, theJointAngles.angles[Joints::headYaw], theJointAngles.angles[Joints::headPitch], theCameraInfo, theImageCoordinateSystem, first, second); \
    recordedSamples.push_back(std::move(s)); \
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

void ExpAutomaticCameraCalibrator::recordSamples()
{
  // We already recorded samples for this configuration, so we are done here.
  if(isSampleConfigurationRecorded(currentSampleConfiguration->index))
    return;

  if(theCameraInfo.camera != currentSampleConfiguration->camera)
    return;

  COMPLEX_DRAWING("module:ExpAutomaticCameraCalibrator:correctedLines")
  {
    for(unsigned int i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      CorrectedLine cLine = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
      if(fitLine(cLine))
      {
        const Vector2f cLineMid = (cLine.aInImage + cLine.bInImage) * 0.5;
        DRAW_TEXT("module:ExpAutomaticCameraCalibrator:correctedLines", cLineMid.x(), cLineMid.y(), 10, ColorRGBA::black, cLine.offset);
        CROSS("module:ExpAutomaticCameraCalibrator:correctedLines", cLine.aInImage.x(), cLine.aInImage.y(), 4, 2, Drawings::solidPen, ColorRGBA::red);
        CROSS("module:ExpAutomaticCameraCalibrator:correctedLines", cLine.bInImage.x(), cLine.bInImage.y(), 4, 2, Drawings::solidPen, ColorRGBA::black);
        LINE("module:ExpAutomaticCameraCalibrator:correctedLines", cLine.aInImage.x(), cLine.aInImage.y(), cLine.bInImage.x(), cLine.bInImage.y(), 1, Drawings::solidPen, ColorRGBA::yellow);
      }
    }
  }


  allRequiredFeaturesVisible = false;
  if(theLinesPercept.lines.size() >= 3 && theLinesPercept.lines.size() <= 8 &&
     (currentSampleConfiguration->containsSampleType(cornerAngle) || currentSampleConfiguration->containsSampleType(parallelAngle) || currentSampleConfiguration->containsSampleType(parallelLinesDistance)) &&
     !(currentSampleConfiguration->containsSampleType(goalAreaDistance) || currentSampleConfiguration->containsSampleType(groundLineDistance)))
  {
    bool discardedParallelLines = false, foundParallelLines = false;
    Rangef& parallelLinesRange = currentSampleConfiguration->camera == CameraInfo::upper ? parallelDisRangeUpper : parallelDisRangeLower;
    for(size_t i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      for(size_t j = 0; j < theLinesPercept.lines.size(); ++j)
      {
        if(i == j)
          continue;
        for(size_t k = j + 1; k < theLinesPercept.lines.size(); ++k)
        {
          if(i == k)
            continue;
          // i is the index of the "short" connecting line, j and k are the indices for the orthogonal lines (ground line and front goal area line).
          // It is checked whether the line i has one end in one line and the other end in the other line.
          // TODO: This should be done in image coordinates because otherwise the camera would have to be calibrated. But still works.
          const float distInImage1 = std::min(Geometry::getDistanceToEdge(theLinesPercept.lines[j].line, theLinesPercept.lines[i].firstField.cast<float>()),
                                              Geometry::getDistanceToEdge(theLinesPercept.lines[j].line, theLinesPercept.lines[i].lastField.cast<float>()));
          if(distInImage1 > 100.f)
            continue;
          const float distInImage2 = std::min(Geometry::getDistanceToEdge(theLinesPercept.lines[k].line, theLinesPercept.lines[i].firstField.cast<float>()),
                                              Geometry::getDistanceToEdge(theLinesPercept.lines[k].line, theLinesPercept.lines[i].lastField.cast<float>()));
          if(distInImage2 > 100.f)
            continue;

          // Make sure that the short connecting line is the closest as seen from the robot.
          const float squaredDistanceToShortLine = ((theLinesPercept.lines[i].firstField + theLinesPercept.lines[i].lastField) * 0.5f).squaredNorm();
          if(((theLinesPercept.lines[j].firstField + theLinesPercept.lines[j].lastField) * 0.5f).squaredNorm() < squaredDistanceToShortLine ||
             ((theLinesPercept.lines[k].firstField + theLinesPercept.lines[k].lastField) * 0.5f).squaredNorm() < squaredDistanceToShortLine)
            continue;

          // Roughly check whether the angles are reasonable (in image coordinates)
          const Angle angleIJ = calculateAngle(theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(),
                                               theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>());
          const Angle angleIK = calculateAngle(theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(),
                                               theLinesPercept.lines[k].firstImg.cast<float>(), theLinesPercept.lines[k].lastImg.cast<float>());
          const Angle angleJK = calculateAngle(theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>(),
                                               theLinesPercept.lines[k].firstImg.cast<float>(), theLinesPercept.lines[k].lastImg.cast<float>());
          if(angleIJ < 20_deg || angleIJ > 160_deg || angleIK < 20_deg || angleIK > 160_deg || angleJK > 40_deg)
            continue;

          allRequiredFeaturesVisible = true;
          if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) continue;

          // Fit lines through the start/end points of the lines
          CorrectedLine cLine1 = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
          CorrectedLine cLine2 = {theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>(), Vector2f(), Vector2f()};
          CorrectedLine cLine3 = {theLinesPercept.lines[k].firstImg.cast<float>(), theLinesPercept.lines[k].lastImg.cast<float>(), Vector2f(), Vector2f()};
          if(!fitLine(cLine1) || !fitLine(cLine2) || !fitLine(cLine3))
            continue;

          const Geometry::Line line2(cLine2.aOnField, (cLine2.bOnField - cLine2.aOnField).normalized());
          const float distance = Geometry::getDistanceToLine(line2, (cLine3.aOnField + cLine3.bOnField) * 0.5f);
          const float combinedOffset = distance > 0 ? cLine2.offset - cLine3.offset : cLine3.offset - cLine2.offset;

          if(parallelLinesRange.isInside(std::abs(distance) - combinedOffset))
          {
            foundParallelLines = true;

            OUTPUT_TEXT("ParallelLinesDistance: " << std::abs(distance) << ", CombinedOffset: " << combinedOffset);
            ANNOTATION("ExpAutomaticCameraCalibrator", "Sample Recorded: " << std::to_string(i) << " " << std::to_string(j) << " " << std::to_string(k));

            std::vector<std::unique_ptr<Sample>> recordedSamples;
            // Use the longer line as orthogonal line.
            ADD_SAMPLE(recordedSamples, cornerAngle, CornerAngleSample, cLine1, ((cLine2.bOnField - cLine2.aOnField).squaredNorm() <
                                                                (cLine3.bOnField - cLine3.aOnField).squaredNorm() ? cLine2 : cLine3))
            ADD_SAMPLE(recordedSamples, parallelAngle, ParallelAngleSample, cLine2, cLine3)
            ADD_SAMPLE(recordedSamples, parallelLinesDistance, ParallelLinesDistanceSample, cLine2, cLine3)

            recordSampleConfiguration(*currentSampleConfiguration, recordedSamples);
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
     (currentSampleConfiguration->containsSampleType(goalAreaDistance) || currentSampleConfiguration->containsSampleType(groundLineDistance)))
  {
    bool discardedGoalAreaLine = false, foundGoalAreaLine = false, discardedGroundLine = false, foundGroundLine = false;
    Rangef& goalAreaDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? goalAreaDisRangeUpper : goalAreaDisRangeLower;
    Rangef& groundLineDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? groundLineDisRangeUpper : groundLineDisRangeLower;

    for(size_t i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      for(size_t j = i + 1; j < theLinesPercept.lines.size(); ++j)
      {
        // Heuristic: Ground line and front goal area line should span at least half the image width.
        if(std::abs(theLinesPercept.lines[i].firstImg.x() - theLinesPercept.lines[i].lastImg.x()) < theCameraInfo.width / 2 ||
           std::abs(theLinesPercept.lines[j].firstImg.x() - theLinesPercept.lines[j].lastImg.x()) < theCameraInfo.width / 2)
          continue;
        // Make sure the lines dont intersect
        if(Geometry::isPointLeftOfLine(theLinesPercept.lines[i].firstField, theLinesPercept.lines[i].lastField, theLinesPercept.lines[j].firstField) !=
           Geometry::isPointLeftOfLine(theLinesPercept.lines[i].firstField, theLinesPercept.lines[i].lastField, theLinesPercept.lines[j].lastField))
          continue;
        // Both lines should be behind the penalty mark (filter the front penalty area line)
        const float squaredDistanceToFirst = ((theLinesPercept.lines[i].firstField + theLinesPercept.lines[i].lastField) * 0.5f).squaredNorm();
        const float squaredDistanceToSecond = ((theLinesPercept.lines[j].firstField + theLinesPercept.lines[j].lastField) * 0.5f).squaredNorm();
        if(std::min(squaredDistanceToFirst, squaredDistanceToSecond) < thePenaltyMarkPercept.positionOnField.squaredNorm())
          continue;

        allRequiredFeaturesVisible = true;
        if(!theCalibrationRequest.sampleConfigurationRequest->doRecord) continue;

        CorrectedLine cLineGoalArea = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
        CorrectedLine cLineGroundLine = {theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>(), Vector2f(), Vector2f()};
        // Use the farther line as the ground line
        if(squaredDistanceToFirst > squaredDistanceToSecond)
          std::swap(cLineGoalArea, cLineGroundLine);
        if(!fitLine(cLineGoalArea) || !fitLine(cLineGroundLine))
          continue;
        float goalAreaLineDistance = std::abs(Geometry::getDistanceToLine(Geometry::Line(cLineGoalArea.aOnField, (cLineGoalArea.bOnField - cLineGoalArea.aOnField).normalized()), thePenaltyMarkPercept.positionOnField));
        float groundLineDistance = std::abs(Geometry::getDistanceToLine(Geometry::Line(cLineGroundLine.aOnField, (cLineGroundLine.bOnField - cLineGroundLine.aOnField).normalized()), thePenaltyMarkPercept.positionOnField));

        // Check if the found lines have a valid distance from the penalty spot.
        bool goalAreaLineDistanceValid, groundLineDistanceValid;
        (goalAreaLineDistanceValid = goalAreaDisRange.isInside(goalAreaLineDistance - cLineGoalArea.offset)) ? foundGoalAreaLine = true : discardedGoalAreaLine = true;
        (groundLineDistanceValid = groundLineDisRange.isInside(groundLineDistance - cLineGroundLine.offset)) ? foundGroundLine = true : discardedGroundLine = true;
        if(!goalAreaLineDistanceValid || !groundLineDistanceValid)
          continue;

        OUTPUT_TEXT("GoalAreaLineDistance: " << goalAreaLineDistance << ", Offset: " << cLineGoalArea.offset);
        OUTPUT_TEXT("GroundLineDistance: " << groundLineDistance << ", Offset: " << cLineGroundLine.offset);
        ANNOTATION("ExpAutomaticCameraCalibrator", "Sample Recorded: " << std::to_string(i) << " " << std::to_string(j));

        std::vector<std::unique_ptr<Sample>> recordedSamples;

        ADD_SAMPLE(recordedSamples, goalAreaDistance, GoalAreaDistanceSample, thePenaltyMarkPercept.positionInImage, cLineGoalArea)
        ADD_SAMPLE(recordedSamples, SampleType::groundLineDistance, GroundLineDistanceSample, thePenaltyMarkPercept.positionInImage, cLineGroundLine)
        ADD_SAMPLE(recordedSamples, parallelAngle, ParallelAngleSample, cLineGoalArea, cLineGroundLine)
        ADD_SAMPLE(recordedSamples, parallelLinesDistance, ParallelLinesDistanceSample, cLineGoalArea, cLineGroundLine)

        recordSampleConfiguration(*currentSampleConfiguration, recordedSamples);
      }
    }
    if(theCalibrationRequest.sampleConfigurationRequest->doRecord)
    {
      INCREASE_RANGE("PenaltyLineRange", discardedGoalAreaLine, foundGoalAreaLine, numOfDiscardedGoalAreaLines, goalAreaDisRange)
      INCREASE_RANGE("GroundLineRange", discardedGroundLine, foundGroundLine, numOfDiscardedGroundLines, groundLineDisRange)
    }
  }

  // TODO: check if we still need this.
  // Make sure the head movement is registered.
  if(!currentSampleConfiguration->containsSampleType(cornerAngle) &&
     !currentSampleConfiguration->containsSampleType(parallelAngle) &&
     !currentSampleConfiguration->containsSampleType(parallelLinesDistance) &&
     !currentSampleConfiguration->containsSampleType(goalAreaDistance) &&
     !currentSampleConfiguration->containsSampleType(groundLineDistance))
  {
    numOfDiscardedParallelLines = numOfDiscardedGoalAreaLines = numOfDiscardedGroundLines = 0;
    allRequiredFeaturesVisible = true;
  }
}

void ExpAutomaticCameraCalibrator::optimize()
{
  if(!optimizer)
  {
    optimizer = std::make_unique<GaussNewtonOptimizer<numOfParameterTranslations>>(functor);
    optimizationParameters = pack(theCameraCalibration, theCameraIntrinsics);
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

    OUTPUT_TEXT("ExpAutomaticCameraCalibrator: delta = " << delta << "\n");
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
      OUTPUT_TEXT("ExpAutomaticCameraCalibrator: converged!");
      unpack(lowestErrorParameters, nextCameraCalibration, nextCameraIntrinsics);
      resetOptimization(true);
    }
    else
      unpack(optimizationParameters, nextCameraCalibration, nextCameraIntrinsics);
  }
}

void ExpAutomaticCameraCalibrator::resetOptimization(const bool finished)
{
  if(finished)
    state = State::idle;
  else
  {
    OUTPUT_TEXT("Restart optimize! An optimization error occurred!");
    optimizationParameters(lowerCameraRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(lowerCameraTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(upperCameraRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(upperCameraTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(bodyRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(bodyTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(lowerOpeningHeight) = originalOpeningAngles.cameras[CameraInfo::lower].openingAngleHeight;
    optimizationParameters(lowerOpeningWidth) = originalOpeningAngles.cameras[CameraInfo::lower].openingAngleWidth;
    optimizationParameters(upperOpeningHeight) = originalOpeningAngles.cameras[CameraInfo::upper].openingAngleHeight;
    optimizationParameters(upperOpeningWidth) = originalOpeningAngles.cameras[CameraInfo::upper].openingAngleWidth;
    unpack(optimizationParameters, nextCameraCalibration, nextCameraIntrinsics);
  }

  optimizer = nullptr;
  lowestDelta = std::numeric_limits<float>::max();
  lowestDeltaParameters = Parameters();
  optimizationSteps = 0;
}

ExpAutomaticCameraCalibrator::Parameters ExpAutomaticCameraCalibrator::pack(const CameraCalibration& cameraCalibration, const CameraIntrinsics& cameraIntrinsics) const
{
  Parameters params;
  params(lowerCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x();
  params(lowerCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y();

  params(upperCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x();
  params(upperCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y();

  params(bodyRollCorrection) = cameraCalibration.bodyRotationCorrection.x();
  params(bodyTiltCorrection) = cameraCalibration.bodyRotationCorrection.y();

  params(lowerOpeningHeight) = cameraIntrinsics.cameras[CameraInfo::lower].openingAngleHeight;
  params(lowerOpeningWidth) = cameraIntrinsics.cameras[CameraInfo::lower].openingAngleWidth;
  params(upperOpeningHeight) = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleHeight;
  params(upperOpeningWidth) = cameraIntrinsics.cameras[CameraInfo::upper].openingAngleWidth;

  return params;
}

void ExpAutomaticCameraCalibrator::unpack(const Parameters& params, CameraCalibration& cameraCalibration, CameraIntrinsics& cameraIntrinsics) const
{
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x() = static_cast<float>(std::fmod(params(lowerCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() = static_cast<float>(std::fmod(params(lowerCameraTiltCorrection), 360_deg));

  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x() = static_cast<float>(std::fmod(params(upperCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y() = static_cast<float>(std::fmod(params(upperCameraTiltCorrection), 360_deg));

  cameraCalibration.bodyRotationCorrection.x() = static_cast<float>(std::fmod(params(bodyRollCorrection), 360_deg));
  cameraCalibration.bodyRotationCorrection.y() = static_cast<float>(std::fmod(params(bodyTiltCorrection), 360_deg));

  // only adjust up to maxOpeningAngleAdjustment for the intrinsics
  const Rangea rangeOpeningHeightLower(originalOpeningAngles.cameras[CameraInfo::lower].openingAngleHeight - maxOpeningAngleAdjustment, originalOpeningAngles.cameras[CameraInfo::lower].openingAngleHeight + maxOpeningAngleAdjustment);
  const Rangea rangeOpeningWidthLower(originalOpeningAngles.cameras[CameraInfo::lower].openingAngleWidth - maxOpeningAngleAdjustment, originalOpeningAngles.cameras[CameraInfo::lower].openingAngleWidth + maxOpeningAngleAdjustment);
  const Rangea rangeOpeningHeightUpper(originalOpeningAngles.cameras[CameraInfo::upper].openingAngleHeight - maxOpeningAngleAdjustment, originalOpeningAngles.cameras[CameraInfo::upper].openingAngleHeight + maxOpeningAngleAdjustment);
  const Rangea rangeOpeningWidthUpper(originalOpeningAngles.cameras[CameraInfo::upper].openingAngleWidth - maxOpeningAngleAdjustment, originalOpeningAngles.cameras[CameraInfo::upper].openingAngleWidth + maxOpeningAngleAdjustment);
  cameraIntrinsics.cameras[CameraInfo::lower].openingAngleHeight = rangeOpeningHeightLower.limit(static_cast<float>(std::fmod(params(lowerOpeningHeight), 360_deg)));
  cameraIntrinsics.cameras[CameraInfo::lower].openingAngleWidth = rangeOpeningWidthLower.limit(static_cast<float>(std::fmod(params(lowerOpeningWidth), 360_deg)));
  cameraIntrinsics.cameras[CameraInfo::upper].openingAngleHeight = rangeOpeningHeightUpper.limit(static_cast<float>(std::fmod(params(upperOpeningHeight), 360_deg)));
  cameraIntrinsics.cameras[CameraInfo::upper].openingAngleWidth = rangeOpeningWidthUpper.limit(static_cast<float>(std::fmod(params(upperOpeningWidth), 360_deg)));
}

void ExpAutomaticCameraCalibrator::updateSampleConfiguration()
{
  if(!theCalibrationRequest.sampleConfigurationRequest)
  {
    return;
  }
  if(static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index) == lastSampleConfigurationIndex)
    return;

  const auto& request = theCalibrationRequest.sampleConfigurationRequest;

  currentSampleConfiguration = std::make_unique<SampleConfiguration>();
  currentSampleConfiguration->index = request->index;
  currentSampleConfiguration->camera = request->camera;
  currentSampleConfiguration->headPan = request->headPan;
  currentSampleConfiguration->headTilt = request->headTilt;
  currentSampleConfiguration->sampleTypes = request->sampleTypes;

  lastSampleConfigurationIndex = static_cast<int>(theCalibrationRequest.sampleConfigurationRequest->index);
}

#define CHECK_PROJECTION_2_LINES(SampleName, line1, line2, otherCoordSys, otherCameraInfo) \
  if(!CHECK_LINE_PROJECTION(line1, otherCoordSys, cameraMatrix, otherCameraInfo) || \
     !CHECK_LINE_PROJECTION(line2, otherCoordSys, cameraMatrix, otherCameraInfo)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

#define CHECK_PROJECTION_LINE_PENALTY(SampleName, line, p, p2, otherCoordSys, otherCameraInfo) \
  if(!CHECK_LINE_PROJECTION(line, otherCoordSys, cameraMatrix, otherCameraInfo) || \
     !Transformation::imageToRobot(otherCoordSys.toCorrected(p), cameraMatrix, otherCameraInfo, p2)) \
  { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
  }

float ExpAutomaticCameraCalibrator::Sample::computeError(const CameraCalibration& cameraCalibration, const CameraInfo& cameraInfo) const
{
  const RobotCameraMatrix robotCameraMatrix(calibrator.theRobotDimensions, headYaw, headPitch, cameraCalibration, cameraInfo.camera);
  const CameraMatrix cameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
  return computeError(cameraMatrix, cameraInfo);
}

float ExpAutomaticCameraCalibrator::CornerAngleSample::computeError(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const
{
  CameraInfo otherCameraInfo = this->cameraInfo;
  otherCameraInfo.openingAngleHeight = cameraInfo.openingAngleHeight;
  otherCameraInfo.openingAngleWidth = cameraInfo.openingAngleWidth;
  otherCameraInfo.updateFocalLength();
  ImageCoordinateSystem otherCoordSys = this->coordSys;
  otherCoordSys.cameraInfo = otherCameraInfo;
  CHECK_PROJECTION_2_LINES("CornerAngleSample", cLine1, cLine2, this->coordSys, this->cameraInfo)

  const float cornerAngle = calculateAngle(cLine1.aOnField, cLine1.bOnField, cLine2.aOnField, cLine2.bOnField);
  const float cornerAngleError = std::abs(90_deg - cornerAngle);
  OUTPUT_TEXT("Angle 90: " << cornerAngle << ", error: " << cornerAngleError);
  return cornerAngleError / calibrator.angleErrorDivisor;
}

float ExpAutomaticCameraCalibrator::ParallelAngleSample::computeError(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const
{
  CameraInfo otherCameraInfo = this->cameraInfo;
  otherCameraInfo.openingAngleHeight = cameraInfo.openingAngleHeight;
  otherCameraInfo.openingAngleWidth = cameraInfo.openingAngleWidth;
  otherCameraInfo.updateFocalLength();
  ImageCoordinateSystem otherCoordSys = this->coordSys;
  otherCoordSys.cameraInfo = otherCameraInfo;
  CHECK_PROJECTION_2_LINES("ParallelAngleSample", cLine1, cLine2, otherCoordSys, otherCameraInfo)

  const float parallelAngle = calculateAngle(cLine1.aOnField, cLine1.bOnField, cLine2.aOnField, cLine2.bOnField);
  const float parallelAngleError = std::min(parallelAngle, 180_deg - parallelAngle);
  OUTPUT_TEXT("Angle 180: " << parallelAngle << ", error: " << parallelAngleError);
  return parallelAngleError / calibrator.angleErrorDivisor;
}

float ExpAutomaticCameraCalibrator::ParallelLinesDistanceSample::computeError(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const
{
  CameraInfo otherCameraInfo = this->cameraInfo;
  otherCameraInfo.openingAngleHeight = cameraInfo.openingAngleHeight;
  otherCameraInfo.openingAngleWidth = cameraInfo.openingAngleWidth;
  otherCameraInfo.updateFocalLength();
  ImageCoordinateSystem otherCoordSys = this->coordSys;
  otherCoordSys.cameraInfo = otherCameraInfo;
  CHECK_PROJECTION_2_LINES("ParallelLinesDistanceSample", cLine1, cLine2, otherCoordSys, otherCameraInfo)

  const Geometry::Line line1(cLine1.aOnField, (cLine1.bOnField - cLine1.aOnField).normalized());
  const Geometry::Line line2(cLine2.aOnField, (cLine2.bOnField - cLine2.aOnField).normalized());
  const float distance1 = Geometry::getDistanceToLine(line1, cLine2.aOnField);
  const float distance2 = Geometry::getDistanceToLine(line1, cLine2.bOnField);

  const float distance3 = Geometry::getDistanceToLine(line2, cLine1.aOnField);
  const float distance4 = Geometry::getDistanceToLine(line2, cLine1.bOnField);

  const float distance1ErrorRange = cLine2.aOnField.norm() / 1000.f * calibrator.pixelInaccuracyPerMeter;
  const float distance2ErrorRange = cLine2.bOnField.norm() / 1000.f * calibrator.pixelInaccuracyPerMeter;

  const float distance3ErrorRange = cLine1.aOnField.norm() / 1000.f * calibrator.pixelInaccuracyPerMeter;
  const float distance4ErrorRange = cLine1.bOnField.norm() / 1000.f * calibrator.pixelInaccuracyPerMeter;

  const float combinedOffset = distance1 > 0 ? cLine1.offset - cLine2.offset : cLine2.offset - cLine1.offset;

  const float optimalDistance = calibrator.theFieldDimensions.xPosOpponentGroundLine - calibrator.theFieldDimensions.xPosOpponentGoalArea + combinedOffset;
  std::vector<float> distanceErrorList;
  distanceErrorList.push_back(std::max(0.f, (std::abs(std::abs(distance1) - optimalDistance) - distance1ErrorRange)));
  distanceErrorList.push_back(std::max(0.f, (std::abs(std::abs(distance2) - optimalDistance) - distance2ErrorRange)));
  distanceErrorList.push_back(std::max(0.f, (std::abs(std::abs(distance3) - optimalDistance) - distance3ErrorRange)));
  distanceErrorList.push_back(std::max(0.f, (std::abs(std::abs(distance4) - optimalDistance) - distance4ErrorRange)));
  const float lineDistanceError = *std::max_element(distanceErrorList.cbegin(), distanceErrorList.cend());
  OUTPUT_TEXT("LineDistanceError: " << lineDistanceError);
  return lineDistanceError / calibrator.distanceErrorDivisor;
}

float ExpAutomaticCameraCalibrator::GoalAreaDistanceSample::computeError(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const
{
  CameraInfo otherCameraInfo = this->cameraInfo;
  otherCameraInfo.openingAngleHeight = cameraInfo.openingAngleHeight;
  otherCameraInfo.openingAngleWidth = cameraInfo.openingAngleWidth;
  otherCameraInfo.updateFocalLength();
  ImageCoordinateSystem otherCoordSys = this->coordSys;
  otherCoordSys.cameraInfo = otherCameraInfo;
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("GoalAreaDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField, otherCoordSys, otherCameraInfo)

  const Geometry::Line line(cLine.aOnField, (cLine.bOnField - cLine.aOnField).normalized());
  const float goalAreaDistance = std::abs(Geometry::getDistanceToLine(line, penaltyMarkOnField));
  const float goalAreaDistanceError = std::abs(goalAreaDistance - (calibrator.theFieldDimensions.xPosOpponentGoalArea -
                                               calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("PenaltyDistanceError: " << goalAreaDistanceError);
  return goalAreaDistanceError / calibrator.distanceErrorDivisor;
}

float ExpAutomaticCameraCalibrator::GroundLineDistanceSample::computeError(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const
{
  CameraInfo otherCameraInfo = this->cameraInfo;
  otherCameraInfo.openingAngleHeight = cameraInfo.openingAngleHeight;
  otherCameraInfo.openingAngleWidth = cameraInfo.openingAngleWidth;
  otherCameraInfo.updateFocalLength();
  ImageCoordinateSystem otherCoordSys = this->coordSys;
  otherCoordSys.cameraInfo = otherCameraInfo;
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("GroundLineDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField, otherCoordSys, otherCameraInfo)

  const Geometry::Line line(cLine.aOnField, (cLine.bOnField - cLine.aOnField).normalized());
  const float groundLineDistance = std::abs(Geometry::getDistanceToLine(line, penaltyMarkOnField));
  const float groundLineDistanceError = std::abs(groundLineDistance - (calibrator.theFieldDimensions.xPosOpponentGroundLine -
                                                 calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("GroundLineDistanceError: " << groundLineDistanceError);
  return groundLineDistanceError / calibrator.distanceErrorDivisor;
}

float ExpAutomaticCameraCalibrator::Functor::operator()(const Parameters& params, size_t measurement) const
{
  CameraCalibration cameraCalibration = calibrator.nextCameraCalibration;
  CameraInfo cameraInfo = calibrator.samples[measurement]->cameraInfo;
  CameraIntrinsics cameraIntrinsics = calibrator.nextCameraIntrinsics;
  calibrator.unpack(params, cameraCalibration, cameraIntrinsics);
  cameraInfo.openingAngleHeight = cameraIntrinsics.cameras[cameraInfo.camera].openingAngleHeight;
  cameraInfo.openingAngleWidth = cameraIntrinsics.cameras[cameraInfo.camera].openingAngleWidth;
  return calibrator.samples[measurement]->computeError(cameraCalibration, cameraInfo);
}

bool ExpAutomaticCameraCalibrator::SampleConfiguration::containsSampleType(SampleType sampleType) const
{
  return sampleTypes & bit(sampleType);
}


bool ExpAutomaticCameraCalibrator::isSampleConfigurationRecorded(const SampleConfigId& index) const
{
  return recordedSampleConfigurations.find(index) != recordedSampleConfigurations.end();
}

void ExpAutomaticCameraCalibrator::recordSampleConfiguration(const SampleConfiguration &sampleConfiguration, std::vector<std::unique_ptr<Sample>>& recordedSamples)
{
  // Check that the all required sample types have been recorded.
  unsigned recordedSampleTypes = 0;
  for(const auto& sample : recordedSamples)
    recordedSampleTypes |= bit(sample->sampleType);
  ASSERT(sampleConfiguration.sampleTypes == recordedSampleTypes);

  // Append the recorded samples to the array of all recorded samples.
  samples.insert(samples.end(),
                 std::make_move_iterator(recordedSamples.begin()),
                 std::make_move_iterator(recordedSamples.end()));

  // Mark the sample configuration as recorded.
  recordedSampleConfigurations.insert(sampleConfiguration.index);
  OUTPUT_TEXT("Saving " << std::to_string(recordedSamples.size()) << " recorded samples for sample configuration #" << std::to_string(sampleConfiguration.index));
}

void ExpAutomaticCameraCalibrator::drawFieldLines() const
{
  const Pose2f robotPoseInv = validationRobotPose.inverse();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from), theCameraMatrix, theCameraInfo, lineInImage))
      LINE("module:ExpAutomaticCameraCalibrator:fieldLines", lineInImage.base.x(), lineInImage.base.y(), (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1, Drawings::solidPen, ColorRGBA::black);
  }
}

bool ExpAutomaticCameraCalibrator::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
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
