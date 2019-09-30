/**
 * @file AutomaticCameraCalibratorPA.cpp
 *
 * This file implements a module that provides an automatic camera calibration
 * based on the penalty area.
 *
 * @author Arne Hasselbring (based on the old AutomaticCameraCalibrator, written by its authors)
 * @author Andreas Baude
 */

#include "AutomaticCameraCalibratorPA.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(AutomaticCameraCalibratorPA, infrastructure);

AutomaticCameraCalibratorPA::AutomaticCameraCalibratorPA() :
  state(Idle),
  functor(*this)
{
  numOfSamples = 0;
  ASSERT(headPansUpper.size() == headPansLower.size());
  for(size_t i = 0; i < headPansUpper.size(); ++i)
  {
    addSampleConfiguration(CameraInfo::upper, headPansUpper[i], headTiltUpper, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));
    addSampleConfiguration(CameraInfo::lower, headPansLower[i], headTiltLower, bit(cornerAngle) | bit(parallelAngle) | bit(parallelLinesDistance));
  }
  for(size_t i = 0; i < headPansUpper.size(); ++i)
  {
    addSampleConfiguration(CameraInfo::upper, headPansUpper[i], headTiltUpper, bit(penaltyAreaDistance) | bit(groundlineDistance) | bit(parallelAngle));
    addSampleConfiguration(CameraInfo::lower, headPansLower[i], headTiltLower, bit(penaltyAreaDistance) | bit(groundlineDistance) | bit(parallelAngle));
  }
  ASSERT(!sampleConfigurations.empty());
  currentSampleConfiguration = sampleConfigurations.begin();
  createLookUpTables();
  parallelDisRangeLower = parallelDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea - 50.f,
                                                         theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea + 50.f);
  penaltyDisRangeLower = penaltyDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentPenaltyArea - theFieldDimensions.xPosOpponentPenaltyMark - 50.f,
                                                       theFieldDimensions.xPosOpponentPenaltyArea - theFieldDimensions.xPosOpponentPenaltyMark + 50.f);
  groundlineDisRangeLower = groundlineDisRangeUpper = Rangef(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark - 50.f,
                                                             theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark + 50.f);
}

void AutomaticCameraCalibratorPA::createLookUpTables()
{
  sinAngles.reserve(numOfAngles);
  cosAngles.reserve(numOfAngles);
  int index = 0;
  for(float deg = 0.f; index < numOfAngles; ++index, deg += 180_deg / numOfAngles)
  {
    cosAngles[index] = std::cos(deg);
    sinAngles[index] = std::sin(deg);
  }
}

void AutomaticCameraCalibratorPA::update(CameraCalibrationNext& cameraCalibrationNext)
{
  DEBUG_DRAWING("module:AutomaticCameraCalibratorPA:fieldLines", "drawingOnImage")
    THREAD("module:AutomaticCameraCalibratorPA:fieldLines", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");

  nextCameraCalibration = theCameraCalibration;

  if(!theGroundContactState.contact)
    lastTimeWithoutGroundContact = theFrameInfo.time;

  if(theHeadMotionEngineOutput.moving)
    lastTimeWhenHeadMoved = theFrameInfo.time;

  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibratorPA:start")
  {
    if(state == Idle)
    {
      optimizer = nullptr;
      successiveConvergations = 0;
      optimizationSteps = 0;
      currentSampleConfiguration = sampleConfigurations.begin();
      samples.resize(numOfSamples);
      std::for_each(samples.begin(), samples.end(), [](std::unique_ptr<Sample>& sample) { sample.reset(); });
      nextCameraCalibration = theCameraCalibration;
      allowSampling = true;
      state = RecordSamples;
    }
  }

  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibratorPA:abort")
    state = Idle;

  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibratorPA:converg")
  {
    if(optimizer && lowestDelta != std::numeric_limits<float>::max())
    {
      unpack(lowestDeltaParameters, nextCameraCalibration);
      OUTPUT_TEXT("AutomaticCameraCalibratorPA: converged!");
      resetOptimization(true);
    }
  }

  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibratorPA:allowNextSample")
    allowSampling = true;

  if(state == RecordSamples)
    recordSamples();

  if(state == RecordSamples && currentSampleConfiguration->samplesExist(samples))
  {
    ++currentSampleConfiguration;
    if(currentSampleConfiguration == sampleConfigurations.end())
    {
      state = Optimize;
      currentSampleConfiguration = sampleConfigurations.begin();
    }
    // Since the simulated robot does not know when it is dragged around (at least not via the GroundContactState)
    // samples are ignored until the user explicitly allows to sample (i.e. after they have turned/placed the robot
    // correctly).
    else if(SystemCall::getMode() == SystemCall::simulatedRobot)
      allowSampling = false;
  }

  if(state == Optimize)
    optimize();

  cameraCalibrationNext.setNext(nextCameraCalibration);

  COMPLEX_DRAWING("module:AutomaticCameraCalibratorPA:fieldLines")
    drawFieldLines();
}

void AutomaticCameraCalibratorPA::update(HeadAngleRequest& headAngleRequest)
{
  headAngleRequest.tilt = currentSampleConfiguration->headTilt;
  headAngleRequest.pan = currentSampleConfiguration->headPan;
  headAngleRequest.stopAndGoMode = false;
  headAngleRequest.speed = headSpeed;
}

void AutomaticCameraCalibratorPA::update(CameraResolutionRequest& cameraResolutionRequest)
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
  {
    cameraResolutionRequest.resolutions[CameraInfo::lower] = (state == Idle) ? CameraResolutionRequest::Resolutions::w320h240 : CameraResolutionRequest::Resolutions::w1280h960;
    cameraResolutionRequest.resolutions[CameraInfo::upper] = (state == Idle) ? CameraResolutionRequest::Resolutions::w640h480 : CameraResolutionRequest::Resolutions::w1280h960;
  }
}

 void AutomaticCameraCalibratorPA::update(LEDRequest& ledRequest)
 {
   if(theCameraInfo.camera != currentSampleConfiguration->camera)
     return;

   for(int i = 0; i < ledRequest.numOfLEDs; ++i)
     ledRequest.ledStates[i] = LEDRequest::off;

   LEDRequest::LED first = LEDRequest::faceLeftRed0Deg;
   static const int redOffset = 0,
                    greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
                    blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
                    numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;
   if(numOfDiscardedPossibleParallelLines > 0 || numOfDiscardedPossiblePenaltyLines > 0 || numOfDiscardedPossibleGroundLines > 0)
     for(int i = 0; i <= numOfLEDsPerColor; i++)
       ledRequest.ledStates[first + redOffset + i] = LEDRequest::on;
   else
     for(int i = 0; i <= numOfLEDsPerColor; i++)
     {
       ledRequest.ledStates[first + redOffset + i] = LEDRequest::on;
       ledRequest.ledStates[first + greenOffset + i] = LEDRequest::on;
       ledRequest.ledStates[first + blueOffset + i] = LEDRequest::on;
     }

   for(unsigned i = LEDRequest::headRearLeft0; i <= LEDRequest::headMiddleLeft0; i++)
     ledRequest.ledStates[i] = LEDRequest::on;
   if(state == RecordSamples && !allRequiredFeaturesVisible)
     for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
       ledRequest.ledStates[i] = LEDRequest::blinking;
   else if(state == Optimize)
     for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
       ledRequest.ledStates[i] = LEDRequest::fastBlinking;
 }

#define CHECK_LINE_PROJECTION(line, coordSys, camMat, camInf) \
  (Transformation::imageToRobot(coordSys.toCorrected(line.aInImage), camMat, camInf, line.aOnField) && \
   Transformation::imageToRobot(coordSys.toCorrected(line.bInImage), camMat, camInf, line.bOnField))

bool AutomaticCameraCalibratorPA::fitLine(CorrectedLine& cline)
{
  Vector2f& correctedStart = cline.aInImage;
  Vector2f& correctedEnd = cline.bInImage;
  // Determine the size of the image section to be processed
  if(correctedEnd.x() < correctedStart.x()) std::swap(correctedStart, correctedEnd);
  const Vector2i mid = ((correctedStart + correctedEnd) * 0.5f).cast<int>();
  const int sizeX = ((std::max(32, static_cast<int>(correctedEnd.x() - correctedStart.x())) + 15) / 16) * 16;
  const int sizeY = std::max(32, std::abs(static_cast<int>(correctedEnd.y() - correctedStart.y())));
  const int startX = mid.x() - sizeX / 2, startY = mid.y() - sizeY / 2;

  // Extract the image patch and calculate the sobel image
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

  // Determine the local maximas in the hough space
  std::vector<Maxima> localMaxima;
  determineLocalMaxima(houghSpace, minIndex, maxIndex, localMaxima);

  if(localMaxima.size() > 1)
  {
    // Calculate the corrected start/end of the upper or lower edge
    std::sort(localMaxima.begin(), localMaxima.end(), [](Maxima a, Maxima b) { return a.maxAcc > b.maxAcc; });
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
      Vector2f midOppositeLine = (startOpposite + endOpposite) * 0.5f;
      float disInImage = optimalLine.signedDistance(midOppositeLine);
      if(!((optimalLine.signedDistance(startOpposite) < 0.f) == (optimalLine.signedDistance(endOpposite) < 0.f)) ||
         std::abs(optimalLine.signedDistance(startOpposite)) < minDisImage ||
         std::abs(optimalLine.signedDistance(endOpposite)) < minDisImage)
        continue;
      cline.offset = disInImage > 0.f ? theFieldDimensions.fieldLinesWidth / 2.f : -theFieldDimensions.fieldLinesWidth / 2.f;
      return CHECK_LINE_PROJECTION(cline, theImageCoordinateSystem, theCameraMatrix, theCameraInfo);
    }
  }
  return false;
}

void AutomaticCameraCalibratorPA::extractImagePatch(const Vector2i& start, const Vector2i& size, Sobel::Image1D& grayImage)
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

float AutomaticCameraCalibratorPA::determineSobelThresh(const Sobel::SobelImage& sobelImage)
{
  float thresh = 0.f;
  for(unsigned int y = 1; y < sobelImage.height - 1; ++y)
    for(unsigned int x = 1; x < sobelImage.width - 1; ++x)
    {
      const Sobel::SobelPixel &pixel = sobelImage[y][x];
      const int value = pixel.x * pixel.x + pixel.y * pixel.y;
      if(value > thresh) thresh = value;
    }
  return sqr(sqrtf(thresh) * sobelThreshValue);
}

void AutomaticCameraCalibratorPA::calcHoughSpace(const Sobel::SobelImage& sobelImage, const int minIndex, const int maxIndex, const int dMax, std::vector<std::vector<int> >& houghSpace)
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

void AutomaticCameraCalibratorPA::determineLocalMaxima(const std::vector<std::vector<int> >& houghSpace, const int minIndex, const int maxIndex, std::vector<Maxima>& localMaximas)
{
  const int maxDisIndex = static_cast<int>(houghSpace[0].size());
  auto localMaxima = [houghSpace, maxDisIndex](const int value, const int angleIndex, const int distanceIndex, const int numOfAngles) -> bool
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
      if(value != 0 && localMaxima(value, angleIndex, distanceIndex, numOfAngles))
        localMaximas.push_back({value, angleIndex, distanceIndex});
    }
    if(minIndex > maxIndex && angleIndex == numOfAngles - 1)
      angleIndex = -1;
  }
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
  if(numDiscarded >= discardsUntilIncreasement) \
  { \
    numDiscarded = 0; \
    range = Rangef(range.min-increasement, range.max+increasement); \
    OUTPUT_TEXT(RangeName << " - Increased range"); \
  }

void AutomaticCameraCalibratorPA::recordSamples()
{
  if(theCameraInfo.camera != currentSampleConfiguration->camera)
    return;
  if(theFrameInfo.getTimeSince(lastTimeWithoutGroundContact) < minGroundContactDurationForSampling)
    return;
  if(theFrameInfo.getTimeSince(lastTimeWhenHeadMoved) < minRestingHeadDurationForSampling)
    return;
  if(!allowSampling)
    return;

  allRequiredFeaturesVisible = false;
  if(theLinesPercept.lines.size() >= 3 && theLinesPercept.lines.size() <= 6 &&
     (currentSampleConfiguration->needToRecord(samples, cornerAngle) || currentSampleConfiguration->needToRecord(samples, parallelAngle) || currentSampleConfiguration->needToRecord(samples, parallelLinesDistance)) &&
     !(currentSampleConfiguration->needToRecord(samples, penaltyAreaDistance) || currentSampleConfiguration->needToRecord(samples, groundlineDistance)))
  {
    bool discardedPossibleParallelLines = false, foundSuitableParallelLines = false;
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
          // i is the index of the "short" connecting line, j and k are the indices for the orthogonal lines (groundline and front penalty area line).
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
          if(((theLinesPercept.lines[j].firstField + theLinesPercept.lines[j].lastField) * 0.5f).squaredNorm() < ((theLinesPercept.lines[i].firstField + theLinesPercept.lines[i].lastField) * 0.5f).squaredNorm() ||
             ((theLinesPercept.lines[k].firstField + theLinesPercept.lines[k].lastField) * 0.5f).squaredNorm() < ((theLinesPercept.lines[i].firstField + theLinesPercept.lines[i].lastField) * 0.5f).squaredNorm())
            continue;
          allRequiredFeaturesVisible = true;

          // Fit lines through the start/end points of the lines
          CorrectedLine cLine1 = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
          CorrectedLine cLine2 = {theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>(), Vector2f(), Vector2f()};
          CorrectedLine cLine3 = {theLinesPercept.lines[k].firstImg.cast<float>(), theLinesPercept.lines[k].lastImg.cast<float>(), Vector2f(), Vector2f()};
          if(!fitLine(cLine1) || !fitLine(cLine2) || !fitLine(cLine3))
            continue;

          const Geometry::Line line2(cLine2.aOnField, cLine2.bOnField - cLine2.aOnField);
          float distance = Geometry::getDistanceToLine(line2, (cLine3.aOnField+cLine3.bOnField)*0.5f);
          distance = std::abs(distance) - (distance > 0 ? cLine2.offset - cLine3.offset : cLine3.offset - cLine2.offset);

          if(parallelLinesRange.isInside(distance))
          {
            foundSuitableParallelLines = true;
            // Use the longer line as orthogonal line.
            ADD_SAMPLE(cornerAngle, CornerAngleSample, cLine1, ((cLine2.bOnField - cLine2.aOnField).squaredNorm() >
                                                                (cLine3.bOnField - cLine3.aOnField).squaredNorm() ? cLine2 : cLine3))
            ADD_SAMPLE(parallelAngle, ParallelAngleSample, cLine2, cLine3)
            ADD_SAMPLE(parallelLinesDistance, ParallelLinesDistanceSample, cLine2, cLine3)
          }
          else
            discardedPossibleParallelLines = true;
        }
      }
    }
    INCREASE_RANGE("ParallelDisRange", discardedPossibleParallelLines, foundSuitableParallelLines, numOfDiscardedPossibleParallelLines, parallelLinesRange)
  }

  if(thePenaltyMarkPercept.wasSeen && theLinesPercept.lines.size() >= 2 && theLinesPercept.lines.size() <= 6 &&
     (currentSampleConfiguration->needToRecord(samples, penaltyAreaDistance) || currentSampleConfiguration->needToRecord(samples, groundlineDistance)))
  {
    bool discardedPossiblePenaltyLine = false, foundSuitablePenaltyLine = false, discardedPossibleGroundLine = false, foundSuitableGroundLine = false;
    Rangef& penaltyDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? penaltyDisRangeUpper : penaltyDisRangeLower;
    Rangef& groundlineDisRange = currentSampleConfiguration->camera == CameraInfo::upper ? groundlineDisRangeUpper : groundlineDisRangeLower;

    for(size_t i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      for(size_t j = i + 1; j < theLinesPercept.lines.size(); ++j)
      {
        // Heuristic: Groundline and front penalty area line should span at least half the image width.
        if(std::abs(theLinesPercept.lines[i].firstImg.x() - theLinesPercept.lines[i].lastImg.x()) < theCameraInfo.width / 2 ||
           std::abs(theLinesPercept.lines[j].firstImg.x() - theLinesPercept.lines[j].lastImg.x()) < theCameraInfo.width / 2)
          continue;
        allRequiredFeaturesVisible = true;

        CorrectedLine cLinePenalty = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
        CorrectedLine cLineGroundLine = {theLinesPercept.lines[j].firstImg.cast<float>(), theLinesPercept.lines[j].lastImg.cast<float>(), Vector2f(), Vector2f()};
        // Use the farther line as the groundline
        if((((theLinesPercept.lines[i].firstImg.cast<float>() + theLinesPercept.lines[i].lastImg.cast<float>()) * 0.5f).squaredNorm() <
            ((theLinesPercept.lines[j].firstImg.cast<float>() + theLinesPercept.lines[j].lastImg.cast<float>()) * 0.5f).squaredNorm()))
          std::swap(cLinePenalty, cLineGroundLine);
        if(!fitLine(cLinePenalty) || !fitLine(cLineGroundLine))
          continue;

        const Geometry::Line penaltyLine(cLinePenalty.aOnField, cLinePenalty.bOnField - cLinePenalty.aOnField);
        const Geometry::Line groundLine(cLineGroundLine.aOnField, cLineGroundLine.bOnField - cLineGroundLine.aOnField);
        float penaltyDistance = std::abs(Geometry::getDistanceToLine(penaltyLine, thePenaltyMarkPercept.positionOnField))-cLinePenalty.offset;
        float groundLineDistance = std::abs(Geometry::getDistanceToLine(groundLine, thePenaltyMarkPercept.positionOnField))-cLineGroundLine.offset;

        // Check if the found lines have a valid distance from the penalty spot.
        bool penaltyDistanceValid, groundLineDistanceValid;
        (penaltyDistanceValid = penaltyDisRange.isInside(penaltyDistance)) ? foundSuitablePenaltyLine = true : discardedPossiblePenaltyLine = true;
        (groundLineDistanceValid = groundlineDisRange.isInside(groundLineDistance)) ? foundSuitableGroundLine = true : discardedPossibleGroundLine = true;
        if(!penaltyDistanceValid || !groundLineDistanceValid)
          continue;

        ADD_SAMPLE(penaltyAreaDistance, PenaltyAreaDistanceSample, thePenaltyMarkPercept.positionInImage, cLinePenalty)
        ADD_SAMPLE(groundlineDistance, GroundlineDistanceSample, thePenaltyMarkPercept.positionInImage, cLineGroundLine)
        ADD_SAMPLE(parallelAngle, ParallelAngleSample, cLinePenalty, cLineGroundLine)
        ADD_SAMPLE(parallelLinesDistance, ParallelLinesDistanceSample, cLinePenalty, cLineGroundLine)
      }
    }
    INCREASE_RANGE("PenaltyLineRange", discardedPossiblePenaltyLine, foundSuitablePenaltyLine, numOfDiscardedPossiblePenaltyLines, penaltyDisRange)
    INCREASE_RANGE("GroundLineRange", discardedPossibleGroundLine, foundSuitableGroundLine, numOfDiscardedPossibleGroundLines, groundlineDisRange)
  }

  // Make sure the head movement is registered.
  if(!currentSampleConfiguration->needToRecord(samples, cornerAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelAngle) &&
     !currentSampleConfiguration->needToRecord(samples, parallelLinesDistance) &&
     !currentSampleConfiguration->needToRecord(samples, penaltyAreaDistance) &&
     !currentSampleConfiguration->needToRecord(samples, groundlineDistance))
  {
    numOfDiscardedPossibleParallelLines = numOfDiscardedPossiblePenaltyLines = numOfDiscardedPossibleGroundLines = 0;
    lastTimeWhenHeadMoved = theFrameInfo.time;
    allRequiredFeaturesVisible = true;
  }
}

void AutomaticCameraCalibratorPA::optimize()
{
  if(!optimizer)
  {
    optimizer = std::make_unique<GaussNewtonOptimizer<numOfParameterTranslations>>(functor);
    optimizationParameters = pack(theCameraCalibration);
    successiveConvergations = 0;
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

    OUTPUT_TEXT("AutomaticCameraCalibratorPA: delta = " << delta << "\n");
    if(std::abs(delta) < lowestDelta)
    {
      lowestDelta = std::abs(delta);
      lowestDeltaParameters = optimizationParameters;
    }
    ++optimizationSteps;
    if(std::abs(delta) < (terminationCriterion * std::max(1u, optimizationSteps / 500 * 50)))
      ++successiveConvergations;
    else
      successiveConvergations = 0;
    if(successiveConvergations >= minSuccessiveConvergations)
    {
      OUTPUT_TEXT("AutomaticCameraCalibratorPA: converged!");
      resetOptimization(true);
    }
    unpack(optimizationParameters, nextCameraCalibration);
  }
}

void AutomaticCameraCalibratorPA::resetOptimization(const bool finished)
{
  if(finished)
    state = Idle;
  else
  {
    OUTPUT_TEXT("Restart optimize! An optimization error occurred!");
    optimizationParameters(lowerCameraRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(lowerCameraTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(upperCameraRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(upperCameraTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(bodyRollCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    optimizationParameters(bodyTiltCorrection) = (float(rand()) / float(RAND_MAX)) * 1_deg - 0.5_deg;
    unpack(optimizationParameters, nextCameraCalibration);
  }

  optimizer = nullptr;
  lowestDelta = std::numeric_limits<float>::max();
  lowestDeltaParameters = Parameters();
  optimizationSteps = 0;
}

AutomaticCameraCalibratorPA::Parameters AutomaticCameraCalibratorPA::pack(const CameraCalibration& cameraCalibration) const
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

void AutomaticCameraCalibratorPA::unpack(const Parameters& params, CameraCalibration& cameraCalibration) const
{
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x() = static_cast<float>(std::fmod(params(lowerCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() = static_cast<float>(std::fmod(params(lowerCameraTiltCorrection), 360_deg));

  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x() = static_cast<float>(std::fmod(params(upperCameraRollCorrection), 360_deg));
  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y() = static_cast<float>(std::fmod(params(upperCameraTiltCorrection), 360_deg));

  cameraCalibration.bodyRotationCorrection.x() = static_cast<float>(std::fmod(params(bodyRollCorrection), 360_deg));
  cameraCalibration.bodyRotationCorrection.y() = static_cast<float>(std::fmod(params(bodyTiltCorrection), 360_deg));
}

void AutomaticCameraCalibratorPA::addSampleConfiguration(CameraInfo::Camera camera, float headPan, float headTilt, unsigned sampleTypes)
{
  sampleConfigurations.emplace_back();
  SampleConfiguration& sampleConfiguration = sampleConfigurations.back();
  sampleConfiguration.camera = camera;
  sampleConfiguration.headPan = headPan;
  sampleConfiguration.headTilt = headTilt;
  sampleConfiguration.sampleTypes = sampleTypes;
  sampleConfiguration.sampleIndexBase = numOfSamples;
  FOREACH_ENUM(SampleType, sampleType)
    if(sampleTypes & bit(sampleType))
      ++numOfSamples;
}

#define CHECK_PROJECTION_2_LINES(SampleName, line1, line2) \
 if(!CHECK_LINE_PROJECTION(line1, calibrator.theImageCoordinateSystem, cameraMatrix, cameraInfo) || \
    !CHECK_LINE_PROJECTION(line2, calibrator.theImageCoordinateSystem, cameraMatrix, cameraInfo)) \
 { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
 }

#define CHECK_PROJECTION_LINE_PENALTY(SampleName, line, p, p2) \
 if(!CHECK_LINE_PROJECTION(line, calibrator.theImageCoordinateSystem, cameraMatrix, cameraInfo) || \
    !Transformation::imageToRobot(calibrator.theImageCoordinateSystem.toCorrected(p), cameraMatrix, cameraInfo, p2)) \
 { \
    OUTPUT_TEXT(SampleName << " projection error!"); \
    return calibrator.notValidError; \
 }

float AutomaticCameraCalibratorPA::Sample::computeError(const CameraCalibration& cameraCalibration) const
{
  const RobotCameraMatrix robotCameraMatrix(calibrator.theRobotDimensions, headYaw, headPitch, cameraCalibration, cameraInfo.camera);
  const CameraMatrix cameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
  return computeError(cameraMatrix);
}

float AutomaticCameraCalibratorPA::CornerAngleSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("CornerAngleSample", cLine1, cLine2)

  const float dot = std::max(-1.f, std::min(static_cast<float>((cLine1.aOnField - cLine1.bOnField).normalized().dot((cLine2.aOnField - cLine2.bOnField).normalized())), 1.f));
  const float cornerAngle = std::acos(dot);
  const float cornerAngleError = std::abs(90_deg - cornerAngle);
  OUTPUT_TEXT("Angle 90: " << cornerAngle << ", error: " << cornerAngleError);
  return cornerAngleError / calibrator.angleErrorDivisor;
}

float AutomaticCameraCalibratorPA::ParallelAngleSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("ParallelAngleSample", cLine1, cLine2)

  const float dot = std::max(-1.f, std::min(static_cast<float>((cLine1.aOnField - cLine1.bOnField).normalized().dot((cLine2.aOnField - cLine2.bOnField).normalized())), 1.f));
  const float parallelAngle = std::acos(dot);
  const float parallelAngleError = std::min(parallelAngle, 180_deg - parallelAngle);
  OUTPUT_TEXT("Angle 180: " << parallelAngle << ", error: " << parallelAngleError);
  return parallelAngleError / calibrator.angleErrorDivisor;
}

float AutomaticCameraCalibratorPA::ParallelLinesDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  CHECK_PROJECTION_2_LINES("ParallelLinesDistanceSample", cLine1, cLine2)

  const Geometry::Line line1(cLine1.aOnField, cLine1.bOnField - cLine1.aOnField);
  float distance = Geometry::getDistanceToLine(line1, (cLine2.aOnField+cLine2.bOnField)*0.5f);
  distance = std::abs(distance) - (distance > 0 ? cLine1.offset - cLine2.offset : cLine2.offset - cLine1.offset);

  const float optimalDistance = calibrator.theFieldDimensions.xPosOpponentGroundline - calibrator.theFieldDimensions.xPosOpponentPenaltyArea;// + overallOffset;
  const float lineDistanceError = std::abs(distance - optimalDistance);
  OUTPUT_TEXT("LineDistanceError: " << lineDistanceError);
  return lineDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibratorPA::PenaltyAreaDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("PenaltyAreaDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField)

  const Geometry::Line line(cLine.aOnField, cLine.bOnField - cLine.aOnField);
  const float penaltyAreaDistance = std::abs(Geometry::getDistanceToLine(line, penaltyMarkOnField));
  const float penaltyAreaDistanceError = std::abs(penaltyAreaDistance - (calibrator.theFieldDimensions.xPosOpponentPenaltyArea -
                                                  calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("PenaltyDistanceError: " << penaltyAreaDistanceError);
  return penaltyAreaDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibratorPA::GroundlineDistanceSample::computeError(const CameraMatrix& cameraMatrix) const
{
  Vector2f penaltyMarkOnField;
  CHECK_PROJECTION_LINE_PENALTY("GroundlineDistanceSample", cLine, penaltyMarkInImage, penaltyMarkOnField)

  const Geometry::Line line(cLine.aOnField, cLine.bOnField - cLine.aOnField);
  const float groundlineDistance = std::abs(Geometry::getDistanceToLine(line, penaltyMarkOnField));
  const float groundlineDistanceError = std::abs(groundlineDistance - (calibrator.theFieldDimensions.xPosOpponentGroundline -
                                                 calibrator.theFieldDimensions.xPosOpponentPenaltyMark + cLine.offset));
  OUTPUT_TEXT("GroundLineDistanceError: " << groundlineDistanceError);
  return groundlineDistanceError / calibrator.distanceErrorDivisor;
}

float AutomaticCameraCalibratorPA::Functor::operator()(const Parameters& params, size_t measurement) const
{
  CameraCalibration cameraCalibration = calibrator.nextCameraCalibration;
  calibrator.unpack(params, cameraCalibration);
  return calibrator.samples[measurement]->computeError(cameraCalibration);
}

bool AutomaticCameraCalibratorPA::SampleConfiguration::needToRecord(const std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType) const
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

void AutomaticCameraCalibratorPA::SampleConfiguration::record(std::vector<std::unique_ptr<Sample>>& samples, SampleType sampleType, std::unique_ptr<Sample> sample) const
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

bool AutomaticCameraCalibratorPA::SampleConfiguration::samplesExist(const std::vector<std::unique_ptr<Sample>>& samples) const
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

void AutomaticCameraCalibratorPA::drawFieldLines() const
{
  const Pose2f robotPoseInv = validationRobotPose.inverse();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from), theCameraMatrix, theCameraInfo, lineInImage))
      LINE("module:AutomaticCameraCalibratorPA:fieldLines", lineInImage.base.x(), lineInImage.base.y(), (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1, Drawings::solidPen, ColorRGBA::black);
  }
}

bool AutomaticCameraCalibratorPA::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
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
