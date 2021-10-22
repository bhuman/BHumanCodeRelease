/**
 * @file FieldBoundaryProvider.cpp
 *
 * This file implements a module that calculates the field boundary
 * using a deep neural network (and subsequent line fitting).
 *
 * @author Arne Hasselbring
 * @author Yannik Meinken
 * @author Jonah Jaeger
 * @author Thorben Lorenzen
 */

#include "FieldBoundaryProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Global.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(FieldBoundaryProvider, perception);

FieldBoundaryProvider::FieldBoundaryProvider() :
  network(&Global::getAsmjitRuntime())
{
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir()) + ((theCameraInfo.camera == CameraInfo::upper) ? "/Config/NeuralNets/FieldBoundary/net.h5" : "/Config/NeuralNets/FieldBoundary/net-uncertainty.h5"));
  model->setInputUInt8(0);

  network.compile(*model);

  ASSERT(network.valid());

  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.numOfOutputs() == 1);

  ASSERT(network.input(0).rank() == 3);
  ASSERT(network.input(0).dims(2) == 1 || network.input(0).dims(2) == 3);

  patchSize = Vector2i(network.input(0).dims(1), network.input(0).dims(0));

  ASSERT(network.output(0).rank() == 1 || network.output(0).rank() == 2);
  ASSERT(network.output(0).dims(0) == static_cast<unsigned>(patchSize.x()));
  ASSERT(network.output(0).rank() == 1 || network.output(0).dims(1) == 2);
}

void FieldBoundaryProvider::update(FieldBoundary& fieldBoundary)
{
  DECLARE_DEBUG_DRAWING("module:FieldBoundaryProvider:prediction", "drawingOnImage");
  DECLARE_DEBUG_RESPONSE("module:FieldBoundaryProvider:debugPrints");

  fieldBoundary.boundaryInImage.clear();
  fieldBoundary.boundaryOnField.clear();
  if((fieldBoundary.isValid = network.valid() && theCameraMatrix.isValid))
  {
    if(!fieldBoundary.isValid)
    {
      std::vector<Spot> spots;
      predictSpots(spots);
      validatePrediction(fieldBoundary, spots);
    }
    else if(theCameraInfo.camera == CameraInfo::upper)
    {
      std::vector<Spot> spots;
      predictSpots(spots);
      bool odd = boundaryIsOdd(spots);
      (odd && ! theOtherFieldBoundary.extrapolated) ? projectPrevious(fieldBoundary) : validatePrediction(fieldBoundary, spots);
      fieldBoundary.odd = odd;
    }
    else if(theCameraInfo.camera == CameraInfo::lower)
    {
      if(theOtherFieldBoundary.odd || theOtherFieldBoundary.extrapolated || theOtherFieldBoundary.boundaryInImage.size() == 0)
      {
        std::vector<Spot> spots;
        predictSpots(spots);

        theOtherFieldBoundary.boundaryInImage.size() > 1 && boundaryIsOdd(spots) ? projectPrevious(fieldBoundary) : validatePrediction(fieldBoundary, spots);
      }
      else
        projectPrevious(fieldBoundary);
    }
  }
  fieldBoundary.isValid = fieldBoundary.boundaryInImage.size() > 1;

  if(!fieldBoundary.isValid)
  {
    fieldBoundary.boundaryInImage.clear();
    fieldBoundary.boundaryOnField.clear();
  }
}

void FieldBoundaryProvider::validatePrediction(FieldBoundary& fieldBoundary, std::vector<Spot>& spots)
{
  if((fieldBoundary.isValid = (spots.size() >= minNumberOfSpots)))
  {
    if(fittingMethod == Ransac)
    {
      std::vector<Spot> newSpots;
      for(auto s : spots)
      {
        if(s.inImage.y() > top)
          newSpots.push_back(s);
      }
      STOPWATCH("FieldBoundaryProvider:fitBoundaryRansac")
        fitBoundaryRansac(newSpots, fieldBoundary);
    }
    else if(fittingMethod == NotRansac)
    {
      std::vector<Spot> newSpots;
      for(auto s : spots)
      {
        if(s.inImage.y() > top)
          newSpots.push_back(s);
      }
      STOPWATCH("FieldBoundaryProvider:fitBoundaryNotRansac")
        fitBoundaryNotRansac(newSpots, fieldBoundary);
    }
    else if(fittingMethod == NoFitting)
    {
      for(const Spot& spot : spots)
      {
        fieldBoundary.boundaryInImage.emplace_back(spot.inImage);
        fieldBoundary.boundaryOnField.emplace_back(spot.onField);
      }
    }
    fieldBoundary.extrapolated = false;
  }
}

void FieldBoundaryProvider::projectPrevious(FieldBoundary& fieldBoundary)
{
  const Pose2f invOdometryOffset = theOdometer.odometryOffset.inverse();
  for(Vector2f spotOnField : theOtherFieldBoundary.boundaryOnField)
  {
    Vector2f spotInImage;
    spotOnField = invOdometryOffset * spotOnField;
    if(Transformation::robotToImage(spotOnField, theCameraMatrix, theCameraInfo, spotInImage))
    {
      fieldBoundary.boundaryInImage.emplace_back(theImageCoordinateSystem.fromCorrected(spotInImage).cast<int>());
      fieldBoundary.boundaryOnField.emplace_back(spotOnField);
    }
  }
  fieldBoundary.extrapolated = true;
}

void FieldBoundaryProvider::predictSpots(std::vector<Spot>& spots)
{
  unsigned char* input = reinterpret_cast<std::uint8_t*>(network.input(0).data());

  if(network.input(0).dims(2) == 1)
    PatchUtilities::extractInput<std::uint8_t, true>(theCameraImage, patchSize, input);
  else
    PatchUtilities::extractInput<std::uint8_t, false>(theCameraImage, patchSize, input);

  network.apply();
  const float* output = network.output(0).data();

  const unsigned int xScale = theCameraInfo.width / patchSize(0);
  const unsigned int stepSize = network.output(0).rank() == 2 ? 2 : 1;
  for(int x = 0, idx = 0; x < patchSize(0); ++x, idx += stepSize)
  {
    const Vector2f spotInImage(x * xScale + xScale / 2, std::max(0.f, std::min(output[idx], 1.f)) * static_cast<float>(theCameraInfo.height - 1));
    DOT("module:FieldBoundaryProvider:prediction", spotInImage.x(), spotInImage.y(), ColorRGBA::orange, ColorRGBA::orange);
    float uncertainty = 0;

    if(network.output(0).rank() == 2)
    {
      uncertainty = 1.f / (output[idx + 1] * output[idx + 1]) * static_cast<float>(theCameraInfo.height - 1);
      DOT("module:FieldBoundaryProvider:prediction", spotInImage.x(), spotInImage.y() + uncertainty, ColorRGBA::blue, ColorRGBA::blue);
      DOT("module:FieldBoundaryProvider:prediction", spotInImage.x(), spotInImage.y() - uncertainty, ColorRGBA::blue, ColorRGBA::blue);
    }

    Vector2f spotOnField;
    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(spotInImage), theCameraMatrix, theCameraInfo, spotOnField) && spotOnField.squaredNorm() >= sqr(minDistance))
      spots.emplace_back(spotInImage.cast<int>(), spotOnField, uncertainty);
  }
}

bool FieldBoundaryProvider::boundaryIsOdd(const std::vector<Spot>& spots) const
{
  //with less than 3 points not usable
  if(spots.size() < 3)
    return true;

  int yMaxBorder = 0;
  //find the lowest point of the three left/right most points
  for(size_t i = 0; i < 3; i++)
  {
    if(spots[i].inImage.y() > yMaxBorder)
      yMaxBorder = spots[i].inImage.y();

    if(spots[spots.size() - 1 - i].inImage.y() > yMaxBorder)
      yMaxBorder = spots[spots.size() - 1 - i].inImage.y();
  }
  int toLowSum = 0;
  float uncertaintySum = 0;
  float sum = 0;
  float gradient;
  int num = 0;
  //for each triple of following points sum the difference from the middle point to the line between the first and last point.
  for(size_t i = 0; i < spots.size(); i++)
  {
    //only if the point is not at the top of the image
    if(spots[i].inImage.y() > top)
    {
      num++;
      //if the point is below the lower end
      if(spots[i].inImage.y() > yMaxBorder)
        toLowSum += spots[i].inImage.y() - yMaxBorder;

      uncertaintySum += spots[i].uncertanty;

      if(i + 2 >= spots.size())
        continue;

      //compute the discrepancy of the middle point to the line between the first and third
      gradient = ((float) spots[i].inImage.y() - spots[i + 2].inImage.y()) / (spots[i].inImage.x() - spots[i + 2].inImage.x());
      sum += std::abs((spots[i].inImage.y() + gradient * (spots[i + 1].inImage.x() - spots[i].inImage.x())) - spots[i + 1].inImage.y());
      LINE("module:FieldBoundaryProvider:prediction", spots[i + 1].inImage.x(), spots[i + 1].inImage.y(), spots[i + 1].inImage.x(), (spots[i].inImage.y() + gradient * (spots[i + 1].inImage.x() - spots[i].inImage.x())), 1, Drawings::solidPen, ColorRGBA::red);
    }
  }
  //if one of the limits is violated
  if(((float)num / spots.size() < nonTopPoints) || uncertaintySum / num > uncertaintyLimit  || toLowSum / num > maxPointsUnderBorder || sum / num > threshold)
  {
    DEBUG_RESPONSE("module:FieldBoundaryProvider:debugPrints")
    {
      if(num)
      {
        OUTPUT_TEXT(((float)num / spots.size() < nonTopPoints) << ", " << (uncertaintySum / num > uncertaintyLimit) << ", " << (toLowSum / num > maxPointsUnderBorder) << ", " << (sum / num > threshold));
        OUTPUT_TEXT((float)num / spots.size() << ", " << uncertaintySum / num <<  ", " << toLowSum / num << ", " << sum / num);
      }
      else
        OUTPUT_TEXT("num = " << num);
    }
    return true;
  }
  else
    return false;
}

void FieldBoundaryProvider::fitBoundaryRansac(const std::vector<Spot>& spots, FieldBoundary& fieldBoundary)
{
  std::vector<Spot> fbmodel;
  fbmodel.reserve(3);

  int minError = std::numeric_limits<int>::max();
  const int goodEnough = static_cast<int>(static_cast<float>(maxSquaredError * spots.size()) * acceptanceRatio);

  for(int i = 0; i < maxNumberOfIterations && minError > goodEnough; ++i)
  {
    // Draw three unique samples sorted by their x-coordinate.
    const size_t middleIndex = Random::uniformInt(static_cast<size_t>(1), spots.size() - 2);
    const Spot& leftSpot = spots[Random::uniformInt(middleIndex - 1)];
    const Spot& middleSpot = spots[middleIndex];
    const Spot& rightSpot = spots[Random::uniformInt(middleIndex + 1, spots.size() - 1)];

    // Construct lines, second is perpendicular to first one on the field.
    Vector2f dirOnField = middleSpot.onField - leftSpot.onField;
    const Geometry::Line leftOnField(leftSpot.onField, dirOnField);
    const Geometry::Line rightOnField(rightSpot.onField, dirOnField.rotateLeft()); // Changes dirOnField!

    // Compute hypothetical corner in field coordinates.
    Vector2f inImage;
    Spot corner;
    if(Geometry::getIntersectionOfLines(leftOnField, rightOnField, corner.onField)
       && Transformation::robotToImage(corner.onField, theCameraMatrix, theCameraInfo, inImage))
    {
      corner.inImage = theImageCoordinateSystem.fromCorrected(inImage).cast<int>();

      // Corner must be right of left spot, left of the right spot, and above connecting line.
      if(corner.inImage.x() <= leftSpot.inImage.x() || corner.inImage.x() >= rightSpot.inImage.x()
         || corner.inImage.y() >= leftSpot.inImage.y() + (corner.inImage.x() - leftSpot.inImage.x())
         * (rightSpot.inImage.y() - leftSpot.inImage.y()) / (rightSpot.inImage.x() - leftSpot.inImage.x()))
        corner.inImage.x() = theCameraInfo.width; // It is not -> ignore
    }
    else
      corner.inImage.x() = theCameraInfo.width; // Corner invalid -> ignore

    const Vector2i dirLeft = middleSpot.inImage - leftSpot.inImage;
    const Vector2i dirRight = rightSpot.inImage - corner.inImage;

    size_t j = 0;

    // Accumulate errors in image coordinates for left line.
    int errorLeft = 0;
    while(j < spots.size() && spots[j].inImage.x() < corner.inImage.x() && errorLeft < minError)
    {
      const Vector2i& s = spots[j++].inImage;
      errorLeft += effectiveError(leftSpot.inImage.y() + dirLeft.y() * (s.x() - leftSpot.inImage.x()) / dirLeft.x() - s.y());
    }

    // Accumulate errors in image coordinates, assuming both a continuing left line and a separate right line.
    int errorRightLine = 0; // Assuming a separate line on the right.
    int errorRightStraight = 0;// Assuming left line continues.
    const int abortError = minError - errorLeft;
    while(j < spots.size() && std::min(errorRightLine, errorRightStraight) < abortError)
    {
      const Vector2i& s = spots[j++].inImage;
      errorRightLine += effectiveError(corner.inImage.y() + dirRight.y() * (s.x() - corner.inImage.x()) / dirRight.x() - s.y());
      errorRightStraight += effectiveError(leftSpot.inImage.y() + dirLeft.y() * (s.x() - leftSpot.inImage.x()) / dirLeft.x() - s.y());
    }

    // Update model if it is better than the best found so far.
    const int error = errorLeft + std::min(errorRightLine, errorRightStraight);
    if(error < minError)
    {
      minError = error;
      fbmodel.clear();
      fbmodel.emplace_back(leftSpot);

      if(errorRightLine < errorRightStraight)
      {
        fbmodel.emplace_back(corner);
        fbmodel.emplace_back(rightSpot);
      }
      else
        fbmodel.emplace_back(middleSpot);
    }
  }

  for(const Spot& spot : fbmodel)
  {
    fieldBoundary.boundaryInImage.emplace_back(spot.inImage);
    fieldBoundary.boundaryOnField.emplace_back(spot.onField);
  }
}

void FieldBoundaryProvider::fitBoundaryNotRansac(const std::vector<Spot>& spots, FieldBoundary& fieldBoundary)
{
  struct TwoLineModel
  {
    LineCandidate first, second;
    Vector2f peak;
    std::vector<Vector2f> pointsInImage;
    Angle angleDev = 359_deg;
  };
  TwoLineModel bestModel;

  for(unsigned int i = 2; i < spots.size() - 2; i += 2)
  {
    LineCandidate first(spots, 0, i, true), second(spots, i, static_cast<int>(spots.size()), true);

    // TODO: Do this in a less stupid way
    Vector2f peak, firstInImage, peakInImage, secondInImage;
    if(!Geometry::getIntersectionOfLines(first.line, second.line, peak) ||
       !Transformation::robotToImage(first.line.base, theCameraMatrix, theCameraInfo, firstInImage) ||
       !Transformation::robotToImage(peak, theCameraMatrix, theCameraInfo, peakInImage) ||
       !Transformation::robotToImage(second.line.base, theCameraMatrix, theCameraInfo, secondInImage) ||
       firstInImage.x() > peakInImage.x() || secondInImage.x() < peakInImage.x())
      continue;

    Angle angleDev = std::abs(first.line.direction.angleTo(second.line.direction) - 90_deg);
    if(angleDev < bestModel.angleDev)
      bestModel = TwoLineModel({first, second, peak, {firstInImage, peakInImage, secondInImage}, angleDev});
  }

  if(bestModel.angleDev < randomlyChosenAngleDevThreshold)
  {
    fieldBoundary.boundaryOnField = {bestModel.first.line.base, bestModel.peak, bestModel.second.line.base};
    for(Vector2f& p : bestModel.pointsInImage)
      fieldBoundary.boundaryInImage.emplace_back(theImageCoordinateSystem.fromCorrected(p).cast<int>());
  }
  else
  {
    LineCandidate singleLine(spots, 0, static_cast<int>(spots.size()), false);
    std::vector<Vector2f> boundaryPoints = {singleLine.line.base - 200.f * singleLine.line.direction, singleLine.line.base, singleLine.line.base + 200.f * singleLine.line.direction};
    for(Vector2f& point : boundaryPoints)
    {
      Vector2f onField;
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(point), theCameraMatrix, theCameraInfo, onField))
      {
        fieldBoundary.boundaryInImage.emplace_back(point.cast<int>());
        fieldBoundary.boundaryOnField.emplace_back(onField);
      }
    }
  }
}
