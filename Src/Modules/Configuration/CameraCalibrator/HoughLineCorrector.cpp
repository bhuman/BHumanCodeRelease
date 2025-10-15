/**
 * @file HoughLineCorrector.cpp
 *
 * This file implements a module that fits lines using a Sobel image and Hough transformation.
 *
 * @author Andreas Baude
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#include "HoughLineCorrector.h"
#include "Debugging/DebugDrawings.h"
#include "Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>
#include <limits>

MAKE_MODULE(HoughLineCorrector);

HoughLineCorrector::HoughLineCorrector()
{
  createLookUpTables();
}

void HoughLineCorrector::update(LineCorrector& lineCorrector)
{
  DEBUG_DRAWING("module:HoughLineCorrector:correctedLines", "drawingOnImage")
  {
    THREAD("module:HoughLineCorrector:correctedLines", theCameraInfo.getThreadName());
  }

  if(!theOptionalECImage.image)
  {
    lineCorrector.fitLine = [](LineCorrector::Line&) { return false; };
    return;
  }

  COMPLEX_DRAWING("module:HoughLineCorrector:correctedLines")
  {
    for(unsigned int i = 0; i < theLinesPercept.lines.size(); ++i)
    {
      LineCorrector::Line cLine = {theLinesPercept.lines[i].firstImg.cast<float>(), theLinesPercept.lines[i].lastImg.cast<float>(), Vector2f(), Vector2f()};
      if(fitLine(cLine))
      {
        const Vector2f cLineMid = (cLine.aInImage + cLine.bInImage) * 0.5;
        DRAW_TEXT("module:HoughLineCorrector:correctedLines", cLineMid.x(), cLineMid.y(), 10, ColorRGBA::black, cLine.offset);
        CROSS("module:HoughLineCorrector:correctedLines", cLine.aInImage.x(), cLine.aInImage.y(), 4, 2, Drawings::solidPen, ColorRGBA::red);
        CROSS("module:HoughLineCorrector:correctedLines", cLine.bInImage.x(), cLine.bInImage.y(), 4, 2, Drawings::solidPen, ColorRGBA::black);
        LINE("module:HoughLineCorrector:correctedLines", cLine.aInImage.x(), cLine.aInImage.y(), cLine.bInImage.x(), cLine.bInImage.y(), 1, Drawings::solidPen, ColorRGBA::yellow);
      }
    }
  }

  lineCorrector.fitLine = [this](LineCorrector::Line& cLine) { return fitLine(cLine); };
}

void HoughLineCorrector::createLookUpTables()
{
  sinAngles.resize(numOfAngles);
  cosAngles.resize(numOfAngles);
  unsigned index = 0;
  for(float angle = 0.f; index < numOfAngles; ++index, angle += 180_deg / numOfAngles)
  {
    cosAngles[index] = std::cos(angle);
    sinAngles[index] = std::sin(angle);
  }
}

bool HoughLineCorrector::fitLine(LineCorrector::Line& cline) const
{
  Vector2f& correctedStart = cline.aInImage;
  Vector2f& correctedEnd = cline.bInImage;
  if(correctedEnd.x() < correctedStart.x()) std::swap(correctedStart, correctedEnd);

  // If one of the points is too far away, clip them closer.
  // Otherwise the fitting algorithm could fail and the resulting line will not be parallel to the real one
  Vector2f relativePointStart, relativePointEnd;
  if(Transformation::imageToRobot(correctedStart, theCameraMatrix, theCameraInfo, relativePointStart) &&
     Transformation::imageToRobot(correctedEnd, theCameraMatrix, theCameraInfo, relativePointEnd) &&
     (relativePointEnd - relativePointStart).squaredNorm() > sqr(maxLineLength))
  {
    Vector2f corrected;
    if(relativePointStart.squaredNorm() > relativePointEnd.squaredNorm())
    {
      relativePointStart = relativePointEnd + (relativePointStart - relativePointEnd).normalized(maxLineLength);
      if(Transformation::robotToImage(relativePointStart, theCameraMatrix, theCameraInfo, corrected))
        correctedStart = corrected;
    }
    else
    {
      relativePointEnd = relativePointStart + (relativePointEnd - relativePointStart).normalized(maxLineLength);
      if(Transformation::robotToImage(relativePointEnd, theCameraMatrix, theCameraInfo, corrected))
        correctedEnd = corrected;
    }
  }

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
  std::vector<std::vector<int>> houghSpace(numOfAngles, std::vector<int>(2 * dMax + 1, 0));
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
    Vector2f n0(cosAngles[angle], sinAngles[angle]);

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
      return Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(cline.aInImage), theCameraMatrix, theCameraInfo, cline.aOnField) &&
             Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(cline.bInImage), theCameraMatrix, theCameraInfo, cline.bOnField);
    }
  }
  return false;
}

void HoughLineCorrector::extractImagePatch(const Vector2i& start, const Vector2i& size, Sobel::Image1D& grayImage) const
{
  const ECImage& theECImage = *theOptionalECImage.image;
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

unsigned int HoughLineCorrector::determineSobelThresh(const Sobel::SobelImage& sobelImage) const
{
  unsigned int thresh = 0;
  for(unsigned int y = 1; y < sobelImage.height - 1; ++y)
    for(unsigned int x = 1; x < sobelImage.width - 1; ++x)
    {
      const Sobel::SobelPixel& pixel = sobelImage[y][x];
      const unsigned int value = pixel.x * pixel.x + pixel.y * pixel.y;
      if(value > thresh) thresh = value;
    }
  return static_cast<unsigned int>(static_cast<float>(thresh) * sqr(sobelThreshValue));
}

void HoughLineCorrector::calcHoughSpace(const Sobel::SobelImage& sobelImage, const unsigned int minIndex, const unsigned int maxIndex, const unsigned int dMax, std::vector<std::vector<int> >& houghSpace) const
{
  const int thresh = determineSobelThresh(sobelImage);
  for(unsigned int y = 1; y < sobelImage.height - 1; ++y)
    for(unsigned int x = 1; x < sobelImage.width - 1; ++x)
    {
      const Sobel::SobelPixel& pixel = sobelImage[y][x];
      if(pixel.x * pixel.x + pixel.y * pixel.y >= thresh)
      {
        for(unsigned int index = minIndex; index != maxIndex; ++index)
        {
          const int d = static_cast<int>(std::ceil(x * cosAngles[index] + y * sinAngles[index]));
          ++houghSpace[index][d + dMax];
          if(minIndex > maxIndex && index == numOfAngles - 1)
            index = std::numeric_limits<unsigned int>::max();
        }
      }
    }
}

void HoughLineCorrector::determineLocalMaxima(const std::vector<std::vector<int> >& houghSpace, const unsigned int minIndex, const unsigned int maxIndex, std::vector<Maximum>& localMaxima) const
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

  for(unsigned int angleIndex = minIndex; angleIndex != maxIndex; ++angleIndex)
  {
    for(int distanceIndex = 0; distanceIndex < maxDisIndex; ++distanceIndex)
    {
      int value = houghSpace[angleIndex][distanceIndex];
      if(value != 0 && localMaximum(value, angleIndex, distanceIndex, numOfAngles))
        localMaxima.push_back({ static_cast<unsigned>(value), static_cast<unsigned>(angleIndex), static_cast<unsigned>(distanceIndex) });
    }
    if(minIndex > maxIndex && angleIndex == numOfAngles - 1)
      angleIndex = -1;
  }
}
