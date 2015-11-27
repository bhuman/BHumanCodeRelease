#include "PotentialLineSpotsProvider.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>
#include <limits>
#include <deque>
#include <algorithm>
#include <tuple>

using namespace std;

MAKE_MODULE(PotentialLineSpotsProvider, perception)


void PotentialLineSpotsProvider::update(PotentialLineSpots& potentialLineSpots)
{
  potentialLineSpots.spots.clear();
  potentialLineSpots.numOfScanlines = 0;

  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:verticalEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:width", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:spotEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:realWidth", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:error", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:spotsWithoutBot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:intersections", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:circleNormals", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:clusters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:invalidEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PotentialLineSpotsProvider:lowHigh", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("module:PotentialLineSpotsProvder:dontCorrect");

  if(!theCameraMatrix.isValid)
    return; //cannot do anything without camera matrix

  //FIXME give attributes to methods instead of accessing them directly
  STOPWATCH_WITH_PLOT("PotentialLineSpotsProvider:spots") findPotentialLineSpots(potentialLineSpots);
}

void PotentialLineSpotsProvider::findPotentialLineSpots(PotentialLineSpots& potentialLineSpots)
{
  scanlinesVert.clear();
  for(unsigned int i = theScanlineRegionsClipped.lowResStart; i < theScanlineRegionsClipped.scanlines.size(); i += std::max(theScanlineRegionsClipped.lowResStep, 1u))
  {
    runVerticalScanline(i, potentialLineSpots);
    ++potentialLineSpots.numOfScanlines;
  }
}

void PotentialLineSpotsProvider::runVerticalScanline(unsigned int scanlineIdx, PotentialLineSpots& potentialLineSpots)
{
  const RScanline& line = theScanlineRegionsClipped.scanlines.at(scanlineIdx);
  scanlinesVert.emplace_back();
  Scanline& scanline = scanlinesVert.back();
  vector<Edge> edgeBuffer; //used to buffer first edge while searching for next edge
  //start searching for non-white<->white edges
  const int x = line.x;

  EdgeType expectedEdge = GreenToWhite; //first edge we expect is greenToWhite

  for(unsigned i = 0; i < line.regions.size();) //i is increased inside the loop
  {
    int nextRegion = -1;
    int edgeY = -1;
    if(expectedEdge == GreenToWhite &&
       line.regions[i].color.is(ColorClasses::green) &&
       isEdgeTowards(line, i, nextRegion, edgeY, ColorClasses::white))
    {
      i = nextRegion;
      ASSERT(line.regions[nextRegion].color.is(ColorClasses::white));
      edgeBuffer.emplace_back(GreenToWhite, edgeY);
      expectedEdge = WhiteToGreen;
    }
    else if(expectedEdge == WhiteToGreen &&
            line.regions[i].color.is(ColorClasses::white) &&
            isEdgeTowards(line, i, nextRegion, edgeY, ColorClasses::green))
    {
      i = nextRegion;
      ASSERT(line.regions[nextRegion].color.is(ColorClasses::green));
      edgeBuffer.emplace_back(WhiteToGreen, edgeY);
      expectedEdge = GreenToWhite;
    }
    else
    {
      ++i;
    }

    if(edgeBuffer.size() == 2)
    {
      if(validateEdges(edgeBuffer[0], edgeBuffer[1], x))//check if edges roughly fit a field line
      {
        CROSS("module:PotentialLineSpotsProvider:spotEdges", x, edgeBuffer[0].y, 2, 1, Drawings::solidPen, ColorRGBA::blue);
        CROSS("module:PotentialLineSpotsProvider:spotEdges", x, edgeBuffer[1].y, 2, 1, Drawings::solidPen, ColorRGBA::blue);
        const int middleY = static_cast<int>((edgeBuffer[0].y + edgeBuffer[1].y) / 2);
        const int height = edgeBuffer[0].y - edgeBuffer[1].y;
        ASSERT(height > 0);
        ASSERT(scanline.size() > 0 ? scanline.back().spotInImg.y() > middleY : true);
        Vector2f inField;
        Vector2f inImage(x, middleY);
        inImage = theImageCoordinateSystem.toCorrected(inImage);
        if(Transformation::imageToRobot(inImage, theCameraMatrix, theCameraInfo, inField))
        {
          scanline.emplace_back(x, middleY, false, inField, height);
          potentialLineSpots.spots.emplace_back(x, middleY, false, inField, height);
          potentialLineSpots.spots.back().scanlineIdx = potentialLineSpots.numOfScanlines;
          CROSS("module:PotentialLineSpotsProvider:spots", x, middleY, 2, 1, Drawings::solidPen, ColorRGBA::red);
        }
      }
      else
      {
        CROSS("module:PotentialLineSpotsProvider:invalidEdges", x, edgeBuffer[0].y, 2, 1, Drawings::solidPen, ColorRGBA::red);
        CROSS("module:PotentialLineSpotsProvider:invalidEdges", x, edgeBuffer[1].y, 2, 1, Drawings::solidPen, ColorRGBA::red);
      }
      edgeBuffer.clear();
    }
  }
}

Vector2f PotentialLineSpotsProvider::calculateEdgeDirection(const int y, const int x) const
{
  //FIXME compare performance of different sobel implementations

  const int increment = sizeof(Image::Pixel);
  const int widthstep = theImage.widthStep * sizeof(Image::Pixel); //because we add it to char*
  const unsigned char* p = ((const unsigned char*) (&theImage[y - 1][x - 1])) + offsetof(Image::Pixel, y);
  int sumX = *p;
  ASSERT(*p == theImage[y - 1][x - 1].y);
  int sumY = sumX;
  p += increment;
  ASSERT(*p == theImage[y - 1][x].y);
  sumY += 2 * (*p);
  p += increment;
  ASSERT(*p == theImage[y - 1][x + 1].y);
  sumY += *p;
  sumX -= *p;

  p += widthstep;
  ASSERT(*p == theImage[y][x + 1].y);

  sumX -= 2 * (*p);
  p -= 2 * increment;
  ASSERT(*p == theImage[y][x - 1].y);
  sumX += 2 * (*p);

  p += widthstep;
  ASSERT(*p == theImage[y + 1][x - 1].y);

  sumX += *p;
  sumY -= *p;
  p += increment;
  ASSERT(*p == theImage[y + 1][x].y);
  sumY -= 2 * (*p);
  p += increment;
  ASSERT(*p == theImage[y + 1][x + 1].y);
  sumX -= *p;
  sumY -= *p;

  Vector2f ret((float) sumX, (float) sumY);
  ARROW("module:PotentialLineSpotsProvider:verticalEdges", x, y, x + ret.x(), y + ret.y(),
        1, Drawings::solidPen, ColorRGBA::red);

  return ret;
}

bool PotentialLineSpotsProvider::validateEdges(const Edge& gToW, const Edge& wToG, const int x) const
{
  const float expectedDistance = calculateExpectedLineWidth(x, gToW.y) * maxAllowedLineHeightFactor;
  return expectedDistance >= 0 && abs(gToW.y - wToG.y) <= expectedDistance;
}

float PotentialLineSpotsProvider::calculateExpectedLineWidth(const int x, const int y) const
{
  const float lineSize = calculateLineSize(Vector2f((float) x, (float) y));
  return lineSize;
  if(x > theImage.width - 2 || y > theImage.height - 2 || x < 1 || y < 1)
  {
    //cannot detect line direction therefore we just return the default size
    return lineSize;
  }

  Vector2f lineDir = calculateEdgeDirection(y, x);
  if(lineDir.y() < 0.1f) //should be <= 0 but values like 0.0000001 break the calculation as well
  {
    //rarely happens at corners or junctions and breaks the calculation below
    return 0;
  }

  lineDir.mirror();
  lineDir.normalize(lineSize);

  //calculate the hypotenuse of the triangle.
  const float alpha = atan2(-lineDir.y(), lineDir.x()) - pi_2; //use -y to compensate
  const float c = lineSize / cos(alpha);
  ASSERT(c >= 0);
  return c;
}

bool PotentialLineSpotsProvider::isEdgeTowards(const RScanline& line, const int currentIndex, int& outNewIndex,
                                               int& outEdgeY, ColorClasses::Color color) const
{
  int noiseCounter = 0;
  const int startIndex = std::min(currentIndex + 1, static_cast<int>(line.regions.size() - 1));
  const int firstLower = line.regions[startIndex].lower;
  for(unsigned i = startIndex; i < line.regions.size(); ++i)
  {
    const Region& r = line.regions[i];
    if(r.color.is(color))
    {
      //found desired region
      outNewIndex = i;
      outEdgeY = static_cast<int>((r.lower + firstLower) / 2);
      return true;
    }
    else
    {
      noiseCounter += r.lower - r.upper;
      if(noiseCounter > maxEdgeNoisePixels)
      {
        return false;
      }
    }
  }

  return false;
}
