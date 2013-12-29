/*
 * ObstacleSpotProvider.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "ObstacleSpotProvider.h"
#include "Representations/Perception/BodyContour.h"
#include <cfloat>
#include <algorithm>

using namespace std;

//initialize lastFrameImageHeight with 1 to avoid division by zero if the first image received is from the upper cam
ObstacleSpotProvider::ObstacleSpotProvider() : lastFrameImageHeight(1), sawSpotsInLowerCamLastFrame(false)
{}

void ObstacleSpotProvider::update(ObstacleSpots& spots)
{
  //FIXME ignore spots outside field border
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:height", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:expectedHeight", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:obstacleSpotUpper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:upHeight", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:upExpectedHeight", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:possibleSpots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:handsArea", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstacleSpotProvider:lowerHandsArea", "drawingOnImage");

  spots.obstacles.clear();
  possibleSpots.clear();

  if(!theGroundContactState.contact)
  {//Dont create any points if we are flying
    return;
  }

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    handleBufferedSpots(possibleSpots);
  }
  lastFrameImageHeight = theCameraInfo.height;
  if(sawSpotsInLowerCamLastFrame && theCameraInfo.camera == CameraInfo::upper)
  {
    sawSpotsInLowerCamLastFrame = false;
    return;
  }

  handleNewSpots(possibleSpots);
  if(theCameraInfo.camera == CameraInfo::lower)
  {//special case to handle hands in lower camera
   //note: this will partially sort the spotBuffer
   //note2: This is a dirty HACK :)
    removeLowerImageBanZone(possibleSpots, spotBuffer);
  }
  sortSpots(possibleSpots); //this way removeBanZone is more efficient and produces less false positives
  removeBanZone(possibleSpots);
  clusterSpots(possibleSpots, spots);
  correctSpots(spots);
  calcClusterBanZone(spots);
  removeOverlappingBanZones(spots);

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    //If an obstacle leaves the upper image into the lower one the spot is directly
    //at the bottom of the image.
    //Those spots are useless because they are not on the field.
    //however they are still used to remove hands, therefore they are filtered out
    //after the hands have been filtered out
    checkIfTooCloseToBottom(spots);
  }

  if(theCameraInfo.camera == CameraInfo::lower &&
     spots.obstacles.size() > 0)
  {
    sawSpotsInLowerCamLastFrame = true;
  }
  else
  {
    sawSpotsInLowerCamLastFrame = false;
  }

}

bool ObstacleSpotProvider::searchDownward(const int x, int y, int yEnd, Vector2<int>& outBottomCoordinate)
{
  if(y < nonGreenCount)
  {
    return false;
  }

  bool breakCausedByGreenCount = false;
  yEnd = yEnd - (nonGreenCount + 1); //in worst case the loop runs nonGreenCount pixels further than yEnd
  int up = 0;
  const Image::Pixel* pPixel = &theImage[y][x];
  const Image::Pixel* pEnd   = &theImage[yEnd][x];
  const int widthstep = theImage.widthStep * nonGreenCount;
  //this loop jumps in nonGreenCount steps and searches backwards if green is
  //found to ensure that there is enough green to skip
  for(; pPixel <= pEnd; pPixel += widthstep, y += nonGreenCount)
  {
    ASSERT(pPixel >= &theImage[0][0]);
    ASSERT(pPixel <= &theImage[theImage.height-1][theImage.width-1]);
    if(isGroundColor(pPixel))
    {
      //search upward to determine how much green we have jumped
      up = countGreen(pPixel, pPixel - widthstep,
                                      theImage.widthStep);

      const int bottomCount = countGreen(pPixel + theImage.widthStep,
                                         pPixel + widthstep, theImage.widthStep);
      if(up + bottomCount > nonGreenCount)
      {
        breakCausedByGreenCount = true;
        break;
      }
      else
      {
        up = 0;
      }
    }
  }
  outBottomCoordinate = Vector2<int>(x,y - up);
  return breakCausedByGreenCount;
}

Vector2<int> ObstacleSpotProvider::searchUpward(const int x, int y, const int yEnd)
{
  const Image::Pixel* pPixel = &theImage[y][x];
  int greenCount = 0;
  for(; y > yEnd; --y, pPixel -= theImage.widthStep)
  {
    ASSERT(pPixel >= &theImage[0][0]);
    ASSERT(pPixel <= &theImage[theImage.height-1][theImage.width-1]);
    if(isGroundColor(pPixel))
    {
      ++greenCount;
    }
    else
    {
      greenCount = 0;
    }

    if(greenCount > nonGreenCount)
    {
      break;
    }
  }
  return Vector2<int>(x,y + greenCount);
}

bool ObstacleSpotProvider::fullFillsHeightConstraint(const int height, const Vector2<int>& footPoint)
{
  Vector2<float> footPointOnField;
  Geometry::calculatePointOnField(footPoint.x, footPoint.y, theCameraMatrix, theCameraInfo, footPointOnField);
  int expectedHeight = int(theCameraInfo.focalLength * minObstacleHeight / (footPointOnField.absFloat() + FLT_EPSILON));
  int expectedHeightClipped = expectedHeight; //we have to copy this because we need the original expectedHeight later
  bool hasToLeaveImage = false; //indicates whether the obstacle has to leave the image or not.
  //clip at image border
  if(expectedHeight >= footPoint.y){
    expectedHeightClipped = footPoint.y;
    hasToLeaveImage = true;
  }

  LINE("module:ObstacleSpotProvider:expectedHeight", footPoint.x + 2, footPoint.y,
       footPoint.x + 2, footPoint.y - expectedHeight, 2, Drawings::ps_solid,hasToLeaveImage? ColorClasses::blue :  ColorClasses::green);

  //if the obstacle has to leave the image we cannot determine if it is an
  //obstacle this frame. If the obstacle actually leaves the image it is
  //buffered and checked in the next frame.
  //This is only possible if the current image was taken by the lower camera.
  if(hasToLeaveImage && theCameraInfo.camera == CameraInfo::lower)
  {
    if(abs(height-expectedHeightClipped) <= allowedDifferenceFromTop)
    {//if it actually leaves the image
      Vector2<float> borderSpotOnField;
      Geometry::calculatePointOnField(footPoint.x, allowedDifferenceFromTop, theCameraMatrix,
                                      theCameraInfo, borderSpotOnField);
      int remainingHeight = expectedHeight - height;
      //not sure if obstacle or not.
      //remember for next frame and check upper image
      BufferedSpot bSpot;
      bSpot.borderPoint = borderSpotOnField;
      bSpot.spotInWorld = footPointOnField;
      bSpot.spotInImg = footPoint;
      bSpot.height = remainingHeight;
      spotBuffer.push_back(bSpot);
    }
    return false;
  }

  return checkHeight(height, expectedHeight);
}

bool ObstacleSpotProvider::checkHeight(const int height, const int expectedHeight)
{
  return height >= expectedHeight || abs(height - expectedHeight) <= expectedHeight * allowedHeightDifference;
}

void ObstacleSpotProvider::handleBufferedSpots(std::list<Vector2<int> >& possibleSpots)
{
   //used to scale the height if the upper and lower camera use a different resolution
    float resolutionScale = theCameraInfo.height / float(lastFrameImageHeight);
    //check buffered spots from last frame
    for(BufferedSpot& spot : spotBuffer)
    {
      //update odometry
      spot.borderPoint = spot.borderPoint.rotate(-theOdometer.odometryOffset.rotation);
      spot.borderPoint -= theOdometer.odometryOffset.translation;
      Vector2<int> spotInImg;
      Geometry::calculatePointInImage(spot.borderPoint, theCameraMatrix, theCameraInfo, spotInImg);

      //sometimes the time between two images is so big that head has moved too much
      //and the spot is not visible in the upper image.
      //This usually happens in logfiles due to the reduced frame rate when recording logs
      //There is nothing to calculate for such spots
      if(spotInImg.x < 0 || spotInImg.x >= theImage.width ||
         spotInImg.y < 0 || spotInImg.y >= theImage.height)
      {
        continue;
      }

      //search upward to see if the obstacle continues in the upper image
      Vector2<int> obstacleEnd = searchUpward(spotInImg.x, spotInImg.y, 0); //FIXME use horizon
      LINE("module:ObstacleSpotProvider:upHeight", spotInImg.x, spotInImg.y,
           obstacleEnd.x, obstacleEnd.y, 2, Drawings::ps_solid, ColorClasses::red);

      const int height = spotInImg.y - obstacleEnd.y;
      const int expectedHeight = int(spot.height * resolutionScale);
      LINE("module:ObstacleSpotProvider:upExpectedHeight", spotInImg.x + 1, spotInImg.y,
           spotInImg.x + 1, spotInImg.y - expectedHeight , 2, Drawings::ps_solid, ColorClasses::blue);
      if(checkHeight(height, expectedHeight))
      {//the spot that was seen by the lower camera is an obstacle
        spot.spotInWorld = spot.spotInWorld.rotate(-theOdometer.odometryOffset.rotation);
        spot.spotInWorld -= theOdometer.odometryOffset.translation;
        Vector2<int> pointInImg;
        Geometry::calculatePointInImage(spot.spotInWorld, theCameraMatrix, theCameraInfo, pointInImg);
        CROSS("module:ObstacleSpotProvider:obstacleSpotUpper",
              pointInImg.x , pointInImg.y,2, 1, Drawings::ps_solid, ColorClasses::red);
          //this spot is most likely outside the image but that shouldn't matter
        possibleSpots.push_back(pointInImg);
      }
    }
    spotBuffer.clear();
}

void ObstacleSpotProvider::removeBanZone(std::list<Vector2<int> >& possibleSpots)
{
  int xLeft, xRight, yTop, yBottom;

  for(std::list<Vector2<int> >::iterator spot = possibleSpots.begin(); spot != possibleSpots.end(); ++spot)
  {
    calcBanZone(*spot, xLeft, xRight, yTop, yBottom, legHeight);

    std::list<Vector2<int> >::iterator s = possibleSpots.begin();
    while(s != possibleSpots.end())
    {
      if(s->x == spot->x && s->y == spot->y)
      {//skip spot. Otherwise we might break the iterator
        ++s;
      } else if(s->x >= xLeft && s->x <= xRight && s->y >= yTop && s->y <= yBottom)
      {
        s = possibleSpots.erase(s);
      }
      else
      {
        ++s;
      }
    }
  }
}

void ObstacleSpotProvider::calcBanZone(const Vector2<int>& spot, int& outXLeft, int& outXRight, int& outYTop, int& outYBottom, int legHeight)
{
  const float dist = calculateDistanceTo(spot);
  int height = calculateLineSize(dist, legHeight);
  int width = calculateLineSize(dist, banZoneWidth);

  outXLeft = spot.x - width;
  outXRight = spot.x + width;
  outYBottom = spot.y - height;
  outYTop = 0; //ban zone goes all the way to the top :)

  if(outXLeft < 0) outXLeft = 0;
  if(outXRight >= theImage.width) outXRight = theImage.width - 1;
  if(outYBottom < 0) outYBottom = 0;

    ARROW("module:ObstacleSpotProvider:handsArea", spot.x, spot.y, outXRight, outYBottom,
      2.f, Drawings::ps_solid, ColorClasses::blue);

    RECTANGLE("module:ObstacleSpotProvider:handsArea", outXLeft, outYTop, outXRight, outYBottom,
              2, Drawings::ps_solid, ColorClasses::blue);

}

inline int ObstacleSpotProvider::squareDistance(const Vector2<int>& a,const Vector2<int>& b) const
{
  const int x = a.x - b.x;
  const int y = a.y - b.y;
  return x * x + y * y;
}

int ObstacleSpotProvider::countGreen(const Image::Pixel* pStart,const Image::Pixel* pEnd,
                                     const int widthstep)
{
  int count = 0;
  if(pStart <= pEnd) //scan forward
  {
    for(; pStart <= pEnd; pStart += widthstep)
    {
      ASSERT(pStart >= &theImage[0][0]);
      ASSERT(pStart <= &theImage[theImage.height-1][theImage.width-1]);
      if(isGroundColor(pStart))
      {
        ++count;
      }
      else
      {
        break;
      }
    }
  }
  else //scan backwards
  {//FIXME duplicate code
    for(; pStart >= pEnd; pStart -= widthstep)
    {
      ASSERT(pStart >= &theImage[0][0]);
      ASSERT(pStart <= &theImage[theImage.height-1][theImage.width-1]);
      if(isGroundColor(pStart))
      {
        ++count;
      }
      else
      {
        break;
      }
    }
  }
  return count;
}

bool ObstacleSpotProvider::isGroundColor(const Image::Pixel* pPixel) const
{
  ASSERT(pPixel >= &theImage[0][0]);
  ASSERT(pPixel <= &theImage[theImage.height-1][theImage.width-1]);

  ColorReference::MultiColor colors = theColorReference.getColorClasses(pPixel);
  return colors.isGreen() || colors.isBlack() || colors.isOrange();
}

void ObstacleSpotProvider::handleNewSpots(std::list<Vector2<int> >& possibleSpots)
{
  //pull the first non line spot of each scanline down until it hits green
  const int lineCount = thePossibleObstacleSpots.scanlineCount;
  const int horizon = calcHorizon();
  for(int i = 0; i < lineCount; ++i)
  {
    const PossibleObstacleSpots::Scanline& line = thePossibleObstacleSpots.scanlines[i];
    const int lineX = line.xImg;
    const int spotCount = std::min(line.spotCount, MaxTriesPerScanline);
    for(int j = 0; j < spotCount; ++j)
    {
      int y = line.spots[j];
      int yEnd = theImage.height;
      theBodyContour.clipBottom(lineX, yEnd);
      Vector2<int> downSpot;

      //searchDownward may hit the image border. That is ok. Those spots are removed later
      searchDownward(lineX, y, yEnd, downSpot);

      Vector2<int> upSpot = searchUpward(lineX, y, horizon);

      LINE("module:ObstacleSpotProvider:height", downSpot.x, downSpot.y,
        upSpot.x, upSpot.y, 2, Drawings::ps_solid, ColorClasses::red);

      const int height = downSpot.y - upSpot.y;
      if(fullFillsHeightConstraint(height, downSpot))
      {
        possibleSpots.push_back(downSpot);
        CROSS("module:ObstacleSpotProvider:possibleSpots",
              downSpot.x , downSpot.y, 2, 1, Drawings::ps_solid, ColorClasses::black);
        break; //found obstacle for this scanline, continue with next scanline
      }
    }
  }
}

void ObstacleSpotProvider::sortSpots(std::list<Vector2<int> >& possibleSpots)
{
  possibleSpots.sort(
    [](const Vector2<int>& a, const Vector2<int>& b) -> bool
    {
      return a.y > b.y;
    });
}

void ObstacleSpotProvider::clusterSpots(std::list<Vector2<int> >& possibleSpots, ObstacleSpots& spots) const
{
  while(!possibleSpots.empty())
  {
    spots.obstacles.push_back(ObstacleSpots::Obstacle());
    ObstacleSpots::Obstacle& cluster = spots.obstacles.back();
    extractCluster(possibleSpots, cluster);
    ASSERT(cluster.spots.size() > 0);
    calculateCenterOfMass(cluster);
    //if the cluster is closer to the robot than shouldHaveNeighborDistance it's
    //size should be at least two
    if(cluster.spots.size() == 1)
    {
      Vector2<float> pOnField;
      Geometry::calculatePointOnField(cluster.centerOfMass.x, cluster.centerOfMass.y, theCameraMatrix, theCameraInfo, pOnField);
      if(pOnField.abs() < shouldHaveNeighborDistance)
      {//discard cluster.
        spots.obstacles.pop_back();
      }
    }
  }
}

void ObstacleSpotProvider::extractCluster(std::list<Vector2<int> >& possibleSpots, ObstacleSpots::Obstacle& cluster) const
{
  if(!possibleSpots.empty())
  {
    cluster.spots.reserve(possibleSpots.size());//this ensures that no reallocation occurs while adding spots. This is important because otherwise iterators will break
    cluster.spots.push_back(possibleSpots.back());
    possibleSpots.pop_back();
    for(auto it = cluster.spots.begin(); it != cluster.spots.end(); ++it)
    {
      extractFitting(possibleSpots, cluster, *it);
    }
  }
}

void ObstacleSpotProvider::extractFitting(std::list<Vector2<int> >& possibleSpots, ObstacleSpots::Obstacle& cluster,
                      const Vector2<int>& reference) const
{
  int distance = Geometry::calculateLineSize(reference, theCameraMatrix, theCameraInfo, maxNeighborDistance);
  distance = distance * distance; //to be able to compare it with the square distance later on
  for(std::list<Vector2<int> >::iterator it = possibleSpots.begin(); it != possibleSpots.end();)
  {
    if(squareDistance(*it, reference) < distance)
    {
      cluster.spots.push_back(*it);//this HAS TO BE push_back otherwise the loop in extractCluster will not work.
      it = possibleSpots.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void ObstacleSpotProvider::calcClusterBanZone(ObstacleSpots& spots)
{
  //calculate the "center of mass" for each cluster.
  //calculate a ban zone based on that center of mass.
  //the center of mass is simply the avg of all vectors in this cluster
  for(ObstacleSpots::Obstacle& cluster : spots.obstacles)
  {
    const float dist = calculateDistanceTo(cluster.centerOfMass);
    const int widthHalf = int(calculateLineSize(dist, robotWidth) / 2);
    const int height = calculateLineSize(dist, robotHeight);

    cluster.banZoneTopLeft.x = int(cluster.centerOfMass.x - widthHalf);
    cluster.banZoneTopLeft.y = int(cluster.centerOfMass.y - height);
    cluster.banZoneBottomRight.x = int(cluster.centerOfMass.x + widthHalf);
    cluster.banZoneBottomRight.y = int(cluster.centerOfMass.y);
  }
}

inline float ObstacleSpotProvider::calculateDistanceTo(const Vector2<int>& spot)const
{
  Vector2<float> spotOnField;
  Geometry::calculatePointOnField(spot.x, spot.y, theCameraMatrix, theCameraInfo, spotOnField);
  return spotOnField.absFloat();
}
inline float ObstacleSpotProvider::calculateDistanceTo(const Vector2<float>& spot)const
{
  Vector2<float> spotOnField;
  Geometry::calculatePointOnField(spot, theCameraMatrix, theCameraInfo, spotOnField);
  return spotOnField.absFloat();
}

inline int ObstacleSpotProvider::calculateLineSize(const float distance, const int size) const
{
  return int(theCameraInfo.focalLength * size / (distance + FLT_EPSILON));
}

void ObstacleSpotProvider::removeLowerImageBanZone(std::list<Vector2<int> >& possibleSpots, std::vector<BufferedSpot>& spotBuffer)
{
  const size_t limit = std::min((std::size_t)4, spotBuffer.size());
  std::partial_sort(spotBuffer.begin(), spotBuffer.begin() + limit, spotBuffer.end(),
                   [](const BufferedSpot& a, const BufferedSpot& b) -> bool
                   {
                     return a.spotInImg.y > b.spotInImg.y;
                   });


  for(size_t i = 0; i < limit; ++i)
  {
    const BufferedSpot& spot = spotBuffer[i];
    int xLeft, xRight, yTop, yBottom;
    calcBanZone(spot.spotInImg, xLeft, xRight, yTop, yBottom, lowerCameraLegHeight);

      LINE("module:ObstacleSpotProvider:expectedHeight", spot.spotInImg.x, spot.spotInImg.y,
       xRight, yBottom, 2, Drawings::ps_solid,  ColorClasses::green);

    RECTANGLE("module:ObstacleSpotProvider:lowerHandsArea", xLeft, yTop, xRight, yBottom,
              2, Drawings::ps_solid, ColorClasses::blue);

    std::list<Vector2<int> >::iterator s = possibleSpots.begin();
    while(s != possibleSpots.end())
    {
      if(s->x >= xLeft && s->x <= xRight && s->y >= yTop && s->y <= yBottom)
      {
        s = possibleSpots.erase(s);
      }
      else
      {
        ++s;
      }
    }
  }
}

void ObstacleSpotProvider::correctSpots(ObstacleSpots& spots)
{
  for(ObstacleSpots::Obstacle& c : spots.obstacles)
  {
    for(ObstacleSpots::Spot& s : c.spots)
    {
      s = Vector2<int>(theImageCoordinateSystem.toCorrected(s));
    }
    c.centerOfMass = Vector2<int>(theImageCoordinateSystem.toCorrected(c.centerOfMass));
  }
}

void ObstacleSpotProvider::calculateCenterOfMass(ObstacleSpots::Obstacle& cluster) const
{
  ObstacleSpots::Spot sum(0,0); //the center of mass
  for(const ObstacleSpots::Spot& s : cluster.spots)
  {
    sum += s;
  }
  Vector2<> avg(sum);
  avg *= (1.0f/cluster.spots.size());
  cluster.centerOfMass.x = int(avg.x);
  cluster.centerOfMass.y = int(avg.y);
}

int ObstacleSpotProvider::calcHorizon() const
{
  int horizon = static_cast<int>(theImageCoordinateSystem.origin.y);
  if(horizon < 0)
  {
    horizon = 0;
  }
  else if(horizon > theImage.height)
  {
    horizon = theImage.height;
  }
  return horizon;
}

void ObstacleSpotProvider::removeOverlappingBanZones(ObstacleSpots& spots) const
{
  /**
   * This method iterates from bottom to top through the obstacle spots.
   * for ech spot it checks if any spot that is further up than the current one
   * is inside the ban zone of the current spot. If yes it is removed.
   */

  //the spots need to be sorted from bottom to top for this to work
  std::sort(spots.obstacles.begin(), spots.obstacles.end(),
            [](const ObstacleSpots::Obstacle& a, const ObstacleSpots::Obstacle& b) -> bool
            {
              return a.centerOfMass.y > b.centerOfMass.y;
            });

  for(unsigned i = 0; i < spots.obstacles.size(); ++i)
  {
    const int leftX = spots.obstacles[i].banZoneTopLeft.x;
    const int rightX = spots.obstacles[i].banZoneBottomRight.x;
    const int topY = spots.obstacles[i].banZoneTopLeft.y;
    const int bottomY = spots.obstacles[i].banZoneBottomRight.y;
    for(unsigned j = i + 1; j < spots.obstacles.size(); ++j)
    {
      const int x = spots.obstacles[j].centerOfMass.x;
      const int y = spots.obstacles[j].centerOfMass.y;
      if(x >= leftX && x <= rightX && y >= topY && y <= bottomY)
      {//if centerOfMass is inside ban zone
        spots.obstacles.erase(spots.obstacles.begin() + j);//this shifts all indices one to the left
        --j; //this is ok even though j is unsigned because j is at least 1
      }
    }
  }
}

void ObstacleSpotProvider::checkIfTooCloseToBottom(ObstacleSpots& spots) const
{
  const int limit = theImage.height - nonGreenCount - 1;
  for(const ObstacleSpots::Obstacle& o : spots.obstacles)
  {
    for(const ObstacleSpots::Spot& s : o.spots)
    {
      //y<= height is important because obstacles from the lower image have coordinates outside the upper camera image
      if(s.y > limit && s.y < theImage.height)
      {
        spots.obstacles.clear();
        goto end;
      }
    }
  }
  end:;
}

MAKE_MODULE(ObstacleSpotProvider, Perception)
