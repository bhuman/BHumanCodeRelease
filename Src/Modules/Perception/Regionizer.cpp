/**
* @file Regionizer.cpp
* @author jeff
* @author benny
* @author Florian Maaß
*/

#include "Regionizer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Modify.h"
#include "Platform/BHAssert.h"
#include <algorithm>

Regionizer::Regionizer()
: RegionizerBase()
{
}

void Regionizer::update(RegionPercept& rPercept)
{
  regionPercept = &rPercept;
  pointExplorer.initFrame(&theImage, &theColorReference, exploreStepSize, gridStepSize, skipOffset, minSegSize);
  regionPercept->segmentsCounter = 0;
  regionPercept->regionsCounter = 0;
  regionPercept->gridStepSize = gridStepSize;
  scanVertically();
  //STOP_TIME_ON_REQUEST("scanVert", scanVertically(););
  STOP_TIME_ON_REQUEST("buildRegions", buildRegions(););
}

bool Regionizer::uniteRegions(RegionPercept::Segment* seg1, RegionPercept::Segment* seg2)
{
  ASSERT(seg1->region);
  //we want straight white regions (lines) so don't unite white regions which would not be straight
  const ColorClasses::Color segCol = seg1->color;
  if(segCol == ColorClasses::white)
  {
    ASSERT(seg1->region->children.size() >= 1);
    if(seg1->region->children.at(seg1->region->children.size() - 1)->x == seg2->x)
    {
      return false;
    }
    if(seg1->link)
    {
      Vector2<int> linkDiff = Vector2<int>(seg1->x - seg1->link->x, (seg1->y + seg1->length / 2) - (seg1->link->y + seg1->link->length / 2)),
                   thisDiff = Vector2<int>(seg2->x - seg1->x, (seg2->y + seg2->length / 2) - (seg1->y + seg1->length / 2));
      if(std::abs(linkDiff.angle() - thisDiff.angle()) > maxAngleDiff)
      {
        return false;
      }
    }
  }
  if(seg1->length * regionLengthFactor[segCol] < seg2->length ||
     seg2->length * regionLengthFactor[segCol] < seg1->length)
  {
    return false;
  }
  //seg1 always has a region
  if(!seg2->region)
  {
    RegionPercept::Segment* sWithRegion = seg1,
                          * sWithoutRegion = seg2;
    if((int)sWithRegion->region->children.size() < regionMaxSize)
    {
      sWithRegion->region->children.push_back(sWithoutRegion);
      sWithoutRegion->region = sWithRegion->region;
      sWithRegion->region->size += sWithoutRegion->explored_size;
      if(sWithoutRegion->y < sWithRegion->region->min_y)
      {
        sWithRegion->region->min_y = sWithoutRegion->y;
      }
      if(sWithoutRegion->y + sWithoutRegion->length > sWithRegion->region->max_y)
      {
        sWithRegion->region->max_y = sWithoutRegion->y + sWithoutRegion->length;
      }
      seg2->link = seg1;
      return true;
    }
    ASSERT(seg1->region == seg2->region || !seg2->region);
  }
  //both segments already have a region
  else
  {
    //don't unite two white regions (since we want straight white regions -> lines)
    if(segCol != ColorClasses::white)
    {
      if(seg1->region != seg2->region && ((int)(seg1->region->children.size() + seg2->region->children.size()) < regionMaxSize))
      {
        RegionPercept::Region* oldRegion = seg2->region;
        seg1->region->mergeWithRegion(seg2->region);
        oldRegion->children.clear();
        oldRegion->root = seg1->region;
        seg2->link = seg1;
        return true;
      }
    }
  }
  return false;
}

inline RegionPercept::Segment* Regionizer::addSegment(int x, int y, int length, ColorClasses::Color color)
{
  if(!(regionPercept->segmentsCounter < MAX_SEGMENTS_COUNT - 1))
  {
    return NULL;
  }
  RegionPercept::Segment* seg = regionPercept->segments + regionPercept->segmentsCounter++;
  seg->color = color;
  seg->x = x;
  seg->y = y;
  seg->explored_min_y = y;
  seg->explored_max_y = y + length;
  seg->explored_size = 0;
  seg->length = length;
  seg->region = NULL;
  seg->link = NULL;
  return seg;
}

RegionPercept::Segment* Regionizer::connectToRegions(RegionPercept::Segment* newSegment, RegionPercept::Segment* lastColumPointer, int xDiff)
{
  ASSERT(!newSegment->region);
  if(!lastColumPointer)
  {
    createNewRegionForSegment(newSegment);
    return NULL;
  }
  ASSERT(lastColumPointer->x == newSegment->x - xDiff); //gridStepSize);
  //if lastColumPointer needs to move on
  // _
  //|_|
  //   _    this case
  //  |_|
  //
  while(lastColumPointer->y + lastColumPointer->length < newSegment->explored_min_y && lastColumPointer->x == newSegment->x - xDiff)
  {
    lastColumPointer++;
  }
  //lastColumPointer is now either the first segment in the last line
  //which ends after the start of newSegment or is already in the next line -> return NULL
  if(lastColumPointer->x != newSegment->x - xDiff)
  {
    createNewRegionForSegment(newSegment);
    return NULL;
  }
  std::vector<RegionPercept::Region*> neighborRegions;
  if(lastColumPointer->y + lastColumPointer->length >= newSegment->explored_min_y)
    if(lastColumPointer->y <= newSegment->explored_max_y)
    {
      //TOUCHING
      if(lastColumPointer->color == newSegment->color)
      {
        if(!uniteRegions(lastColumPointer, newSegment))
        {
          neighborRegions.push_back(lastColumPointer->region);
        }
      }
      else
      {
        if(lastColumPointer->region)
        {
          neighborRegions.push_back(lastColumPointer->region);
        }
      }
    }
  //  _
  // | |_
  // |_| |
  //   |_| this case
  RegionPercept::Segment* tmpLastColumPointer;
  tmpLastColumPointer = lastColumPointer;
  while(tmpLastColumPointer->y + tmpLastColumPointer->length <= newSegment->explored_max_y)
  {
    tmpLastColumPointer++;
    //if the lastColumPointer moved to the next colum, move one back and break
    if(tmpLastColumPointer->x != newSegment->x - xDiff)
    {
      if(!newSegment->region)
      {
        if(!createNewRegionForSegment(newSegment))
        {
          return NULL;
        }
      }
      break;
    }
    ASSERT(tmpLastColumPointer->y + tmpLastColumPointer->length >= newSegment->explored_min_y);
    if(tmpLastColumPointer->y <= newSegment->explored_max_y)
    {
      //TOUCHING
      if(tmpLastColumPointer->color == newSegment->color)
      {
        if(!uniteRegions(tmpLastColumPointer, newSegment))
        {
          neighborRegions.push_back(tmpLastColumPointer->region);
        }
      }
      else
      {
        if(tmpLastColumPointer->region)
        {
          neighborRegions.push_back(tmpLastColumPointer->region);
        }
      }
    }
    else
    {
      break;
    }
  }

  if(!newSegment->region)
  {
    if(!createNewRegionForSegment(newSegment))
    {
      return NULL;
    }
  }
  for(std::vector<RegionPercept::Region*>::iterator nb_reg = neighborRegions.begin(); nb_reg != neighborRegions.end(); nb_reg++)
  {
    ASSERT(newSegment->region);
    (*nb_reg)->neighborRegions.push_back(newSegment->region);
    newSegment->region->neighborRegions.push_back(*nb_reg);
  }
  return lastColumPointer;
}

bool Regionizer::createNewRegionForSegment(RegionPercept::Segment* seg)
{
  if(regionPercept->regionsCounter < MAX_REGIONS_COUNT)
  {
    seg->region = regionPercept->regions + regionPercept->regionsCounter++;
    seg->region->color = seg->color;
    seg->region->children.clear();
    seg->region->neighborRegions.clear();
    seg->region->min_y = seg->y;
    seg->region->max_y = seg->y + seg->length;
    seg->region->root = NULL;
    seg->region->size = seg->explored_size;
    seg->region->children.push_back(seg);
    return true;
  }
  return false;
}

void Regionizer::buildRegions()
{
  if (!theCameraMatrix.isValid)
  {
    return;
  }
  int lastx = -1;
  RegionPercept::Segment* firstInColum = NULL, *lastColumPointer = NULL, *newSegment, *lastSegment = NULL;
  for(int i = 0; i < regionPercept->segmentsCounter; i++)
  {
    newSegment = regionPercept->segments + i;
    if(newSegment->color == ColorClasses::green)
    {
      continue;
    }
    //a new colum started
    if(lastx == -1 || lastx != newSegment->x)
    {
      if(lastx == newSegment->x - gridStepSize)
      {
        lastColumPointer = firstInColum;
      }
      else
      {
        lastColumPointer = NULL;
      }
      firstInColum = newSegment;
      lastSegment = NULL;
    }
    lastColumPointer = connectToRegions(newSegment, lastColumPointer, gridStepSize);
    ASSERT(newSegment->region != NULL || regionPercept->regionsCounter >= MAX_REGIONS_COUNT);
    if(newSegment->region == NULL) //MAX_REGIONS_COUNT
    {
      break;
    }
    if(lastSegment != NULL)
    {
      if(newSegment-> y - (lastSegment->y + lastSegment->length) < skipOffset)
      {
        lastSegment->region->neighborRegions.push_back(newSegment->region);
        newSegment->region->neighborRegions.push_back(lastSegment->region);
      }
      else
      {
        lastSegment = NULL;
      }
    }

    lastSegment = newSegment;
    lastx = newSegment->x;
  }
}

void Regionizer::scanVertically()
{
  if (!theCameraMatrix.isValid)
  {
    return;
  }
  const int xStart = gridStepSize - 1 + ((theImage.width) %  gridStepSize) / 2,
      xEnd = theImage.width;
  int yEnd,
      yStart;
  const int yHorizon = std::max(1, theImageCoordinateSystem.fromHorizonBased(Vector2<>()).y); // first line of image might be shitty
  int fBoundary;
  RegionPercept::Segment* newSegment;
  int run_end_y, explored_min_y, explored_max_y, explored_size;
  bool perceptFull = false;
  int x = xStart, y;
  ColorClasses::Color curColor;
  bool ballScanline = false;
  for(; x < xEnd && !perceptFull ; x += gridStepSize / 2) // gridstepsize/2 hack for ballscanlines
  {
    // calculate yEnd based on stopPolygon
    yEnd = theImage.height;
    fBoundary = theFieldBoundary.getBoundaryY(x) - 1; // -1 is the tolerance
    theBodyContour.clipBottom(x, yEnd);
    y = yStart = std::max(yHorizon, fBoundary);
    while(y < yEnd)
    {
      const Image::Pixel* pixel = theImage[y] + x;
      curColor = pointExplorer.getColor(pixel);
      if(ballScanline && CameraInfo::upper == theCameraInfo.camera)
      {
        const int ballYEnd = std::min(yStart + 30, theImage.height - 1); //value 30 determined by empiric
        explored_size = pointExplorer.explorePoint(x, y, curColor, std::max(0, x - gridStepSize/2), ballYEnd, yStart, run_end_y, explored_min_y, explored_max_y);
        if(run_end_y >= ballYEnd)
        {
          break;
        }
      }
      else if(ballScanline)
        break;
      else
        explored_size = pointExplorer.explorePoint(x, y, curColor, std::max(0, x - gridStepSize), yEnd, yStart, run_end_y, explored_min_y, explored_max_y);
      // make use of banZones provided by ObstacleSpots
      int yTemp = y;
      if(curColor == ColorClasses::white)
      {
        for(const ObstacleSpots::Obstacle& cluster : theObstacleSpots.obstacles)
        {
          if((cluster.banZoneBottomRight.x - cluster.banZoneTopLeft.x) > (theImage.width / 2))
          {
            break;
          }
          if(x >= cluster.banZoneTopLeft.x && x <= cluster.banZoneBottomRight.x)
          {
            if(y >= cluster.banZoneTopLeft.y && y <= cluster.banZoneBottomRight.y)
            {
              yTemp = std::max(yTemp, explored_max_y);
              if(ballScanline)
              {
                break;
              }
            }
            else if(y <= cluster.banZoneTopLeft.y && explored_max_y >= cluster.banZoneTopLeft.y)
            {
              explored_max_y = cluster.banZoneTopLeft.y;
              run_end_y = cluster.banZoneTopLeft.y;
              explored_size = (run_end_y - y) * gridStepSize;
            }
            else if(y >= cluster.banZoneBottomRight.y && explored_min_y <= cluster.banZoneBottomRight.y)
            {
              explored_min_y = cluster.banZoneBottomRight.y;
            }
          }
        }
      }
      if(yTemp > y)
      {
        if(yTemp == theImage.height)
        {
          break;
        }
        y = yTemp;
        pixel = theImage[y] + x;
        curColor = pointExplorer.getColor(pixel);
        explored_size = pointExplorer.explorePoint(x, y, curColor, std::max(0, x - gridStepSize), yEnd, y, run_end_y, explored_min_y, explored_max_y);
      }
      // end of using banZones
      if(run_end_y - y >= minSegSize[curColor])
      {
        if(run_end_y > yStart)
        {
          if(y < yStart)
          {
            ASSERT(false); // call Florian
            y = yStart;
            explored_min_y = y;
            explored_size = explored_max_y - explored_min_y;
          }
          newSegment = addSegment(x, y, run_end_y - y, curColor);
          if(!newSegment) //MAX_SEGMENTS_COUNT
          {
            ASSERT(regionPercept->segmentsCounter == MAX_SEGMENTS_COUNT - 1);
            perceptFull = true;
            break;
          }
          newSegment->explored_min_y = explored_min_y;
          newSegment->explored_max_y = explored_max_y;
          newSegment->explored_size = explored_size;
        }
      }
      y = run_end_y;
    }
    ballScanline = !ballScanline;
  }
}
MAKE_MODULE(Regionizer, Perception)
