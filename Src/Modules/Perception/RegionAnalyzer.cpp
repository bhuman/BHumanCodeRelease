/**
* @file RegionAnalyzer.cpp
* @author jeff
* @author Florian Maaß
*/

#include "RegionAnalyzer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

using namespace std;

#define TOTAL_GROATS 42345645.0f

void RegionAnalyzer::update(LineSpots& otherLineSpots)
{
  lineSpots = &otherLineSpots;
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:Line", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenBelow", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenAbove", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenLeft", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenRight", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:Robots", "drawingOnImage");

  lineSpots->spots.clear();
  lineSpots->nonLineSpots.clear();
  ballSpots->ballSpots.clear();

  if(theCameraMatrix.isValid)
  {
    analyzeRegions();
  }
}

int RegionAnalyzer::getGreenBelow(const RegionPercept::Region* region)
{
  int greenBelow = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->children.begin(); child != region->children.end(); child++)
  {
    if((*child) - theRegionPercept.segments == theRegionPercept.segmentsCounter - 1)
      continue;

    RegionPercept::Segment* next = (*child) + 1;
    const int childEndY = (*child)->y + (*child)->length;

    //find next green segment
    while(next - theRegionPercept.segments < theRegionPercept.segmentsCounter &&
          next->x == (*child)->x &&
          next->color != ColorClasses::green)
      next++;

    if(next - theRegionPercept.segments < theRegionPercept.segmentsCounter && next->x == (*child)->x)
    {
      //implicit due to the if condition and the while condition
      //if(next->color == ColorClasses::green)
      if(next->y - childEndY <= maxLineNghbGreySkip)
      {
        greenBelow += next->length;
        LINE("module:RegionAnalyzer:greenBelow", next->x, next->y, next->x, next->y + next->length, 0, Drawings::ps_solid, ColorClasses::green);
      }
    }
  }
  ASSERT(greenBelow >= 0);
  return greenBelow;
}

int RegionAnalyzer::getGreenAbove(const RegionPercept::Region* region)
{
  int greenAbove = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->children.begin(); child != region->children.end(); child++)
  {
    if((*child) == theRegionPercept.segments)
      continue;

    RegionPercept::Segment* next = (*child) - 1;

    //find previous green segment
    while(next->x == (*child)->x &&
          next > theRegionPercept.segments &&
          next->color != ColorClasses::green)
      next--;

    ASSERT(next >= theRegionPercept.segments);

    if(next->color == ColorClasses::green && next->x == (*child)->x)
    {
      //if(next->color == ColorClasses::green)
      if((*child)->y - (next->y + next->length) <= maxLineNghbGreySkip)
      {
        greenAbove += next->length;
        LINE("module:RegionAnalyzer:greenAbove", next->x, next->y, next->x, (*child)->y, 0, Drawings::ps_solid, ColorClasses::blue);
      }
    }
  }
  ASSERT(greenAbove >= 0);
  return greenAbove;
}

int RegionAnalyzer::getGreenRight(const RegionPercept::Region* region)
{
  const RegionPercept::Segment* nextColumnSegment;
  const int& gridStepSize = theRegionPercept.gridStepSize;
  int greenRight = 0;
  for(vector<RegionPercept::Segment*>::const_iterator seg_iter = region->children.begin();
      seg_iter != region->children.end();
      seg_iter++)
  {
    nextColumnSegment = *seg_iter + 1;
    //find first segment in next column (and skip that ones only used for extendend ball finding).
    while(nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter &&
          nextColumnSegment->x < (*seg_iter)->x + gridStepSize)
      nextColumnSegment++;

    if(nextColumnSegment >= theRegionPercept.segments + theRegionPercept.segmentsCounter ||
       nextColumnSegment->x > (*seg_iter)->x + gridStepSize)
      continue;

    ASSERT(nextColumnSegment->x == (*seg_iter)->x + gridStepSize);

    //find first segment which ends after seg_iter->y
    while(nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter &&
          nextColumnSegment->x == (*seg_iter)->x + gridStepSize &&
          nextColumnSegment->y + nextColumnSegment->length <= (*seg_iter)->y)
      nextColumnSegment++;

    if(nextColumnSegment >= theRegionPercept.segments + theRegionPercept.segmentsCounter ||
       nextColumnSegment->x > (*seg_iter)->x + gridStepSize)
      continue;

    //find first segment touching seg_iter and check from there on whether there are green segments
    while(nextColumnSegment->y < (*seg_iter)->y + (*seg_iter)->length &&
          nextColumnSegment->x == (*seg_iter)->x + gridStepSize &&
          nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter)
    {
      if(nextColumnSegment->color == ColorClasses::green)
      {
        const int start = std::max<int>(nextColumnSegment->y, (*seg_iter)->y);
        const int end = std::min<int>(nextColumnSegment->y + nextColumnSegment->length, (*seg_iter)->y + (*seg_iter)->length);
        ASSERT(end >= start);
        greenRight += end - start;
        LINE("module:RegionAnalyzer:greenRight", nextColumnSegment->x, start, nextColumnSegment->x, end, 0, Drawings::ps_solid, ColorClasses::blue);
      }
      nextColumnSegment++;
    }
  }
  ASSERT(greenRight >= 0);
  return greenRight;
}


int RegionAnalyzer::getGreenLeft(const RegionPercept::Region* region)
{
  const RegionPercept::Segment* lastColumnSegment;
  const int& gridStepSize = theRegionPercept.gridStepSize;
  int greenLeft = 0;
  for(vector<RegionPercept::Segment*>::const_iterator seg_iter = region->children.begin();
      seg_iter != region->children.end();
      seg_iter++)
  {
    lastColumnSegment = *seg_iter - 1;
    //find first segment in previous column (and skip that ones only used for extendend ball finding).
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->x > (*seg_iter)->x - gridStepSize)
      lastColumnSegment--;

    if(lastColumnSegment < theRegionPercept.segments ||
       lastColumnSegment->x < (*seg_iter)->x - gridStepSize)
      continue;

    ASSERT(lastColumnSegment->x == (*seg_iter)->x - gridStepSize);

    //find first segment which begins before seg_iter->y + set_iter->length
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->x == (*seg_iter)->x - gridStepSize &&
          lastColumnSegment->y >= (*seg_iter)->y + (*seg_iter)->length)
      lastColumnSegment--;

    if(lastColumnSegment < theRegionPercept.segments ||
       lastColumnSegment->x < (*seg_iter)->x - gridStepSize)
      continue;

    //find first segment touching seg_iter and check from there on whether there are green segments
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->y + lastColumnSegment->length > (*seg_iter)->y &&
          lastColumnSegment->x == (*seg_iter)->x - gridStepSize)
    {
      if(lastColumnSegment->color == ColorClasses::green)
      {
        const int start = max<int>(lastColumnSegment->y, (*seg_iter)->y);
        const int end = min<int>(lastColumnSegment->y + lastColumnSegment->length, (*seg_iter)->y + (*seg_iter)->length);
        ASSERT(end >= start);
        greenLeft += end - start;
        LINE("module:RegionAnalyzer:greenLeft", lastColumnSegment->x, start, lastColumnSegment->x, end, 0, Drawings::ps_solid, ColorClasses::blue);
      }
      lastColumnSegment--;
    }
  }
  ASSERT(greenLeft >= 0);
  return greenLeft;
}

bool RegionAnalyzer::isLine(const RegionPercept::Region* region, float& direction, LineSpots::LineSpot& spot)
{
  bool ret = true;
  if(region->size < minLineSize ||
     (region->children.size() < (size_t) minLineSegmentCount &&
      region->size < minLineSingleSegmentSize))
  {
    COMPLEX_DRAWING("module:RegionAnalyzer:Line",
    {
      Vector2<int> c = region->getCenter();
      if(region->size < minLineSize)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x, c.y, 150, ColorClasses::black, region->size << "<s");
      }
      if(region->children.size() < (size_t) minLineSegmentCount)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x, c.y - 5, 150, ColorClasses::black, (unsigned) region->children.size() << "<c");
      }
    });

    ret = false;
  }

  int neighborNoneSize = 0;
  for(vector<RegionPercept::Region*>::const_iterator neighbor = region->neighborRegions.begin(); neighbor != region->neighborRegions.end(); neighbor++)
  {
    switch((*neighbor)->color)
    {
    case ColorClasses::none:
      neighborNoneSize += (*neighbor)->size;
      break;
    }
  }

  if(neighborNoneSize > maxLineNghbNoneSize ||
     ((float) neighborNoneSize / region->size) > maxLineNghbNoneRatio)
  {
    COMPLEX_DRAWING("module:RegionAnalyzer:Line",
    {
      Vector2<int> c = region->getCenter();
      if(neighborNoneSize > maxLineNghbNoneSize)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorRGBA(255, 0, 0), neighborNoneSize);
        ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x, c.y - 5, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      }
      if((float) neighborNoneSize / region->size > maxLineNghbNoneRatio)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x + 5, c.y, 150, ColorRGBA(255, 0, 0), neighborNoneSize / (float) region->size);
        ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x + 5, c.y, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      }

    });
    ret = false;
  }

  //calculate mass centroid axis and centroid from moments
  const int m00 = region->calcMoment00(),
            m10 = region->calcMoment10(),
            m01 = region->calcMoment01();
  const int xs = m10 / m00,
            ys = m01 / m00;
  const float m20 = (float) region->calcCMoment20(xs),
              m02 = (float) region->calcCMoment02(ys),
              m11 = (float) region->calcCMoment11(xs, ys);
  const float cm20 = m20 / m00,
              cm02 = m02 / m00;
  CROSS("module:RegionAnalyzer:Line", xs, ys, 2, 2, Drawings::ps_solid, ColorClasses::black);
  spot.xs = xs;
  spot.ys = ys;
  float alpha = 0;
  if(cm20 != cm02)
  {
    //for an (yet) unknown reason we need to add an offset of 90 degrees
    alpha = .5f * atan2(-2 * ((float) m11), (float) (m02 - m20)) + (float) pi_2;
    spot.alpha = alpha;
  }
  else
  {
    spot.alpha = TOTAL_GROATS;
    ret = false;
  }

  bool verticalRegion = false;
  if(alpha > pi_2 - pi_4 && alpha < pi_2 + pi_4)
  {
    verticalRegion = true;
    CROSS("module:RegionAnalyzer:Line", xs, ys, 2, 2, Drawings::ps_solid, ColorClasses::green);
  }

  if(verticalRegion)
  {
    int greenLeftSize = getGreenLeft(region);
    int greenRightSize = getGreenRight(region);

    if(greenLeftSize < minLineNghbGreenSideSize || greenRightSize < minLineNghbGreenSideSize)
    {
      COMPLEX_DRAWING("module:RegionAnalyzer:Line",
      {
        Vector2<int> c = region->getCenter();
        if(greenLeftSize < minLineNghbGreenSideSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x - 5, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x - 3, c.y + 5, 150, ColorClasses::black, greenLeftSize);
        }
        if(greenRightSize < minLineNghbGreenSideSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x + 5, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::black, greenRightSize);
        }
      });
      ret = false;
    }
  }
  else
  {
    int greenAboveSize = getGreenAbove(region);
    int greenBelowSize = getGreenBelow(region);

    if(greenAboveSize < minLineNghbGreenAboveSize ||
       greenBelowSize < minLineNghbGreenBelowSize)
    {
      COMPLEX_DRAWING("module:RegionAnalyzer:Line",
      {
        Vector2<int> c = region->getCenter();
        if(greenAboveSize < minLineNghbGreenAboveSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x, c.y - 5, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::green, greenAboveSize);
        }
        if(greenBelowSize < minLineNghbGreenBelowSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y - 5, c.x, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::green, greenBelowSize);
        }
      });
      ret = false;
    }

  }

  //cut alphaLen and alphaLen2 to the maximum distance a point has to
  //the mass centroid axis
  const Vector2<> achse1(cos(alpha), sin(alpha)),
        achse2(cos(alpha + pi_2), sin(alpha + pi_2));
  float len_max = 0,
        len_min = 0,
        len_max2 = 0,
        len_min2 = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->children.begin(); child != region->children.end(); child++)
  {
    const RegionPercept::Segment* seg = *child;
    const Vector2<> childRelToSwp(float(seg->x - spot.xs), float(seg->y - spot.ys)),
          child2RelToSwp(float(seg->x - spot.xs), float(seg->y + seg->length - spot.ys));

    const float dist = childRelToSwp * achse1;
    if(dist > len_max)
      len_max = dist;
    else if(dist < len_min)
      len_min = dist;

    const float dist2 = child2RelToSwp * achse1;
    if(dist2 > len_max)
      len_max = dist2;
    else if(dist2 < len_min)
      len_min = dist2;

    const float dist3 = childRelToSwp * achse2;
    if(dist3 > len_max2)
      len_max2 = dist3;
    else if(dist3 < len_min2)
      len_min2 = dist3;

    const float dist4 = child2RelToSwp * achse2;
    if(dist4 > len_max2)
      len_max2 = dist4;
    else if(dist4 < len_min2)
      len_min2 = dist4;

  }
  spot.alphaLen = abs(len_min) < len_max ? abs(len_min) : len_max;
  spot.alphaLen2 = abs(len_min2) < len_max2 ? abs(len_min2) : len_max2;
  if(spot.alphaLen2 == 0)
    spot.alphaLen2 = (float) theRegionPercept.gridStepSize;
  if(spot.alphaLen == 0) //I don't think this can happen, but.....
    spot.alphaLen = (float) theRegionPercept.gridStepSize;

  spot.p1 = Vector2<int>((int)(xs + cos(spot.alpha) * len_max), (int)(ys + sin(spot.alpha) * len_max));
  spot.p2 = Vector2<int>((int)(xs + cos(spot.alpha) * len_min), (int)(ys + sin(spot.alpha) * len_min));
  direction = alpha;

  return ret;
}

void RegionAnalyzer::analyzeRegions()
{
  LineSpots::LineSpot lineSpot;

  for(const RegionPercept::Region* region = theRegionPercept.regions; region - theRegionPercept.regions < theRegionPercept.regionsCounter; region++)
  {
    if(region->children.size() == 0)
      continue;

    if(region->color == ColorClasses::orange)
    {
      Vector2<int> c = region->getCenter();
      ballSpots->addBallSpot(c.x, c.y);
    }

    if(region->color == ColorClasses::white)
    {
      float dir;
      if(isLine(region, dir, lineSpot))
      {
        lineSpots->spots.push_back(lineSpot);
      }
      else
      {
        if(lineSpot.alpha != TOTAL_GROATS)
        {
          LINE("module:RegionAnalyzer:Robots", lineSpot.xs, lineSpot.ys, lineSpot.xs + (int)(cos(lineSpot.alpha + pi_2) * lineSpot.alphaLen2), lineSpot.ys + (int)(sin(lineSpot.alpha + pi_2)*lineSpot.alphaLen2), 0, Drawings::ps_solid, ColorClasses::black);
          ARROW("module:RegionAnalyzer:Robots", lineSpot.p1.x, lineSpot.p1.y, lineSpot.p2.x, lineSpot.p2.y, 0, Drawings::ps_solid, ColorClasses::black);
          while(lineSpot.alpha > pi)
            lineSpot.alpha -= 2 * pi;
          while(lineSpot.alpha < -pi)
            lineSpot.alpha += 2 * pi;

          if(region->size > minRobotRegionSize && abs(abs(lineSpot.alpha) - pi_2) < maxRobotRegionAlphaDiff && lineSpot.alphaLen / lineSpot.alphaLen2 > minRobotWidthRatio)
          {
            Vector2<int> c = lineSpot.p1.y > lineSpot.p2.y ? lineSpot.p1 : lineSpot.p2;//region->getCenter();
            Vector2<int> c2 = lineSpot.p1.y > lineSpot.p2.y ? lineSpot.p2 : lineSpot.p1;//region->getCenter();
            CROSS("module:RegionAnalyzer:Robots", c.x, c.y, 4, 2, Drawings::ps_solid, ColorClasses::red);
            CROSS("module:RegionAnalyzer:Robots", c2.x, c2.y, 4, 2, Drawings::ps_solid, ColorClasses::red);
            DRAWTEXT("module:RegionAnalyzer:Robots", c.x, c.y, 15, ColorClasses::black, region->size);
            LineSpots::NonLineSpot spot;
            spot.p1 = c;
            spot.p2 = c2;
            spot.size = region->size;
            lineSpots->nonLineSpots.push_back(spot);
          }
        }
      }
    }
  }
}
MAKE_MODULE(RegionAnalyzer, Perception)
