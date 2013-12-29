/**
* @file Representations/Perception/RegionPercept.cpp
*
* @author jeff
*/

#include "RegionPercept.h"

RegionPercept::Region* RegionPercept::Region::getRootRegion() const
{
  Region* r = root;
  if(r == NULL)
    return NULL;
  while(r->root != NULL)
    r = r->root;
  return r;
}

Vector2<int> RegionPercept::Region::getCenter() const
{
  if(children.size() == 0)
    return getRootRegion()->getCenter();

  Vector2<int> center;
  Segment* segment;
  for(std::vector<Segment*>::const_iterator segmentIter = children.begin();
      segmentIter != children.end();
      segmentIter++)
  {
    segment = *segmentIter;
    center.x += segment->x;
    center.y += segment->y + (segment->length >> 1);
  }
  center.x /= (int)children.size();
  center.y /= (int)children.size();

  return center;
}

int RegionPercept::Region::calcMoment00() const
{
  int m00 = 0;
  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
    m00 += (*seg)->length;

  return m00;
}

int RegionPercept::Region::calcMoment10() const
{
  int m10 = 0;

  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
    m10 += (*seg)->x * (*seg)->length;

  return m10;
}

int RegionPercept::Region::calcMoment01() const
{
  int m01 = 0;

  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
  {
    const int ylo = (*seg)->y;
    const int yhi = (*seg)->y + (*seg)->length;

    m01 += (yhi * (yhi - 1) - ylo * (ylo - 1)) / 2;
  }

  return m01;
}

float RegionPercept::Region::calcCMoment11(int swp_x, int swp_y) const
{
  float cm11 = 0;
  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
  {
    const float yStart = (float)(*seg)->y, yEnd = float((*seg)->y + (*seg)->length);
    const float ySum = GAUSS_SUM(yEnd) - GAUSS_SUM(yStart);
    cm11 += (*seg)->x * ySum - (*seg)->length * (*seg)->x * swp_y -
            swp_x * ySum + (*seg)->length * swp_x * swp_y;
  }

  return cm11;
}

int RegionPercept::Region::calcCMoment20(int swp_x) const
{
  int cm20 = 0;

  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
    cm20 += (*seg)->length * ((*seg)->x - swp_x) * ((*seg)->x - swp_x);

  return cm20;
}


float RegionPercept::Region::calcCMoment02(int swp_y) const
{
  float cm02 = 0;

  for(std::vector<Segment*>::const_iterator seg = children.begin(); seg != children.end(); seg++)
  {
    // sum(x) = sum_(y = seg->y ; y< seg->y + seg->length)(x)
    // cm02 = sum((y-y_s)^2)
    //      = sum(y^2 - 2y*y_s + y_s^2)
    //      = sum(y^2) - 2*y_s*sum(y) + sum(y_s^2)
    //      = sum(y^2) - 2*y_s*sum(y) + length * y_s^2
    const float yStart = (float)(*seg)->y, yEnd = float((*seg)->y + (*seg)->length);
    cm02 += GAUSS_SUM2(yEnd) - GAUSS_SUM2(yStart) - 2 * swp_y * (GAUSS_SUM(yEnd) - GAUSS_SUM(yStart)) + (*seg)->length * swp_y * swp_y;
  }

  return cm02;
}

void RegionPercept::Region::mergeWithRegion(Region* other)
{
  //merge the childs of both regions (sorted)
  std::vector<RegionPercept::Segment*>::iterator regIter1 = children.begin();
  std::vector<RegionPercept::Segment*>::iterator regIter2 = other->children.begin();
  std::vector<RegionPercept::Segment*> mergedChildren;
  while(!(regIter1 == children.end() && regIter2 == other->children.end()))
  {
    if(regIter1 == children.end())
    {
      ASSERT(*regIter2 != NULL);
      mergedChildren.push_back(*regIter2);
      (*regIter2)->region = this;
      regIter2++;
    }
    else if(regIter2 == other->children.end())
    {
      ASSERT(*regIter1 != NULL);
      mergedChildren.push_back(*regIter1);
      regIter1++;
    }
    else
    {
      if(*regIter1 < *regIter2)
      {
        mergedChildren.push_back(*regIter1);
        regIter1++;
      }
      else
      {
        mergedChildren.push_back(*regIter2);
        (*regIter2)->region = this;
        regIter2++;
      }
    }
  }
  children = mergedChildren;

  size += other->size;
  neighborRegions.insert(neighborRegions.end(), other->neighborRegions.begin(), other->neighborRegions.end());
  if(other->min_y < min_y)
    min_y = other->min_y;
  if(other->max_y > max_y)
    max_y = other->max_y;
}

bool RegionPercept::Segment::operator<(Segment* s2)
{
  if(this->x < s2->x)
    return true;
  else if(this->x > s2->x)
    return false;
  else if(this->y < s2->y)
    return true;
  else
    return false;
}

void RegionPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RegionPercept:Regions", "drawingOnImage");
  COMPLEX_DRAWING("representation:RegionPercept:Regions",
  {
    std::vector<Vector2<int> > upperPoints;
    std::vector<Vector2<int> > lowerPoints;
    for(const Region* region = regions; region - regions < regionsCounter; region++)
    {
      if(region->children.size() == 0)
        continue;

      std::vector<Region*>::const_iterator i;

      //Draw Region
      ColorRGBA color = getOnFieldDrawColor(region->color);
      upperPoints.clear();
      lowerPoints.clear();
      Segment* child;

      //create two list of points, the one representing all points belonging to the upper line of
      //the polygon the region covers, the other one represents the lower line
      for(int childIdx = 0; childIdx < (int)region->children.size(); childIdx++)
      {
        child = region->children.at(childIdx);

        if(childIdx == 0 || region->children.at(childIdx - 1)->x != child->x)
          upperPoints.push_back(Vector2<int>(child->x, child->y + 1));
        if(childIdx == (int)region->children.size() - 1 || region->children.at(childIdx + 1)->x != child->x)
          lowerPoints.push_back(Vector2<int>(child->x, child->y + child->length - 1));
      }
      //draw upper line
      for(int idx = 0; idx < (int)upperPoints.size() - 1; idx++)
        LINE("representation:RegionPercept:Regions", upperPoints.at(idx).x, upperPoints.at(idx).y, upperPoints.at(idx + 1).x, upperPoints.at(idx + 1).y, 0, Drawings::ps_solid, color);
      //draw lower line
      for(int idx = 0; idx < (int)lowerPoints.size() - 1; idx++)
        LINE("representation:RegionPercept:Regions", lowerPoints.at(idx).x, lowerPoints.at(idx).y, lowerPoints.at(idx + 1).x, lowerPoints.at(idx + 1).y, 0, Drawings::ps_solid, color);
      //draw left line
      LINE("representation:RegionPercept:Regions", upperPoints.at(0).x, upperPoints.at(0).y, lowerPoints.at(0).x, lowerPoints.at(0).y, 0, Drawings::ps_solid, color);
      //draw right line
      LINE("representation:RegionPercept:Regions", upperPoints.at(upperPoints.size() - 1).x, upperPoints.at(upperPoints.size() - 1).y, lowerPoints.at(lowerPoints.size() - 1).x, lowerPoints.at(lowerPoints.size() - 1).y, 0, Drawings::ps_solid, color);
    }
  });

  DECLARE_DEBUG_DRAWING("representation:RegionPercept:Segments", "drawingOnImage"); // Draws the segment to the image
  COMPLEX_DRAWING("representation:RegionPercept:Segments",
  {
    for(int idx = 0; idx < segmentsCounter; idx++)
    {
      const Segment* i = segments + idx;
      ColorRGBA lineColor = getOnFieldDrawColor(i->color);
      LINE("representation:RegionPercept:Segments", i->x, i->y, i->x, i->y + i->length, 0, Drawings::ps_solid, lineColor);
      LINE("representation:RegionPercept:Segments", i->x - 3, i->explored_min_y, i->x, i->explored_min_y, 0, Drawings::ps_solid, lineColor);
      LINE("representation:RegionPercept:Segments", i->x - 3, i->explored_max_y, i->x, i->explored_max_y, 0, Drawings::ps_solid, lineColor);
    }
  });
}

