/**
* @file GoalPerceptor.cpp
* @author Michel Bartsch
* @author Thomas Münder
*/

#include "GoalPerceptor.h"
#include <algorithm>

void GoalPerceptor::update(GoalPercept& percept)
{
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Scans", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Validation", "drawingOnImage");

  //clear old data
  spots.clear();
  percept.goalPosts.clear();

  if(!theCameraMatrix.isValid)
  {
    return;
  }

  int scanHeight = std::max(1, (int)theImageCoordinateSystem.origin.y);
  scanHeight = std::min(scanHeight, theImage.height-2);
  LINE("module:GoalPerceptor:Spots", 1, scanHeight, theImage.width-1, scanHeight, 1, Drawings::ps_dash, ColorClasses::orange);

  // find goal spots
  findSpots(scanHeight);

  // process goal spots
  verticalColorScanDown();
  verticalColorScanUp();
  calculatePosition(scanHeight);
  validate();
  posting(percept);
}

void GoalPerceptor::findSpots(const int& height)
{
  int start;
  int sum;
  int skipped;

  for(int i = 0; i < theImage.width; i++)
  {
    if(isYellow(i, height))
    {
      start = i;
      sum = 0;
      skipped = 0;
      while (i < theImage.width && skipped < yellowSkipping)
      {
        if (isYellow(i, height))
        {
          sum++;
          skipped = 0;
        }
        else
        {
          skipped++;
        }
        i++;
      }

      if(sum > 0) // do not allow posts with width = 0
      {
        spots.push_back(Spot(start, i-skipped, height));
        CROSS("module:GoalPerceptor:Spots", start, height, 2, 2, Drawings::ps_solid, ColorClasses::green);
        CROSS("module:GoalPerceptor:Spots", i-skipped, height, 2, 2, Drawings::ps_solid, ColorClasses::blue);
      }
    }
  }
}

void GoalPerceptor::verticalColorScanDown()
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    Vector2<int> mid = i->mid;
    Vector2<int> lastMid = Vector2<int>(0, 0);
    int width = i->width;
    int baseY = 0;

    while(mid.x != lastMid.x)
    {
      int noGaps = 2;
      for(baseY = mid.y+1; baseY < theImage.height-1; baseY++)
      {
        if(isYellow(mid.x, baseY))
        {
          noGaps++;
        } else if(noGaps > 1)  {
          noGaps = 0;
        } else {
          baseY -= 2;
          break;
        }
      }
      LINE("module:GoalPerceptor:Scans", mid.x, mid.y, mid.x, baseY, 1, Drawings::ps_solid, ColorClasses::yellow);
      lastMid = mid;
      mid.y = mid.y + (baseY-mid.y)/2;
      noGaps = 2;
      int left = 1;
      for(int x = mid.x; x > 1; x--)
      {
        if(isYellow(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          left = x+2;
          break;
        }
      }
      noGaps = 2;
      width = theImage.width - left;
      for(int x = mid.x; x < theImage.width-1; x++)
      {
        if(isYellow(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          width = x-2 - left;
          break;
        }
      }
      i->widths.push_back(width);
      mid.x = left+width/2;
    }
    i->base = Vector2<int>(mid.x, baseY + 1);
    CROSS("module:GoalPerceptor:Scans", i->base.x, i->base.y, 2, 2, Drawings::ps_solid, ColorClasses::red);
  }
}

void GoalPerceptor::verticalColorScanUp()
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    Vector2<int> mid = i->mid;
    Vector2<int> lastMid = Vector2<int>(0, 0);
    int width = i->width;
    int initialWidth = 0;
    int topY = 0;
    int left = i->mid.x-width/2;
    int right = i->mid.x+width/2;
    int lastLeft;
    int lastRight;
    bool crossbarChecking = false;

    while(mid.y != lastMid.y)
    {
      int noGaps = 2;
      for(topY = mid.y-1; topY > 0; topY--)
      {
        if(isYellow(mid.x, topY))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          topY += 2;
          break;
        }
      }
      LINE("module:GoalPerceptor:Scans", mid.x, mid.y, mid.x, topY, 1, Drawings::ps_solid, ColorClasses::yellow);
      lastMid = mid;
      mid.y = mid.y - (mid.y-topY)/2;
      noGaps = 2;
      lastLeft = left;
      lastRight = right;
      left = 1;
      right = theImage.width-1;
      for(int x = mid.x; x > 1; x--)
      {
        if(isYellow(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          left = x+2;
          break;
        }
      }
      noGaps = 2;
      for(int x = mid.x; x < theImage.width-1; x++)
      {
        if(isYellow(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          right = x-2;
          width = right-left;
          break;
        }
      }
      if(!initialWidth)
      {
        if(lastLeft > 1 && lastRight < theImage.width-2)
        {
          initialWidth = width;
          crossbarChecking = true;
        }
      }
      if(crossbarChecking)
      {
        if(lastLeft-left > initialWidth && abs(right-lastRight) < initialWidth)
        {
          i->top = Vector2<int>(mid.x, topY);
          i->leftRight = GoalPost::Position::IS_RIGHT;
          LINE("module:GoalPerceptor:Scans", mid.x, mid.y, left, mid.y, 1, Drawings::ps_solid, ColorClasses::yellow);
          goto end_vcs_up;
        }
        else if(right-lastRight > initialWidth && abs(left-lastLeft) < initialWidth)
        {
          i->top = Vector2<int>(mid.x, topY);
          i->leftRight = GoalPost::Position::IS_LEFT;
          LINE("module:GoalPerceptor:Scans", mid.x, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorClasses::yellow);
          goto end_vcs_up;
        }
      }
      mid.x = left+width/2;
    }
    i->leftRight = GoalPost::Position::IS_UNKNOWN;
    end_vcs_up:
    i->top = Vector2<int>(mid.x, topY + 1);
    CROSS("module:GoalPerceptor:Scans", i->top.x, i->top.y, 2, 2, Drawings::ps_solid, ColorClasses::orange);
  }
}

void GoalPerceptor::calculatePosition(const int& height)
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    if (i->base.y > theImage.height-5)
    {
      bool matching = false;
      Vector2<> lastPosition;
      if(theCameraInfo.camera == CameraInfo::upper)
      {
        Vector2<int> projection;
        for(unsigned e = 0; e < lastPosts.size(); e++)
        {
          Vector2<> updated = lastPosts[e].position;
          updated = updated.rotate(-theOdometer.odometryOffset.rotation);
          updated -= theOdometer.odometryOffset.translation;
          Geometry::calculatePointInImage(updated, theCameraMatrix, theCameraInfo, projection);
          if(projection.x < i->end && projection.x > i->start)
          {
            Vector2<int> intersection;
            Geometry::Line l1 = Geometry::Line(Vector2<int>(i->start, height), (Vector2<int>(i->end, height) - Vector2<int>(i->start, height)));
            Geometry::Line l2 = Geometry::Line(projection, (i->base - projection));
            Geometry::getIntersectionOfLines(l1, l2, intersection);
            if(intersection.x < i->end && intersection.x > i->start)
            {
              matching = true;
              lastPosition = updated;
            }
          }
        }
      }
      if(matching)
      {
        i->position = lastPosition;
      }
      else
      {
        float distance = Geometry::getDistanceBySize(theCameraInfo, theFieldDimensions.goalPostRadius * 2.f, (float)i->width);
        Vector2<> angle;
        Geometry::calculateAnglesForPoint(Vector2<>(i->mid), theCameraMatrix, theCameraInfo, angle);
        i->position = Vector2<>(std::cos(angle.x), std::sin(angle.x)) * distance;
      }
    }
    else
    {
      Vector2<> pCorrected = theImageCoordinateSystem.toCorrected(i->base);
      Geometry::calculatePointOnField((int)pCorrected.x, (int)pCorrected.y, theCameraMatrix, theCameraInfo, i->position);
    }
  }
}

// probability based evaluation
void GoalPerceptor::validate()
{
  int distanceEvaluation;
  int relationWidthToHeight;
  int minimalHeight;
  int belowFieldBorder;
  int constantWidth;
  int expectedWidth;
  int expectedHeight;
  int distanceToEachOther;
  int matchingCrossbars;

  int height;
  float value;
  float expectedValue;
  float maxDistance = (Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder) - Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder)).abs() * 1.3f;

  for(std::list<Spot>::iterator i = spots.begin(); i != spots.end(); i++)
  {
    height = (i->base - i->top).abs();

    // if goappost is too far away or too near this post gets 0 %
    i->position.abs() > maxDistance || i->position.abs() < theFieldDimensions.goalPostRadius ? distanceEvaluation = 0 : distanceEvaluation = 1;

    //min height
    height < Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, maxDistance) ? minimalHeight = 0 : minimalHeight = 1;

    //if goalpost base is above the fieldborde
    value = (float)theFieldBoundary.getBoundaryY(i->base.x);
    i->base.y > value - (value / 20) ? belowFieldBorder = 1 : belowFieldBorder = 0;

    //if all width of the goalposts are alike
    constantWidth = 1;
    for(int w : i->widths){if(w > i->width * 2) constantWidth = 0;}

    //gaolposts relation of height to width
    value = ((float)height) / i->width;
    expectedValue = theFieldDimensions.goalHeight / (theFieldDimensions.goalPostRadius * 2);
    relationWidthToHeight = (int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50);

    //ditance compared to width
    expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalPostRadius * 2, i->position.abs());
    //clipping with left image limit
    if(i->base.x < (expectedValue / 2))
    {
      expectedValue -= ((expectedValue / 2) - i->base.x);
    }
    //clipping with right image limit
    if(((theImage.width - 1) - i->base.x) < (expectedValue / 2))
    {
      expectedValue -= ((expectedValue / 2) - ((theImage.width - 1) - i->base.x));
    }
    expectedWidth = (int)(100 - (std::abs(expectedValue - i->width) / expectedValue) * 50);

    //ditance compared to height
    expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, i->position.abs());
    //clipping with upper image limit
    if(i->base.y < expectedValue)
    {
      expectedValue -= (expectedValue - i->base.y);
    }
    //clipping with lower image limit
    if(((theImage.height - 1) - i->top.y) < expectedValue)
    {
      expectedValue -= (expectedValue - ((theImage.height - 1) - i->top.y));
    }
    expectedHeight = (int)(100 - (std::abs(expectedValue - height) / expectedValue) * 50);

    distanceToEachOther = quality;
    matchingCrossbars = quality;
    if(spots.size() > 1)
    {
      for(std::list<Spot>::iterator j = spots.begin(); j != spots.end(); j++)
      {
        if(i != j)
        {
          //distance to each other
          value = (float)(i->position - j->position).abs();
          expectedValue = std::abs(theFieldDimensions.yPosLeftGoal) * 2;
          if((int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50) > 75) distanceToEachOther = 75;

          //matching crossbars
          if((i->leftRight == GoalPost::IS_LEFT && j->leftRight == GoalPost::IS_RIGHT) || (i->leftRight == GoalPost::IS_RIGHT && j->leftRight == GoalPost::IS_LEFT)) matchingCrossbars = 75;
        }
      }
    }

    if(relationWidthToHeight < 0)
      relationWidthToHeight *= 3;
    if(expectedWidth < 0)
      expectedWidth *= 3;
    if(expectedHeight < 0)
      expectedHeight *= 3;

    i->validity = ((relationWidthToHeight + expectedWidth + expectedHeight + distanceToEachOther + matchingCrossbars) / 5.0f) * distanceEvaluation * minimalHeight * belowFieldBorder * constantWidth;

    int low = 0;
    int high = 110;
    MODIFY("module:GoalPerceptor:low", low);
    MODIFY("module:GoalPerceptor:high", high);

    if(i->validity < high && i->validity > low){
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 55, 10, ColorClasses::black, "distanceEvaluation: " << distanceEvaluation);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 44, 10, ColorClasses::black, "minimalHeight: " << minimalHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 33, 10, ColorClasses::black, "belowFieldBorder: " << belowFieldBorder);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 22, 10, ColorClasses::black, "constantWidth: " << constantWidth);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 11, 10, ColorClasses::black, "relationWidthToHeight: " << relationWidthToHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y     , 10, ColorClasses::black, "expectedWidth: " << expectedWidth);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 11, 10, ColorClasses::black, "expectedHeight: " << expectedHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 22, 10, ColorClasses::black, "distanceToEachOther: " << distanceToEachOther);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 33, 10, ColorClasses::black, "matchingCrossbars: " << matchingCrossbars);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 44, 10, ColorClasses::black, "validity: " << i->validity);
    }
  }
  spots.sort();
}

void GoalPerceptor::posting(GoalPercept& percept)
{
  lastPosts.clear();
  if(!spots.empty())
  {
    Spot first = spots.back();
    if(first.validity > quality)
    {
      GoalPost p1;
      p1.position = first.leftRight;
      p1.positionInImage = first.base;
      p1.positionOnField = first.position;
      spots.pop_back();
      if(!spots.empty())
      {
        Spot second = spots.back();
        if(second.validity > quality)
        {
          GoalPost p2;
          p2.position = second.leftRight;
          p2.positionInImage = second.base;
          p2.positionOnField = second.position;

          if(p1.positionInImage.x < p2.positionInImage.x)
          {
            p1.position = GoalPost::Position::IS_LEFT;
            p2.position = GoalPost::Position::IS_RIGHT;
          }
          else
          {
            p1.position = GoalPost::Position::IS_RIGHT;
            p2.position = GoalPost::Position::IS_LEFT;
          }
          percept.goalPosts.push_back(p2);
          percept.timeWhenCompleteGoalLastSeen = theFrameInfo.time;
          if(theCameraInfo.camera == CameraInfo::lower) lastPosts.push_back(second);
        }
      }
      percept.goalPosts.push_back(p1);
      percept.timeWhenGoalPostLastSeen = theFrameInfo.time;
      if(theCameraInfo.camera == CameraInfo::lower) lastPosts.push_back(first);
    }
  }
}

inline bool GoalPerceptor::isYellow(const int& x, const int& y)
{
  return theColorReference.isYellow(&theImage[y][x]);
}

MAKE_MODULE(GoalPerceptor, Perception)