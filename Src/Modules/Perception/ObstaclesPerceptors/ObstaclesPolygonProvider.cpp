/**
 * @file ObstaclesPolygonProvider.cpp
 *
 *
 * @author Nele Matschull
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#include "ObstaclesPolygonProvider.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPolygonPercept.h"
#include "Debugging/DebugDrawings.h"
#include "ImageProcessing/PixelTypes.h"
#include "Math/Eigen.h"
#include "Math/Geometry.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>
#include <cmath>

MAKE_MODULE(ObstaclesPolygonProvider, perception);

void ObstaclesPolygonProvider::update(ObstaclesPolygonPercept& theObstaclesPolygonPercept)
{
  DECLARE_DEBUG_DRAWING("module:ObstaclesPolygonProvider:obstacles", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPolygonProvider:startPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPolygonProvider:deviation", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPolygonProvider:lines", "drawingOnImage");
  groundNonGreenRegions.reserve(theColorScanLineRegionsHorizontal.scanLines.size());

  groundNonGreenRegions.clear();
  for(size_t i = 0; i < theColorScanLineRegionsHorizontal.scanLines.size(); ++i)
  {
    groundNonGreenRegions.push_back({});
  }
  for(size_t scanLine = 0; scanLine < theColorScanLineRegionsHorizontal.scanLines.size(); ++scanLine)
  {
    std::vector<ScanLineRegion> regions = theColorScanLineRegionsHorizontal.scanLines[scanLine].regions;
    bool lastRegionWasField = true;
    Vector2f first;
    for(size_t region = 0; region < regions.size(); ++region)
    {
      if(regions[region].color != PixelTypes::Color::field)
      {
        if(lastRegionWasField)
          first = Vector2f(regions[region].range.from, theColorScanLineRegionsHorizontal.scanLines[scanLine].y);
        lastRegionWasField = false;
        if(region == regions.size() - 1 || regions[region + 1].color == PixelTypes::Color::field)
        {
          Vector2f second = Vector2f(regions[region].range.to - 1, theColorScanLineRegionsHorizontal.scanLines[scanLine].y);
          groundNonGreenRegions[scanLine].push_back({first, second});
          //LINE("module:ObstaclesPolygonProvider:startPoints", first.x(), first.y(), second.x(), second.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        }
      }
      else
        lastRegionWasField = true;
    }
  }

  //close gaps
  for(size_t i = 0; i + 1 < groundNonGreenRegions.size(); i++)
  {
    if(groundNonGreenRegions[i].size() < 2)
      continue;
    for(auto iter = groundNonGreenRegions[i].begin(); iter != --groundNonGreenRegions[i].end(); iter++)
    {
      std::array<Vector2f, 2> next = *++iter;
      iter--;
      bool up = false;
      bool down = false;
      if(i == 0)
      {
        up = true;
      }
      else
      {
        for(auto iterUp = groundNonGreenRegions[i - 1].begin(); iterUp != groundNonGreenRegions[i - 1].end(); iterUp++)
        {
          if(completeOverlap({(*iter)[1], next[0]}, *iterUp))
          {
            up = true;
            break;
          }
        }
      }
      if(i + 1 == groundNonGreenRegions.size())
      {
        down = true;
      }
      else
      {
        for(auto iterDown = groundNonGreenRegions[i + 1].begin(); iterDown != groundNonGreenRegions[i + 1].end(); iterDown++)
        {
          if(completeOverlap({(*iter)[1], next[0]}, *iterDown))
          {
            down = true;
            break;
          }
        }
      }
      if(up && down)
      {
        Vector2f start = (*iter)[0];
        iter = groundNonGreenRegions[i].erase(iter);
        (*iter)[0] = start;
        iter--;
      }
    }
  }

  //filter the ball out
  if(theBallPercept.status != BallPercept::Status::notSeen)
  {
    Geometry::Circle ball(theBallPercept.positionInImage, theBallPercept.radiusInImage * ballRadiusFactor);
    for(auto scanline = groundNonGreenRegions.begin(); scanline != groundNonGreenRegions.end(); scanline++)
    {
      //only consider non empty lines
      if((*scanline).empty())
        continue;
      //if we are already below the ball we need no longer to search
      if((*scanline).front()[0].y() < theBallPercept.positionInImage.y() - theBallPercept.radiusInImage * ballRadiusFactor)
        break;
      //jump over the lines above the ball
      if((*scanline).front()[0].y() > theBallPercept.positionInImage.y() + theBallPercept.radiusInImage * ballRadiusFactor)
        continue;

      //compute the intersections of the current scanLine an the ball
      Geometry::Line line(Vector2f(0, (*scanline).front()[0].y()), {1, 0});
      Vector2f firstIntercection;
      Vector2f secondIntercection;
      //should always be 2
      if(Geometry::getIntersectionOfLineAndCircle(line, ball, secondIntercection, firstIntercection) != 2)
        assert(true);

      //for all non green regions in this scanLine
      for(auto iter = (*scanline).begin(); iter != (*scanline).end(); iter++)
      {
        //if the intersections are left to the current region we are finished
        if(secondIntercection.x() < (*iter)[0].x())
          break;
        //if the intersections are right to the region we can jump over the region
        if(firstIntercection.x() > (*iter)[1].x())
          continue;

        //if the region is completely inside by the ball
        if(firstIntercection.x() <= (*iter)[0].x() && secondIntercection.x() >= (*iter)[1].x())
        {
          // the region can be deleted
          iter = (*scanline).erase(iter);
        }

        //if the right side of the region is inside the ball
        else if(firstIntercection.x() < (*iter)[1].x() && secondIntercection.x() >= (*iter)[1].x())
        {
          //it should end at the first intersection
          (*iter)[1].x() = firstIntercection.x();
          continue;
        }

        //if the left side of the region is inside the ball
        else if(firstIntercection.x() <= (*iter)[0].x() && secondIntercection.x() >= (*iter)[0].x())
        {
          //it should start at the second intersection
          (*iter)[0].x() = secondIntercection.x();
          //the next regions are behind the ball
          break;
        }

        //if the region overlap the ball completely
        else
        {
          //split the region in the parts before and after the ball
          Vector2f end = (*iter)[1];
          (*iter)[1].x() = firstIntercection.x();
          iter++;
          (*scanline).insert(iter, {secondIntercection, end});
          //the next regions are behind the ball
          break;
        }
      }
    }
  }

  for(auto scanline : groundNonGreenRegions)
  {
    for(auto region : scanline)
      LINE("module:ObstaclesPolygonProvider:startPoints", region[0].x(), region[0].y(), region[1].x(), region[1].y(), 3, Drawings::solidPen, ColorRGBA::blue);
  }

  nonGreenRegions[0] = groundNonGreenRegions;
  nonGreenRegions[1] = groundNonGreenRegions;
  for(auto& region : nonGreenRegions[1]) region.reverse();

  //delete obstacles from the last frame
  theObstaclesPolygonPercept.obstacles.clear();

  bool downwards = false;
  size_t currentIndex = 0;
  std::list<Vector2f> currentPolygon;
  ColorRGBA colors[] = { ColorRGBA::cyan, ColorRGBA::orange, ColorRGBA::green, ColorRGBA::yellow, ColorRGBA::black };
  int color = 0;

  while(true)
  {
    //skip lines without obstacles
    while(currentIndex < nonGreenRegions[downwards].size() && nonGreenRegions[downwards][currentIndex].empty())
    {
      currentIndex++;
    }
    if(currentIndex >= nonGreenRegions[downwards].size())
      break;

    //start a new obstacle
    std::array<Vector2f, 2> currentPoint = nonGreenRegions[downwards][currentIndex].front();
    currentPolygon.clear();
    currentPolygon.push_back(currentPoint[0]);

    //delete used point
    nonGreenRegions[downwards][currentIndex].pop_front();

    //make the obstacle
    short unsuccessfulDirectionChanges = 0;
    downwards = false;
    while(true)
    {
      if((!downwards && currentIndex + 1 >= nonGreenRegions[downwards].size()) || (downwards && currentIndex <= 0) || currentPoint[!downwards] == currentPolygon.front() || unsuccessfulDirectionChanges > 1)
        break;

      //search in the same line for a matching point
      for(auto iter = std::begin(nonGreenRegions[!downwards][currentIndex]); iter != std::end(nonGreenRegions[!downwards][currentIndex]); iter++)
      {
        if(*iter == currentPoint)
          continue;

        //look if there is a line on the same scanline that is in one of the neighboring scanlines connected (gap is completely overlapped) and is on the right side: go to there
        bool found = false;
        if(currentIndex + 1 < groundNonGreenRegions.size())
        {
          for(auto iterUp = std::begin(groundNonGreenRegions[currentIndex + 1]); iterUp != std::end(groundNonGreenRegions[currentIndex + 1]); iterUp++)
          {
            if(((downwards && currentPoint[downwards].x() < (*iter)[!downwards].x()) || (!downwards && currentPoint[downwards].x() > (*iter)[!downwards].x())) && completeOverlap({ currentPoint[downwards], (*iter)[!downwards] }, *iterUp))
            {
              found = true;
              break;
            }
          }
        }

        if(currentIndex != 0)
        {
          for(auto iterDown = std::begin(groundNonGreenRegions[currentIndex - 1]); iterDown != std::end(groundNonGreenRegions[currentIndex - 1]) && !found; iterDown++)
          {
            if(((downwards && currentPoint[downwards].x() < (*iter)[!downwards].x()) || (!downwards && currentPoint[downwards].x() > (*iter)[!downwards].x())) && completeOverlap({ currentPoint[downwards], (*iter)[!downwards] }, *iterDown))
            {
              found = true;
              break;
            }
          }
        }

        //if a point was found
        if(found)
        {
          //save point and invert direction
          downwards = !downwards;
          currentPoint = *iter;
          currentPolygon.push_back(currentPoint[downwards]);
          nonGreenRegions[downwards][currentIndex].erase(iter);
          unsuccessfulDirectionChanges = 0;

          //if we are on the top ore bottom change direction back
          if((currentIndex == 0 && downwards) || (currentIndex + 1 >= nonGreenRegions[downwards].size() && !downwards))
          {
            //invert direction
            downwards = !downwards;
            unsuccessfulDirectionChanges++;

            currentPolygon.push_back(currentPoint[downwards]);

            //remove the point
            for(auto iter = nonGreenRegions[downwards][currentIndex].begin(); iter != nonGreenRegions[downwards][currentIndex].end(); iter++)
            {
              if(*iter == currentPoint)
              {
                nonGreenRegions[downwards][currentIndex].erase(iter);
                break;
              }
            }
          }
          break;
        }
      }

      currentIndex += (downwards ? -1 : 1);

      //search the next line for a matching point
      bool broken = false;
      for(auto iter = std::begin(nonGreenRegions[downwards][currentIndex]); iter != std::end(nonGreenRegions[downwards][currentIndex]); iter++)
      {
        std::array<Vector2f, 2> point = *iter;

        //if the two lines associated to the points (currentPoint and point) are overlapping -> the orthogonal line through one of the endpoints intersects with the other line
        if(overlap(currentPoint, point))
        {
          unsuccessfulDirectionChanges = 0;
          //save the new found point
          currentPoint = point;
          currentPolygon.push_back(currentPoint[downwards]);

          //remove the point and exit loop
          nonGreenRegions[downwards][currentIndex].erase(iter);
          broken = true;
          break;
        }
      }
      /*if(downwards)
        nonGreenRegions[downwards][currentIndex].reverse();*/
      //if the loop was not exit through break or the top or bottom of the image has arrived the bottom or top of the obstacle has arrived
      if(!broken || (currentIndex == 0 && downwards) || (currentIndex + 1 >= nonGreenRegions[downwards].size() && !downwards))
      {
        //invert direction
        downwards = !downwards;
        unsuccessfulDirectionChanges++;

        currentPolygon.push_back(currentPoint[downwards]);
        if(!broken)
          currentIndex += (downwards ? -1 : 1);

        //remove the point
        for(auto iter = nonGreenRegions[downwards][currentIndex].begin(); iter != nonGreenRegions[downwards][currentIndex].end(); iter++)
        {
          if(*iter == currentPoint)
          {
            nonGreenRegions[downwards][currentIndex].erase(iter);
            break;
          }
        }
      }
    }

    if(currentPolygon.size() > 2)
    {
      //simplify the polygon
      Vector2f lastIndex;
      Vector2f index = currentPolygon.back();
      Vector2f nextIndex;
      auto iter = std::cbegin(currentPolygon);
      while(iter != std::cend(currentPolygon))
      {
        lastIndex = index;
        index = *iter;
        if(++iter != std::cend(currentPolygon))
          nextIndex = *iter;
        else
          nextIndex = *std::cbegin(currentPolygon);

        const Geometry::Line line(lastIndex, (nextIndex - lastIndex));
        const float deviation = Geometry::getDistanceToLine(line, index);
        DRAW_TEXT("module:ObstaclesPolygonProvider:deviation", index.x(), index.y(), 8, ColorRGBA::red, deviation);
        if(deviation < maxDeviationFromLine)
        {
          iter = currentPolygon.erase(--iter);
          index = lastIndex;
        }
      }

      //transform to robot coordinates
      bool validTransformation = true;
      std::list<Vector2f> robotPolygon;
      for(Vector2f pointInImage : currentPolygon)
      {
        Vector2f pointInRobot;
        validTransformation &= Transformation::imageToRobot(pointInImage, theCameraMatrix, theCameraInfo, pointInRobot);
        if(!validTransformation)
          break;
        robotPolygon.push_back(pointInRobot);
      }
      if(!validTransformation)
      {
        robotPolygon.clear();
        currentPolygon.clear();
        break;
      }
      //find potential lines
      bool firstIsLine = false;
      std::vector<std::list<Vector2f>::const_iterator> iterators; //list of iterators which point to the end of an edge that is considered as part of a line
      iterators.push_back(robotPolygon.cbegin());
      auto imgIter = currentPolygon.cbegin();
      for(auto iter = robotPolygon.cbegin(); iter != robotPolygon.cend(); iter++, imgIter++)
      {
        //next point of the polygon
        auto nextPoint = robotPolygon.cbegin();
        auto nextPointImg = currentPolygon.cbegin();
        if(iter != --robotPolygon.cend())
        {
          nextPoint = ++iter;
          iter--;

          nextPointImg = ++imgIter;
          imgIter--;
        }

        //bool isLong = std::abs((*nextPointImg - *imgIter).x()) > 50;

        Geometry::Line line(*iter, *nextPoint - *iter);

        //search for a second edge with that this cud form a fieldline
        auto imgIter2 = currentPolygon.cbegin();
        for(auto iter2 = robotPolygon.cbegin(); iter2 != robotPolygon.cend(); iter2++, imgIter2++)
        {
          Vector2f nextPoint2 = robotPolygon.front();
          auto nextPointImg2 = currentPolygon.front();
          if(iter2 != --robotPolygon.cend())
          {
            nextPoint2 = *(++iter2);
            iter2--;

            nextPointImg2 = *(++imgIter2);
            imgIter2--;
          }

          //the same, next and previous line can be skipped
          if(iter2 == iter || nextPoint2 == *iter || iter2 == nextPoint)
            continue;

          float distance1 = Geometry::getDistanceToLine(line, *iter2);
          float distance2 = Geometry::getDistanceToLine(line, nextPoint2);

          Geometry::Line line2(*iter2, nextPoint2 - *iter2);

          float distance3 = Geometry::getDistanceToLine(line2, *iter);
          float distance4 = Geometry::getDistanceToLine(line2, *nextPoint);

          //Todo: robot polygon sometimes lower border
          bool isAtBorder = (*imgIter).x() == 0 || (*imgIter).x() == (theCameraInfo.width - 1) || (*imgIter2).x() == 0 || (*imgIter2).x() == theCameraInfo.width;

          double lengthFactor = lengthWeight({ *iter, *nextPoint }, { *iter2, nextPoint2 });

          bool bothLong = false; //isLong && std::abs((nextPointImg2 - *imgIter2).x()) > 50 && *imgIter != *imgIter2;

          //Todo: is at border: problem if a actual obstacle is an triangle at the border -> too much sorted out
          if(overlap({*iter, *nextPoint}, {*iter2, nextPoint2}) &&
             ((distance1 < lengthFactor * maxFieldLineWidth && (lengthFactor * distance1 > minFieldLineWidth || isAtBorder || bothLong) && distance2 < lengthFactor * maxFieldLineWidth && (lengthFactor * distance2 > minFieldLineWidth || isAtBorder || bothLong)) &&
              (distance3 < lengthFactor * maxFieldLineWidth && (lengthFactor * distance3 > minFieldLineWidth || isAtBorder || bothLong) && distance4 < lengthFactor * maxFieldLineWidth && (lengthFactor * distance4 > minFieldLineWidth || isAtBorder || bothLong))))
          {
            //line found
            std::array<Vector2f, 2> lineInImage;
            validTransformation &= Transformation::robotToImage(*iter, theCameraMatrix, theCameraInfo, lineInImage[0]);
            validTransformation &= Transformation::robotToImage(*nextPoint, theCameraMatrix, theCameraInfo, lineInImage[1]);

            if(validTransformation)
            {
              LINE("module:ObstaclesPolygonProvider:lines", lineInImage[0].x(), lineInImage[0].y(), lineInImage[1].x(), lineInImage[1].y(), 3, Drawings::solidPen, ColorRGBA::red);
              if(nextPoint != robotPolygon.begin())
                iterators.push_back(nextPoint);
              else
                firstIsLine = true;
              break;
            }
          }
        }
      }

      //remove fieldlines from obstacles
      std::vector<std::list<Vector2f>> obstacles;
      obstacles.reserve(iterators.size());

      iterators.push_back(--robotPolygon.cend());
      for(size_t i = 0; i + 1 < iterators.size(); i++)
      {
        //the part of the polygon between edges that belong to a fieldline is obstacle
        std::list<Vector2f> obstacle(iterators[i], iterators[i + 1]);
        obstacles.push_back(obstacle);
      }
      //the first and last part can be combined because of the circular structure of polygons
      if(!firstIsLine)
      {
        obstacles[0].insert(obstacles[0].cbegin(), robotPolygon.back());
        if(iterators.size() > 2)
          obstacles[0].splice(obstacles[0].cbegin(), robotPolygon, iterators[iterators.size() - 2], iterators[iterators.size() - 1]);
      }

      //transform back to image
      for(size_t i = 0; i < obstacles.size(); i++)
      {
        //Todo Problem: First scanline not always at the top
        bool atBorder = false;
        std::vector<Vector2f> obstacleInImage;
        for(auto iter = obstacles[i].cbegin(); iter != obstacles[i].cend(); iter++)
        {
          Vector2f test;
          validTransformation &= Transformation::robotToImage(*iter, theCameraMatrix, theCameraInfo, test);
          obstacleInImage.push_back(test);
          if(test.x() < distanceToBorder || test.y() < distanceToBorder || test.x() > theCameraInfo.width - distanceToBorder || test.y() > theCameraInfo.height - distanceToBorder)
            atBorder = true;
        }
        if(validTransformation && obstacleInImage.size() > 4 && atBorder)
          POLYGON("module:ObstaclesPolygonProvider:obstacles", obstacleInImage.size(), obstacleInImage, 3, Drawings::solidPen, colors[color], Drawings::noBrush, colors[color]);
        color = ++color % 5;
      }
    }
    //theObstaclesPolygonPercept.obstacles.push_back(currentObstacle);
    downwards = false;
    std::vector<Vector2f> polygon(currentPolygon.begin(), currentPolygon.end());
    //POLYGON("module:ObstaclesPolygonProvider:obstacles", currentPolygon.size(), polygon, 3, Drawings::solidPen, colors[color], Drawings::noBrush, colors[color]);
    color = ++color % 5;
  }
}

Vector2f ObstaclesPolygonProvider::orthogonalVector(Vector2f vec) const
{
  if(vec.y() == 0) return { 0, 1 };
  return { 1, -vec.x() / vec.y() };
}

bool ObstaclesPolygonProvider::overlap(std::array<Vector2f, 2> line1, std::array<Vector2f, 2> line2) const
{
  Vector2f orth = orthogonalVector(line1[0] - line1[1]);
  return Geometry::checkIntersectionOfLines(line2[0], line2[1], line1[0] + 100000 * orth, line1[0] - 100000 * orth) || Geometry::checkIntersectionOfLines(line2[0], line2[1], line1[1] + 100000 * orth, line1[1] - 100000 * orth) || Geometry::checkIntersectionOfLines(line1[0], line1[1], line2[1] + 100000 * orth, line2[1] - 100000 * orth);
}

bool ObstaclesPolygonProvider::completeOverlap(std::array<Vector2f, 2> overlapped, std::array<Vector2f, 2> overlapping) const
{
  Vector2f orth = orthogonalVector(overlapped[0] - overlapped[1]);
  return Geometry::checkIntersectionOfLines(overlapping[0], overlapping[1], overlapped[0] + 100000 * orth, overlapped[0] - 100000 * orth) && Geometry::checkIntersectionOfLines(overlapping[0], overlapping[1], overlapped[1] + 100000 * orth, overlapped[1] - 100000 * orth);
}

double ObstaclesPolygonProvider::lengthWeight(std::array<Vector2f, 2> firstLine, std::array<Vector2f, 2> secondLine) const
{
  const double length1 = (firstLine[0] - firstLine[1]).squaredNorm() * lengthWeigthFactor;
  const double length2 = (secondLine[0] - secondLine[1]).squaredNorm() * lengthWeigthFactor;

  double out = -exp(-(length1 + length2)) + 2;
  return out;
}
