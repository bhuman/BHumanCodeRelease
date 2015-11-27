
#include "LineSpotProvider.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>
#include <limits>
#include <deque>
#include <algorithm>
#include <tuple>

using namespace std;

MAKE_MODULE(LineSpotProvider, perception)

void LineSpotProvider::update(LinePercept& linePercept)
{
  using SpotLine = LineSpots::Line;
  using PerceptLine = LinePercept::Line;
  linePercept.lines.clear();

  linePercept.intersections.clear();
  linePercept.circle.found = false;

  for(const SpotLine& line : theLineSpots.lines)
  {
    if(line.belongsToCircle)
    {
      //LinePercept should not contain lines that are on the circle
      continue;
    }
    linePercept.lines.emplace_back();
    PerceptLine& pLine = linePercept.lines.back();
    pLine.midLine = false;
    if(theLineSpots.circleSeen)
    {//check if this is the midline
      Vector2f intersectionA;
      Vector2f intersectionB;
      const int numIntersections = Geometry::getIntersectionOfLineAndCircle(line.line,
                                   theLineSpots.circle, intersectionA, intersectionB);
      if(numIntersections == 2)
      {//if the line intersects the circle twice
        //check if both intersections are between lineStart and lineEnd
        if(isPointOnLine(line, intersectionA) && isPointOnLine(line, intersectionB))
        {
          pLine.midLine = true;
        }
      }
    }

    pLine.first = line.firstField;
    pLine.last = line.lastField;
    pLine.startInImage.x() = static_cast<float>(line.firstImg.x());
    pLine.startInImage.y() = static_cast<float>(line.firstImg.y());
    pLine.endInImage.x() = static_cast<float>(line.lastImg.x());
    pLine.endInImage.y() = static_cast<float>(line.lastImg.y());

    pLine.alpha = Vector2f(pLine.last - pLine.first).rotateLeft().angle();
    pLine.d = Geometry::getDistanceToLine(Geometry::Line(pLine.first, (pLine.last - pLine.first)), Vector2f::Zero());
  }

  if(theLineSpots.circleSeen)
  {
    linePercept.circle.pos = theLineSpots.circle.center;
    linePercept.circle.found = true;
    linePercept.circle.lastSeen = theImage.timeStamp;
  }

  for(const LineSpots::Intersection& i : theLineSpots.intersections)
  {
    linePercept.intersections.emplace_back();
    linePercept.intersections.back().pos = i.pos;
    linePercept.intersections.back().type = (LinePercept::Intersection::IntersectionType) i.type;
    linePercept.intersections.back().dir1 = i.dir1.normalized();
    linePercept.intersections.back().dir2 = i.dir2.normalized();

  }
}

void LineSpotProvider::update(LineSpots& lineSpotsExp)
{
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:verticalEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:width", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:spotEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:realWidth", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:error", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:spotsWithoutBot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:intersections", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:circleNormals", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:clusters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:invalidEdges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineSpotProvider:lowHigh", "drawingOnField");

  lineSpotsExp.lines.clear();
  lineSpotsExp.intersections.clear();
  lineSpotsExp.spotCount = 0;
  lineSpotsExp.circleSeen = false;
  if(!theCameraMatrix.isValid)
    return; //cannot do anything without camera matrix
  //FIXME give attributes to methods instead of accessing them directly
  //STOPWATCH_WITH_PLOT("LineSpotProvider:spots") findPotentialLineSpots();
  scanlinesVert.clear();
  scanlinesVert.resize(thePotentialLineSpots.numOfScanlines);
  for(const PotentialScanlineSpot& spot : thePotentialLineSpots.spots)
  {
    scanlinesVert.at(spot.scanlineIdx).emplace_back();
    ScanlineSpot& last = scanlinesVert.at(spot.scanlineIdx).back();
    last.heightInImg = spot.heightInImg;
    last.spotInField = spot.spotInField;
    last.spotInImg = spot.spotInImg;
    last.usedAsStartingPoint = spot.usedAsStartingPoint;
  }
  if(scanlinesVert.size() > 0)//if spots have been found
  {
    STOPWATCH_WITH_PLOT("LineSpotProvider:removePlayers") removeSpotsInsidePlayers();
    STOPWATCH_WITH_PLOT("LineSpotProvider:lines") findLines(lineSpotsExp);
    STOPWATCH_WITH_PLOT("LineSpotProvider:validate") validateLines(lineSpotsExp);
    STOPWATCH_WITH_PLOT("LineSpotProvider:circles") findCircle(lineSpotsExp);
    STOPWATCH_WITH_PLOT("LineSpotProvider:intersect") findIntersections(lineSpotsExp);
  }
}

void calcVariance(const std::vector<int>& data, float& variance)
{
  //see http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  //NOTE: This code has been duplicated to the PenaltyMarkPerceptor, if you fix
  //      a bug in here, fix it over there as well
  int n = 0;
  float Sum = 0;
  float Sum_sqr = 0;

  for(const int x : data)
  {
    ++n;
    Sum += x;
    Sum_sqr += x * x;
  }
  variance = (Sum_sqr - (Sum * Sum) / n) / (data.size());
}

void LineSpotProvider::validateLines(LineSpots& lineSpots) const
{
  //For each line the hight of all spots should be roughly the same
  //if it is not that is an indication that for the line being made up of
  //spots on a player and a line
  for(vector<Line>::iterator line = lineSpots.lines.begin(); line < lineSpots.lines.end();)
  {
    float variance;
    calcVariance(line->spotInImgHeights, variance);
    if(variance > maxAllowedLineHeightVariance)
    {
      line = lineSpots.lines.erase(line);
    }
    else
    {
      ++line;
    }
  }
}

void LineSpotProvider::addIntersection(LineSpots& ls, LineSpots::Intersection::IntersectionType type,
                                       const Vector2f& intersection, const Vector2f& dir1, const Vector2f& dir2,
                                       const Line& line1, const Line& line2) const
{
  COMPLEX_DRAWING("module:LineSpotProvider:intersections")
  {
    Vector2f pInImg;
    if(Transformation::robotToImage(intersection, theCameraMatrix, theCameraInfo, pInImg))
      DRAWTEXT("module:LineSpotProvider:intersections", pInImg.x(), pInImg.y(), 30,
               ColorRGBA::black, LineSpots::Intersection::getName(type));
  }
  ls.intersections.emplace_back();
  ls.intersections.back().type = type;
  ls.intersections.back().pos = intersection;
  ls.intersections.back().dir1 = dir1;
  ls.intersections.back().dir2 = dir2;
  ls.intersections.back().line1 = line1;
  ls.intersections.back().line2 = line2;
}

void LineSpotProvider::findIntersections(LineSpots& lineSpots)
{
  const float maxIntersectionDistance2 = maxIntersectionDistance / 2.0f;
  for(vector<Line>::iterator line = lineSpots.lines.begin(); line < lineSpots.lines.end(); ++line)
  {
    for(vector<Line>::iterator line2 = line + 1; line2 < lineSpots.lines.end(); ++line2)
    {
      //only continue if the angle between the two lines is roughly 90°
      const float dot = line->line.direction.normalized().dot(line2->line.direction.normalized());
      const float angle = acos(dot);//angle between lines (in rad)
      const float angleDiff = abs(angle - pi_2);
      if(angleDiff > maxAllowedIntersectionAngleDifference)
      {
        continue;
      }
      Vector2f intersection;
      if(Geometry::getIntersectionOfLines(line->line, line2->line, intersection))
      {
        Vector2f lineEndCloser;//the end of line that is closest to the intersection
        Vector2f line2EndCloser;//the end of line that is further away from the intersection
        Vector2f lineEndFurther;
        Vector2f line2EndFurther;
        const float lineDist = getCloserPoint(line->firstField, line->lastField, intersection, lineEndCloser, lineEndFurther);
        const float line2Dist = getCloserPoint(line2->firstField, line2->lastField, intersection, line2EndCloser, line2EndFurther);

        if(isPointOnLine(*line, intersection) && isPointOnLine(*line2, intersection) &&
           lineDist > maxIntersectionDistance2 && line2Dist > maxIntersectionDistance2)
        {
          addIntersection(lineSpots, LineSpots::Intersection::X, intersection, line->line.direction, line2->line.direction, *line, *line2);
        }
        else if(isPointOnLine(*line, intersection) && isPointOnLine(*line2, intersection) &&
               lineDist <= maxIntersectionDistance2 && line2Dist > maxIntersectionDistance2)
        {
          //line2 is horizontal
          Vector2f vertical = lineEndFurther - intersection;
          Vector2f horizontal = line2EndFurther - intersection;
          enforceTIntersectionDirections(vertical, horizontal);
          addIntersection(lineSpots, LineSpots::Intersection::T, intersection, vertical, horizontal, *line2, *line);
        }
        else if(isPointOnLine(*line, intersection) && isPointOnLine(*line2, intersection) &&
                lineDist > maxIntersectionDistance2 && line2Dist <= maxIntersectionDistance2)
        {
          //line is horizontal
          Vector2f vertical = line2EndFurther - intersection;
          Vector2f horizontal = lineEndFurther - intersection;
          enforceTIntersectionDirections(vertical, horizontal);
          addIntersection(lineSpots, LineSpots::Intersection::T, intersection, vertical, horizontal, *line, *line2);
        }
        else if(isPointOnLine(*line, intersection) && lineDist > maxIntersectionDistance2 &&
                (!isPointOnLine(*line2, intersection)) && line2Dist <= maxIntersectionDistance)
        {
          //line is the horizontal line, line2 the vertical line
          Vector2f vertical = line2EndFurther - intersection;
          Vector2f horizontal = lineEndFurther - intersection;
          enforceTIntersectionDirections(vertical, horizontal);
          addIntersection(lineSpots, LineSpots::Intersection::T, intersection, vertical, horizontal, *line, *line2);
        }
        else if(isPointOnLine(*line2, intersection) && line2Dist > maxIntersectionDistance2 &&
                (!isPointOnLine(*line, intersection)) && lineDist <= maxIntersectionDistance)
        {
          //line2 is the horizontal line
          Vector2f vertical = lineEndFurther - intersection;
          Vector2f horizontal = line2EndFurther - intersection;
          enforceTIntersectionDirections(vertical, horizontal);
          addIntersection(lineSpots, LineSpots::Intersection::T, intersection, vertical, horizontal, *line2, *line);
        }
        else if(lineDist <= maxIntersectionDistance && line2Dist <= maxIntersectionDistance)
        {
          const Vector2f dir1 = lineEndFurther - intersection;
          const Vector2f dir2 = line2EndFurther - intersection;
          addIntersection(lineSpots, LineSpots::Intersection::L, intersection, dir1, dir2, *line, *line2);
        }
      }
    }
  }
}

void LineSpotProvider::enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const
{
  Vector2f vertical90 = vertical;
  vertical90.rotate(pi_2); //vertical rotated by +90°
  if(vertical90.angleTo(horizontal) > pi_2)
  {
    horizontal.mirror();
  }
}

bool LineSpotProvider::isPointOnLine(const Line& line, const Vector2f& point) const
{
  //First check if the point is inside the rectangle created by line.first and line.last
  //If it is check if it is close enough to the line to be considered 'on the line'
  return isPointInsideRectangle(line.firstField, line.lastField, point);
  //FIXME remove method!
}

float LineSpotProvider::getCloserPoint(const Vector2f& a, const Vector2f& b, const Vector2f target, Vector2f& closer, Vector2f& further) const
{
  const float aDist = (a - target).squaredNorm();
  const float bDist = (b - target).squaredNorm();
  if(aDist < bDist)
  {
    closer = a;
    further = b;
    return sqrt(aDist);
  }
  closer = b;
  further = a;
  return sqrt(bDist);
}

void LineSpotProvider::findCircle(LineSpots& lineSpots)
{
  /** Idea:
   * (1) For each line:
   *        Calculate the two normals pointing from the center of the line outwards
   *        normalize them to the length of the circle radius
   * (2) Find the biggest cluster of normals
   * (3) If the cluster is big enough it has to be the center of the center circle
   */

  vector<Cluster> clusters;
  for(LineSpots::Line& line : lineSpots.lines)
  {
    //calculate normals
    Vector2f leftNormal = line.line.direction;
    leftNormal.rotateLeft();
    leftNormal.normalize(theFieldDimensions.centerCircleRadius);
    Vector2f rightNormal = leftNormal;
    rightNormal.mirror();

    //shift normals to center
    const Vector2f lineCenter = (line.firstField + line.lastField) / 2.0f;
    leftNormal += lineCenter;
    rightNormal += lineCenter;
    LINE("module:LineSpotProvider:circleNormals", lineCenter.x(), lineCenter.y(), leftNormal.x(), leftNormal.y(), 20, Drawings::dottedPen, ColorRGBA::red);
    LINE("module:LineSpotProvider:circleNormals", lineCenter.x(), lineCenter.y(), rightNormal.x(), rightNormal.y(), 20, Drawings::dottedPen, ColorRGBA::red);
    clusters.emplace_back(&line, leftNormal);
    clusters.emplace_back(&line, rightNormal);
  }

  //hierarchical clustering
  vector<tuple<Cluster*, Cluster*, float>> distancesA;
  vector<tuple<Cluster*, Cluster*, float>> distancesB;
  vector<tuple<Cluster*, Cluster*, float>>* currentDistances = &distancesA;
  vector<tuple<Cluster*, Cluster*, float>>* otherDistances = &distancesB;
  distancesA.reserve(clusters.size() * clusters.size());
  distancesB.reserve(clusters.size() * clusters.size());
  for(size_t i = 0; i < clusters.size(); ++i)
  {
    for(size_t j = i + 1; j < clusters.size(); ++j)
    {
      currentDistances->emplace_back(&clusters[i], &clusters[j], (clusters[i].center - clusters[j].center).squaredNorm());
    }
  }
  while(currentDistances->size() > 1)
  {
    const auto& min = *std::min_element(currentDistances->begin(), currentDistances->end(), compareCluster);
    if(get<2>(min) > maxCenterCircleClusterDist)
    {
      break;
    }
    Cluster* a = get<0>(min);
    Cluster* b = get<1>(min);
    a->lines.insert(a->lines.end(), b->lines.begin(), b->lines.end());
    for(const Vector2f& point : b->normalSpots)
    {
      a->sum += point;
      a->normalSpots.emplace_back(point);
    }
    a->center = a->sum / (float) a->normalSpots.size();
    b->deleted = true;
    otherDistances->clear();
    for(const auto& tup : *currentDistances)
    {
      if(get<0>(tup) != a && get<0>(tup) != b && get<1>(tup) != a && get<1>(tup) != b)
      {
        otherDistances->emplace_back(tup);
      }
    }
    swap(currentDistances, otherDistances);
    for(auto& cluster : clusters)
    {
      if(!cluster.deleted && &cluster != a)
      {
        currentDistances->emplace_back(a, &cluster, (a->center - cluster.center).squaredNorm());
      }
    }
  }

  int foundCluster = -1;
  float minError = numeric_limits<float>::infinity();
  Geometry::Circle bestCircle;
  for(unsigned i = 0; i < clusters.size(); ++i)
  {
    const Cluster& cluster = clusters[i];
    if(!cluster.deleted && cluster.lines.size() >= minCenterCircleEvidence)
    {
      Geometry::Circle circle;
      float error = evaluateCircle(cluster, circle);
      if(error <= maxCenterCircleFittingError && error < minError)
      {
        foundCluster = i;
        bestCircle = circle;
        minError = error;
      }
    }
  }
  if(foundCluster > -1)
  {
    const Cluster& cluster = clusters[foundCluster];
    lineSpots.circle = bestCircle;
    lineSpots.circleSeen = true;
    for(auto& line : cluster.lines)
    {
      line->belongsToCircle = true;
    }
  }
}

float LineSpotProvider::evaluateCircle(const Cluster& cluster, Geometry::Circle& circle)
{
  std::vector<Vector2f> XY;
  for(LineSpots::Line* line : cluster.lines)
  {
    XY.insert(XY.end(), line->spotsInField.begin(), line->spotsInField.end());
  }
  Eigen::MatrixX3f A(XY.size(), 3);
  Eigen::VectorXf B(XY.size());
  float radiusSqr = theFieldDimensions.centerCircleRadius * theFieldDimensions.centerCircleRadius;
  for(unsigned i = 0; i < XY.size(); ++i)
  {
    const Vector2f& point = XY[i];
    A.row(i) = Vector3f(point.x(), point.y(), 1.f);
    B(i) = point.x() * point.x() + point.y() * point.y() - radiusSqr;
  }
  Eigen::Matrix3Xf AT = A.transpose();
  Vector3f V = (AT * A).inverse() * (AT * B);

  circle.center.x() = V.x() / 2.f;
  circle.center.y() = V.y() / 2.f;
  circle.radius = theFieldDimensions.centerCircleRadius;

  float error = 0;
  for(const Vector2f& point : XY)
    error += std::pow(std::abs((circle.center - point).norm() - circle.radius), 2);
  return error / XY.size();
}

Vector2f LineSpotProvider::mean(const std::vector<Vector2f>& vector)
{
  Vector2f m(0.0f, 0.0f);
  for(const Vector2f& point : vector)
    m += point;
  return m / (float) vector.size();
}

bool LineSpotProvider::compareCluster(const tuple<const Cluster*, const Cluster*, float>& a, const tuple<const Cluster*, const Cluster*, float>& b)
{
  return get<2>(a) < get<2>(b);
}

void LineSpotProvider::findLines(LineSpots& lineSpots)
{
  std::vector<Line>& lines = lineSpots.lines;
  while(lines.size() < maxNumLines)
  {
    /**
     * (1) find two adjacent initial points to form a rough guess of a line
     * (2) search to the left and right to find additional spots that fit the line
     *     (2.1) if minElemsPerLine have been found do intermediate regression for a better line fit
     * (3) continue searching left and right to find the remaining spots
     */

    /**NOTE: This algorithm works because the relation of the spots on the scanlines
     * is not broken when transforming from image to field coordinates.
     * I.e. If we have to spots a and b with a.x < b.x and a.y < b.y this will
     * still hold after transforming to the field plane.
     * The distance between the spots will be distorted but the ordering remains
     * the same.
     */

    deque<pair<ScanlineIter, SpotIter>> foundSpots;
    ScanlineIter leftScanline; //leftSpot has been found on this scanline
    ScanlineIter rightScanline; //rightSpot has been found on this scanline
    SpotIter leftSpot;
    SpotIter rightSpot;
    bool parametersNotYetImproved = true; //indicates that the intermediate regression step has not yet occurred
    bool leftEndReached = false;
    bool rightEndReached = false;
    if(getInitialPoints(leftScanline, leftSpot, rightScanline, rightSpot))
    {
      foundSpots.emplace_front(leftScanline, leftSpot);
      foundSpots.emplace_back(rightScanline, rightSpot);
      //initialize line based on the first two spots
      const Vector2f direction = leftSpot->spotInField - rightSpot->spotInField;
      lines.emplace_back(leftSpot->spotInField, direction); //use leftSpot as base
      Line& possibleLine = lines.back();
      //search for additional spots that fit the line
      while((leftScanline > scanlinesVert.begin() || rightScanline < scanlinesVert.end() - 1) &&
            !(leftEndReached && rightEndReached))
      {
        //FIXME duplicate code
        if((!leftEndReached) && leftScanline > scanlinesVert.begin() &&
           getNextLeft(leftScanline, leftSpot, possibleLine)) //one step search to the left
        {
          foundSpots.emplace_front(leftScanline, leftSpot);
        }
        else
        {
          //this bool is used to avoid further calls to getNextLeft() if it returned false once
          leftEndReached = true;
        }
        if((!rightEndReached) && rightScanline < scanlinesVert.end() - 1 &&
           getNextRight(rightScanline, rightSpot, possibleLine)) //one step search to the right
        {
          foundSpots.emplace_back(rightScanline, rightSpot);
        }
        else
        {
          rightEndReached = true;
        }
        if(parametersNotYetImproved && foundSpots.size() >= refitLineAfter)
        {
          //update the line parameters after a minimum number of points has been found
          //this increases the chance to correctly identify spots further away from the initial pair
          updateLine(possibleLine, foundSpots);
          parametersNotYetImproved = false;
        }
      }
      const float dist = (foundSpots.front().second->spotInField -
                          foundSpots.back().second->spotInField).norm(); //FIXME use squaredNorm() to gain some speed
      if(dist >= minLineLength && foundSpots.size() >= minSpotsOnLine)
      {
        //move spots to line and update parameters
        updateLine(possibleLine, foundSpots);
        lineSpots.spotCount += static_cast<unsigned>(foundSpots.size());
        moveSpotsToLine(foundSpots, possibleLine);
      }
      else
      {
        //line not valid, pop candidate from lines
        lines.pop_back();
      }
    }
    else
    {
      break;
      //FIXME maybe do something with the remaining spots?
    }
  }

  ASSERT(lines.size() <= maxNumLines);
}

void LineSpotProvider::moveSpotsToLine(deque<pair<ScanlineIter, SpotIter>>& spots, Line& line)
{
  //line.firstField is set in updateLine
  //line.lastField is set in updateLine
  line.firstImg = spots.front().second->spotInImg;
  line.lastImg = spots.back().second->spotInImg;
  for(auto& p : spots)
  {
    line.spotsInField.push_back(p.second->spotInField);
    line.spotsInImg.push_back(p.second->spotInImg);
    line.spotInImgHeights.emplace_back(p.second->heightInImg);
    p.first->erase(p.second);
  }
  spots.clear();
}

bool LineSpotProvider::compareSpots(pair<ScanlineIter, SpotIter>& i, pair<ScanlineIter, SpotIter>& j)
{
  //we compare the coordinate with the higher distance, this way it works for both
  //perfectly vertical and perfectly horizontal lines
  const float iy = i.second->spotInField.y();
  const float ix = i.second->spotInField.x();
  const float jy = j.second->spotInField.y();
  const float jx = j.second->spotInField.x();
  const float yDist = (iy - jy) * (iy - jy);
  const float xDist = (ix - jx) * (ix - jx);
  if(yDist > xDist)
  {
    return iy < jy;
  }
  else
  {
    return ix < jx;
  }
}

void LineSpotProvider::updateLine(Line& line, deque<pair<ScanlineIter, SpotIter>>& spots) const
{
    //see http://de.wikipedia.org/wiki/Lineare_Regression

  //NOTE: x and y is swapped
  float avgX = 0;
  float avgY = 0;
  for(auto& p : spots)
  {
    const float x = p.second->spotInField.y();
    const float y = p.second->spotInField.x();
    avgX += x;
    avgY += y;
  }
  avgX /= spots.size();
  avgY /= spots.size();
  //y = a + bx
  ASSERT(!std::isnan(avgX));
  ASSERT(!std::isinf(avgX));
  ASSERT(!std::isnan(avgY));
  ASSERT(!std::isinf(avgY));

  float SSxy = 0; //up
  float SSxx = 0; //down
  for(auto& p : spots)
  {
    const float x = p.second->spotInField.y();
    const float y = p.second->spotInField.x();
    const float xx = x - avgX;
    SSxy += xx * (y - avgY);
    SSxx += xx * xx;
  }
  ASSERT(!std::isnan(SSxy));
  ASSERT(!std::isinf(SSxy));
  ASSERT(!std::isnan(SSxx));
  ASSERT(!std::isinf(SSxx));
  const float b = SSxy / (SSxx + 0.00001f); //+ 0.00001 to avoid div by zero
  ASSERT(!std::isnan(b));
  ASSERT(!std::isinf(b));
  const float a = avgY - b * avgX;
  ASSERT(!std::isnan(a));
  ASSERT(!std::isinf(a));

  //update the line by using the points (x0, f(x0)) and (xn, f(xn))
  //Due to the head movement the front() and back() spots in field coordinates are not
  //necessarily the end points of the line. Therefore we have to find the start and end points
  const auto firstLast = std::minmax_element(spots.begin(), spots.end(), compareSpots);
  const float x0 = firstLast.first->second->spotInField.y(); //use y instead of x because x and y are swapped
  const float x1 = firstLast.second->second->spotInField.y();

  CROSS("module:LineSpotProvider:lowHigh", firstLast.first->second->spotInField.x(), firstLast.first->second->spotInField.y(), 20, 2, Drawings::solidPen, ColorRGBA::yellow);
  CROSS("module:LineSpotProvider:lowHigh", firstLast.second->second->spotInField.x(), firstLast.second->second->spotInField.y(), 20, 2, Drawings::solidPen, ColorRGBA::yellow);

  //FIXME this does not work correctly for nearly vertical lines in field coordinates
  line.firstField.x() = a + b * x0; //swap back
  line.firstField.y() = x0;
  line.lastField.x() = a + b * x1;
  line.lastField.y() = x1;
  line.line.base = line.firstField;
  line.line.direction = line.lastField - line.line.base;
}

bool LineSpotProvider::compare(const pair<float, SpotIter>& a, const pair<float, SpotIter>& b)
{
  return a.first < b.first;
}

bool LineSpotProvider::getNextLeft(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const
{
  for(int i = 0; i < scanlineLookahead && currentLine > scanlinesVert.begin(); ++i)
  {
    vector<pair<float, SpotIter>> tempSpots;
    currentLine -= 1;
    for(SpotIter nextSpot = currentLine->begin(); nextSpot != currentLine->end(); ++nextSpot)
    {
      //collect all spots that are close enough to the given line
      const float dist = abs(Geometry::getDistanceToLine(line.line, nextSpot->spotInField));
      if(dist <= maxDistToLine)
      {
        tempSpots.emplace_back(dist, nextSpot);
      }
    }
    //select the closest spot from the list that is reachable without leaving
    //the white
    sort(tempSpots.begin(), tempSpots.end(), compare);
    for(pair<float, SpotIter>& spot : tempSpots)
    {
      if(isWhite(currentSpot->spotInImg, spot.second->spotInImg))
      {
        //if the line between the two spots is mostly white we accept this spot
        //as next spot in the line
        currentSpot = spot.second;
        return true;
      }
    }
  }
  return false;
}

bool LineSpotProvider::getNextRight(ScanlineIter& currentLine, SpotIter& currentSpot, const Line& line) const
{
  //FIXME this method is 99% identical to getNextLeft()!!!!
  for(int i = 0; i < scanlineLookahead && currentLine < scanlinesVert.end() - 1; ++i)
  {
    vector<pair<float, SpotIter>> tempSpots;
    currentLine += 1;
    for(SpotIter nextSpot = currentLine->begin(); nextSpot != currentLine->end(); ++nextSpot)
    {
      //collect all spots that are close enough to the given line
      const float dist = abs(Geometry::getDistanceToLine(line.line, nextSpot->spotInField));
      if(dist <= maxDistToLine)
      {
        tempSpots.emplace_back(dist, nextSpot);
      }
    }
    //select the closest spot from the list that is reachable without leaving
    //the white
    sort(tempSpots.begin(), tempSpots.end(), compare);
    for(pair<float, SpotIter>& spot : tempSpots)
    {
      if(isWhite(currentSpot->spotInImg, spot.second->spotInImg))
      {
        //if the line between the two spots is mostly white we accept this spot
        //as next spot in the line
        currentSpot = spot.second;
        return true;
      }
    }
  }
  return false;
}

bool LineSpotProvider::getInitialPoints(ScanlineIter& leftScanline, SpotIter& leftSpot,
                                        ScanlineIter& rightScanline, SpotIter& rightSpot)
{
  //FIXME this is a very simple educated guess, a more sophisticated one should be
  //implemented
  //search from left to right for connected neighboring points
  for(unsigned i = 0; i < scanlinesVert.size() - 1; ++i)
  {
    if(getConnectedPair(i, i + 1, leftSpot, rightSpot))
    {
      leftScanline = scanlinesVert.begin() + i;
      rightScanline = scanlinesVert.begin() + i + 1;
      return true;
    }
  }
  return false;
}

bool LineSpotProvider::getConnectedPair(const unsigned lineAIdx, const unsigned lineBIdx,
                                        SpotIter& outSpotA, SpotIter& outSpotB)
{
  ASSERT(lineAIdx < scanlinesVert.size());
  ASSERT(lineBIdx < scanlinesVert.size());
  Scanline& leftScanline = scanlinesVert[lineAIdx];
  Scanline& rightScanline = scanlinesVert[lineBIdx];
  if(leftScanline.size() == 0 || rightScanline.size() == 0)
  {
    return false;
  }
  for(SpotIter leftSpot = leftScanline.begin(); leftSpot != leftScanline.end(); ++leftSpot)
  {
    //this spot has been used as starting point before and did not lead to a
    //line, skip it!
    if(leftSpot->usedAsStartingPoint) continue;

    for(SpotIter rightSpot = rightScanline.begin(); rightSpot != rightScanline.end(); ++rightSpot)
    {
      if(rightSpot->usedAsStartingPoint) continue;

      if(isWhite(leftSpot->spotInImg, rightSpot->spotInImg))
      {
        outSpotA = leftSpot;
        outSpotB = rightSpot;
        //remember that we have used these spots
        //if we see them again in a later iteration they did not lead to a valid line
        leftSpot->usedAsStartingPoint = true;
        rightSpot->usedAsStartingPoint = true;
        return true;
      }
    }
  }
  return false;
}

bool LineSpotProvider::isWhite(Vector2i a, Vector2i b) const
{
  Geometry::PixeledLine line(a, b);
  unsigned whiteCount = 0;
  for(const Vector2i& p : line)
  {
    //FIXME is it possible to use pointer magic instead of [y][x]??
    if(theColorTable[theImage[p.y()][p.x()]].is(ColorClasses::white))
    {
      //fixme break as soon as the whiteCoun can no longer be reached
      ++whiteCount;
    }
  }
  return whiteCount / ((float) line.size()) >= whitePercent;
}


void LineSpotProvider::removeSpotsInsidePlayers()
{
  for(Scanline& line : scanlinesVert)
  {
    for(Scanline::iterator spot = line.begin(); spot < line.end();)
    {
      if(isSpotInsidePlayer(spot->spotInImg.x(), spot->spotInImg.y()))
      {
        CROSS("module:LineSpotProvider:spotsWithoutBot", spot->spotInImg.x(), spot->spotInImg.y(), 5, 2, Drawings::solidPen, ColorRGBA::red);
        spot = line.erase(spot);
      }
      else
      {
        CROSS("module:LineSpotProvider:spotsWithoutBot", spot->spotInImg.x(), spot->spotInImg.y(), 5, 2, Drawings::solidPen, ColorRGBA::blue);
        ++spot;
      }
    }
  }
}

bool LineSpotProvider::isSpotInsidePlayer(const int x, const int y) const
{
  for(const PlayersPercept::Player& player : thePlayersPercept.players)
  {
    if(player.detectedFeet)//only use box if the player's feet have been detected
    {
      if(x >= player.x1FeetOnly && x <= player.x2FeetOnly && y >= player.y1 && y <= player.y2)
      {
        return true;
      }
    }
  }
  return false;
}
