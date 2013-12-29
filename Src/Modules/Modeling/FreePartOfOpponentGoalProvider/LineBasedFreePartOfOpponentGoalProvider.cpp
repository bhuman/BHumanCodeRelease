/**
 * @author Carsten Könemann
 */

#include "LineBasedFreePartOfOpponentGoalProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include <cmath>
#include <algorithm>

using namespace std;

bool comp(pair<int,int> i,pair<int,int> j) { return (i.second > j.second);};


LineBasedFreePartOfOpponentGoalProvider::LineBasedFreePartOfOpponentGoalProvider()
{
  // initially none of the cells on the opponent ground line is occupied
  for(int i = 0; i < NUM_OF_CELLS; i++)
  {
    freePartsOfGoal[i].first = 0;
  }

  initialized = false;
}

void LineBasedFreePartOfOpponentGoalProvider::update(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel)
{
  if (!initialized)
  {
    /* setup */
    // dimensions of opponent goalline
    leftOppGoalPostAbs  = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal);
    rightOppGoalPostAbs = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal);
    oppGoallineRange = Range<>(rightOppGoalPostAbs.y, leftOppGoalPostAbs.y);
    widthOfOppGoal = theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal;
    cellOnOppGoallineWidth = widthOfOppGoal / NUM_OF_CELLS;
    initialized = true;

    /** calculating the centes of the cells in fieldcoordinates*/
    for(int i = 0; i < NUM_OF_CELLS; i++)
    {
      centersOnField[i] = Vector2<>(rightOppGoalPostAbs.x, rightOppGoalPostAbs.y + cellOnOppGoallineWidth * i - cellOnOppGoallineWidth/2);
    }
  }

  int xMin;
  int xMax = theCameraInfo.width - 1;
  int yMin = (int)std::max(theImageCoordinateSystem.origin.y + theImageCoordinateSystem.rotation.c[1].y * 25, 0.f);
  int yMax = theCameraInfo.height - 1;

  // absolute positions of the corners of the camera image on the field
  Vector2<> cameraRange[4];
  float cameraShrinkFactorInv = 1 - cameraShrinkFactor;
  yMax = static_cast<int>(yMax * cameraShrinkFactor);
  yMin = static_cast<int>(yMin ? yMin * (1 + cameraShrinkFactorInv) : yMax * cameraShrinkFactorInv);
  xMin = static_cast<int>(xMax * cameraShrinkFactorInv);
  xMax = static_cast<int>(xMax * cameraShrinkFactor);

  cameraRange[0] = projectOnField(xMin, yMax);
  cameraRange[1] = projectOnField(xMin, yMin);
  cameraRange[2] = projectOnField(xMax, yMin);
  cameraRange[3] = projectOnField(xMax, yMax);

  /** Adding all matching Line Segments to the FreePart */
  for (vector<LinePercept::LineSegment >::const_iterator it = theLinePercept.rawSegs.begin(); it != theLinePercept.rawSegs.end(); ++it)
  {
    Vector2<> absFirstLine = Geometry::relative2FieldCoord(theRobotPose,static_cast<float>(it->p1.x), static_cast<float>(it->p1.y));
    Vector2<> absLastLine = Geometry::relative2FieldCoord(theRobotPose, static_cast<float>(it->p2.x), static_cast<float>(it->p2.y));

    addLineToFreePart(absFirstLine, absLastLine);
  }




  /* calculate which free part is the largest / best */
  int currentFreePartSize = 0, currentFreePartFirst = -1;
  /** highest (left side when facing the opponent goal) cell of the largest free part of opponent goal */
  int bestFreePartLeft;

  /** lowest (right side when facing the opponent goal) cell of the largest free part of opponent goal */
  int bestFreePartRight;

  /** all currentFreeParts first = beginning of the FreeParts, second = number of FreeParts */
  vector<pair<int, int > > freeParts;

  /** freeParts can not be greater than the half of the number of Cells */
  freeParts.reserve(NUM_OF_CELLS/2);

  /** Calculating of all FreeParts*/
  for(int i = 0; i < NUM_OF_CELLS; ++i)
  {
    if(freePartsOfGoal[i].first > likelinessThresholdMin)
    {
      if (currentFreePartFirst < 0)
      {
        currentFreePartFirst = i;
      }

      currentFreePartSize++;
    }
    else if (currentFreePartSize > 0)
    {
        freeParts.push_back(pair<int, int>(currentFreePartFirst, currentFreePartSize));
        currentFreePartFirst = -1;
        currentFreePartSize = 0;
    }
  }

  /** Adding the last FreePart*/
  if (currentFreePartSize > 0)
  {
        freeParts.push_back(pair<int, int>(currentFreePartFirst, currentFreePartSize));
  }

  /** preparing the bestFreePartsOfGoal for the freePartOfOpponentGoalModel*/
  if(freeParts.size() > 0)
  {
    sort(freeParts.begin(), freeParts.end(), comp);

    bestFreePartLeft = freeParts.front().second + freeParts.front().first;
    bestFreePartRight = freeParts.front().first;

    bestFreePartLeft -= 2;
    bestFreePartRight += 1;

    /* calculate relative position to ends of free part */
    Vector2<> leftEndOfFreePartAbs  = Vector2<>(
      (float) theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosRightGoal + (bestFreePartLeft + 1) * cellOnOppGoallineWidth
      );
    Vector2<> rightEndOfFreePartAbs = Vector2<>(
      (float) theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosRightGoal + bestFreePartRight * cellOnOppGoallineWidth
      );
    freePartOfOpponentGoalModel.leftEnd  = Geometry::fieldCoord2Relative(theRobotPose, leftEndOfFreePartAbs);
    freePartOfOpponentGoalModel.rightEnd = Geometry::fieldCoord2Relative(theRobotPose, rightEndOfFreePartAbs);
    freePartOfOpponentGoalModel.valid = true;
  }
  else
  {
    freePartOfOpponentGoalModel.valid = false;
  }

  /** aging of the freePartsOfGoal */
  for (int i = 0; i < NUM_OF_CELLS; ++i)
  {
    /** when the cellTTL is reached, the Cell is set to zero*/
    if (freePartsOfGoal[i].second + cellTTL < theFrameInfo.time && freePartsOfGoal[i].first > 0)
    {
      freePartsOfGoal[i].first = 0;
    }
    else
    {
      /** when the ageRate is reached and the corresponding Line was in sight but not recognised,
       *  the cell will be age by the agingFactor
       */
      if(freePartsOfGoal[i].first > 0 &&
        freePartsOfGoal[i].second + ageRate <= theFrameInfo.time &&
        Geometry::isPointInsideConvexPolygon(cameraRange, 4, centersOnField[i]))
      {
        freePartsOfGoal[i].first = static_cast<unsigned int>(freePartsOfGoal[i].first * agingFactor);
        freePartsOfGoal[i].second = theFrameInfo.time;
      }
    }
  }

  /* draw absolute position of free part into worldstate, see draw()-function below */
  DECLARE_DEBUG_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnFieldImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:SeenGoalLineOnField", "drawingOnField");
  COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField", drawingOnField(freePartOfOpponentGoalModel));
  COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnFieldImg", drawingOnImage(freePartOfOpponentGoalModel));
  COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:SeenGoalLineOnField", drawingSeenGoalLineOnField(cameraRange));
}

Vector2<> LineBasedFreePartOfOpponentGoalProvider::projectOnField(int x, int y)
{
  Vector2<> pointOnField;
  Geometry::calculatePointOnField(x, y, theCameraMatrix, theCameraInfo, pointOnField);
  pointOnField = Geometry::relative2FieldCoord(theRobotPose, pointOnField);
  return pointOnField;
}

void LineBasedFreePartOfOpponentGoalProvider::drawingSeenGoalLineOnField(const Vector2<> *cameraRange)
{
  for (int i = 0; i < NUM_OF_CELLS; ++i)
  {
    if(Geometry::isPointInsideConvexPolygon(cameraRange,4,centersOnField[i]))
    {
      COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:SeenGoalLineOnField",{
        LINE(
          "module:LineBasedFreePartOfOpponentGoalProvider:SeenGoalLineOnField",
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + i * cellOnOppGoallineWidth,
          theFieldDimensions.xPosOpponentGroundline,
          theFieldDimensions.yPosRightGoal + (i + 1)*cellOnOppGoallineWidth,
          50,
          Drawings::ps_solid,
          ColorClasses::red
        );
      });
    }
  }
}

void LineBasedFreePartOfOpponentGoalProvider::drawingOnField(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel)
{
  COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField",
  {
    // draw goal line black
    LINE(
      "module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField",
      theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosRightGoal,
      theFieldDimensions.xPosOpponentGroundline,
      theFieldDimensions.yPosLeftGoal,
      50,
      Drawings::ps_solid,
      ColorClasses::black
    );
    if(freePartOfOpponentGoalModel.valid)
    {
      Vector2<> p1,p2;
      p1 = Geometry::relative2FieldCoord(theRobotPose,freePartOfOpponentGoalModel.leftEnd);
      p2 = Geometry::relative2FieldCoord(theRobotPose,freePartOfOpponentGoalModel.rightEnd);
      LINE(
            "module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField",
            p1.x-50,p1.y,p2.x-50,p2.y,50,Drawings::ps_solid,ColorClasses::blue
          );
    }
    for(int i = 0; i < NUM_OF_CELLS; i++)
    {
      // draw all free parts in green shades
      LINE(
        "module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnField",
        theFieldDimensions.xPosOpponentGroundline,
        theFieldDimensions.yPosRightGoal + i * cellOnOppGoallineWidth,
        theFieldDimensions.xPosOpponentGroundline,
        theFieldDimensions.yPosRightGoal + (i + 1)*cellOnOppGoallineWidth,
        50,
        Drawings::ps_solid,
        ColorRGBA(0,255,0, static_cast<unsigned char>(255 / likelinessThresholdMax * freePartsOfGoal[i].first))
      );
    }



  });
}

void LineBasedFreePartOfOpponentGoalProvider::drawingOnImage(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel)
{
  if(freePartOfOpponentGoalModel.valid)
  {
    COMPLEX_DRAWING("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnFieldImg",
    {
      Vector2<> left(freePartOfOpponentGoalModel.leftEnd.x,
        freePartOfOpponentGoalModel.leftEnd.y);
      Vector2<> right(freePartOfOpponentGoalModel.rightEnd.x,
        freePartOfOpponentGoalModel.rightEnd.y);
      Vector2<int> p1;
      Vector2<int> p2;

      if(Geometry::calculatePointInImage(left, theCameraMatrix, theCameraInfo, p1) &&
      Geometry::calculatePointInImage(right, theCameraMatrix, theCameraInfo, p2))
      {
        Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedLinearized(p1);
        Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedLinearized(p2);
        LINE("module:LineBasedFreePartOfOpponentGoalProvider:FreePartOnFieldImg", uncor1.x,
        uncor1.y, uncor2.x, uncor2.y, 3, Drawings::ps_solid, ColorClasses::blue);
      }
    });
  }
}

void LineBasedFreePartOfOpponentGoalProvider::addLineToFreePart(Vector2<> &firstLine, Vector2<> &lastLine)
{
  /** when the Line overlaps with the goalline, the overlapping part is probably a free part of goal*/
  if (firstLine.x > leftOppGoalPostAbs.x - goalLineTolerance && firstLine.x < leftOppGoalPostAbs.x + goalLineTolerance &&
    lastLine.x > leftOppGoalPostAbs.x - goalLineTolerance && lastLine.x < leftOppGoalPostAbs.x + goalLineTolerance)
  {
    /** sorting the line corner points*/
    if (firstLine.y > lastLine.y)
    {
      Vector2<> buffer = firstLine;
      firstLine = lastLine;
      lastLine = buffer;
    }

    int freePartStart, freePartEnd;

    /** calculating the start and end points of the freeParts*/
    if (firstLine.y < rightOppGoalPostAbs.y)
    {
      freePartStart = static_cast<int>(rightOppGoalPostAbs.y);
    }
    else if(firstLine.y < leftOppGoalPostAbs.y)
    {
      freePartStart = static_cast<int>(firstLine.y);
    }
    else
    {
      return;
    }

    if (lastLine.y > leftOppGoalPostAbs.y)
    {
      freePartEnd = static_cast<int>(leftOppGoalPostAbs.y) - 1;
    }
    else
    {
      freePartEnd = static_cast<int>(lastLine.y);
    }

    /** normalising the start and end to the array indices */
    freePartStart += static_cast<int>(leftOppGoalPostAbs.y);
    freePartEnd += static_cast<int>(leftOppGoalPostAbs.y);

    freePartStart /= static_cast<int>(cellOnOppGoallineWidth);
    freePartEnd /= static_cast<int>(cellOnOppGoallineWidth);

    /** entering the freeParts to the array */
    for (int i = freePartStart; i <= freePartEnd; ++i)
    {
      if(freePartsOfGoal[i].second + updateRate < theFrameInfo.time)
      {
        if(freePartsOfGoal[i].first < likelinessThresholdMax)
        {
          freePartsOfGoal[i].first++;
        }

        freePartsOfGoal[i].second = theFrameInfo.time;
      }
    }

  }
}

MAKE_MODULE(LineBasedFreePartOfOpponentGoalProvider, Modeling)
