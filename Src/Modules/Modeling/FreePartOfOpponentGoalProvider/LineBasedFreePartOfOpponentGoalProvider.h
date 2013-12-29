/**
 * @author Carsten Könemann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImageCoordinateSystem.h"

MODULE(LineBasedFreePartOfOpponentGoalProvider)
  // required for calculation of free part
  REQUIRES(RobotPose)
  REQUIRES(FieldDimensions)

  // required only for debugging
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(LinePercept)
  REQUIRES(FrameInfo)
  REQUIRES(ImageCoordinateSystem)

  PROVIDES_WITH_MODIFY_AND_DRAW(FreePartOfOpponentGoalModel)

  DEFINES_PARAMETER(unsigned short, goalLineTolerance, 300) // the offset, the percepted line is accepted as a goal line
  DEFINES_PARAMETER(unsigned int, ageRate, 200) // the time a free part is buffered
  DEFINES_PARAMETER(unsigned int, updateRate, 100) // the rate a cell is updated
  DEFINES_PARAMETER(unsigned int, cellTTL, 4000) // the time to live of a cell
  DEFINES_PARAMETER(unsigned short, likelinessThresholdMin, 4) // the min value a cell must have to interpreted as a free part
  DEFINES_PARAMETER(unsigned short, likelinessThresholdMax, 10) // the max value a cell can have
  DEFINES_PARAMETER(float, agingFactor, .9f) // the factor, a cell value is decreased with
  DEFINES_PARAMETER(float, cameraShrinkFactor, .95f)
END_MODULE

/** goal line of opponent goal will be divided into this many cell */
static const int NUM_OF_CELLS = 28;

class LineBasedFreePartOfOpponentGoalProvider : public LineBasedFreePartOfOpponentGoalProviderBase
{
public:
  LineBasedFreePartOfOpponentGoalProvider();

  void update(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel);
  Vector2<> projectOnField(int x, int y);
  void drawingOnField(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel);
  void drawingOnImage(FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel);
  void drawingSeenGoalLineOnField(const Vector2<> *cameraRange);

private:
  /** this array holds values (likeliness a robot is standing there) for each cell on the opponent goal
   *  first = hitCount; second = timeStamp
   */
  std::pair<unsigned,unsigned> freePartsOfGoal[NUM_OF_CELLS];

  /** all center of a cell in field coordinates */
  Vector2<> centersOnField[NUM_OF_CELLS];

  bool initialized;

  /**
   * Enters the given line to the freeParts
   * @param firstline the first corner point of line
   * @param lastLine the last corner point of line
   */
  void addLineToFreePart(Vector2<> &firstline, Vector2<>&lastLine);

  /* setup */
  // dimensions of opponent goalline
  Vector2<> leftOppGoalPostAbs;
  Vector2<> rightOppGoalPostAbs;
  Range<> oppGoallineRange;
  float widthOfOppGoal;
  float cellOnOppGoallineWidth;
};
