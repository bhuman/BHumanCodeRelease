

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Perception/ObstacleSpots.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(ObstacleWheelProvider)
REQUIRES(FrameInfo)
REQUIRES(ObstacleSpots)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(Image)
REQUIRES(Odometer)
REQUIRES(FallDownState)
REQUIRES(MotionInfo)
REQUIRES(RobotInfo)
REQUIRES(GameInfo)
REQUIRES(GroundContactState)
PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleWheel)

DEFINES_PARAMETER(int, wheelRadius, 2700) /**< Radius of the obstacle wheel in mm */
DEFINES_PARAMETER(int, coneWidth, 6) /**< Width of one sensor code in deg */
DEFINES_PARAMETER(float, stuffHoleDistThresholdAngle, 0.2f) /**< If the difference in distance between two obstacles is less than this value the hole inbetween is stuffed. */
DEFINES_PARAMETER(int, stuffHoleMaxRadius, 800) /**< Holes between spots further away than this value will NOT be stuff */
DEFINES_PARAMETER(int, decreaseSeenTickCount, 1200); /**< every x ms the seenCount of all obstacles will be decreased by 1 */
DEFINES_PARAMETER(int, maxSeenCount, 9); /**< what the name says :P */
DEFINES_PARAMETER(int, spotIsObstacleCount, 3); /**< If the seenCount of a spot is higher than this value it is an obstacle */
DEFINES_PARAMETER(float, mergeCloserWeight, 8.0f); /**< When two obstacles are merged the one that is closer to the robot is weighted with this factor. */
DEFINES_PARAMETER(float, mergeFurtherWeight,2.0f); /**< When two obstacles are merged the one that is further away from the robot is weighted with this factor. */
END_MODULE

class ObstacleWheelProvider : public ObstacleWheelProviderBase
{
public:
  ObstacleWheelProvider();

private:
  int oldConeWidth;
  int oldWheelRadius;
  int lastDecreaseTimestamp;
  int coneCount;


  void update(ObstacleWheel& wheel);

  /**
   * @param pInField point in field coordinates
   * @param coneWidth width of one sensor cone in rad
   * @param coneCount total number of cones
   * @return
   */
  int calcConeIndex(const Vector2<float>& pInField, ObstacleWheel& wheel) const;

  void updateOdometry(ObstacleWheel& wheel) const;

  void initialize(ObstacleWheel& wheel);

  void enterObstacles(ObstacleWheel& wheel) const;

  /**Enters a single spot into the wheel, merges if needed*/
  void enterSpot(ObstacleWheel& wheel, const Vector2<>& pInField,
                 const int initialSeenCount, const CameraInfo::Camera seenBy,
                 const bool seenThisFrame, const int seenCountIncrement) const;


  /**Calculate angle distance from normal distance*/
  float calcAngleDist(const float dist) const;

  void decreaseSeenCount(ObstacleWheel& wheel);

  /** Check if both neighbors of a cone contain obstacles at roughly the same distance.
   *  If so the cone will contain an obstacle as well*/
  void stuffHoles(ObstacleWheel& wheel) const;

  /**If left and right are obstacles and middle is not, middle will become an obstacle
   * as well */
  void stuffHole(std::vector<ObstacleWheel::Cone>::iterator left,
                 std::vector<ObstacleWheel::Cone>::iterator middle,
                 std::vector<ObstacleWheel::Cone>::iterator right) const;

  void clearWheel(ObstacleWheel& wheel) const;

};
