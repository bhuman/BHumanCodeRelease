

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Perception/RobotPercept.h"
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
#include "Representations/Perception/ImageCoordinateSystem.h"

MODULE(ObstacleWheelProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(RobotPercept),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(Image),
  REQUIRES(Odometer),
  REQUIRES(FallDownState),
  REQUIRES(MotionInfo),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleWheel),
  DEFINES_PARAMETERS(
  {,
    (int)(2700) wheelRadius, /**< Radius of the obstacle wheel in mm */
    (int)(6) coneWidth, /**< Width of one sensor code in deg */
    (float)(0.2f) stuffHoleDistThresholdAngle, /**< If the difference in distance between two obstacles is less than this value the hole inbetween is stuffed. */
    (int)(800) stuffHoleMaxRadius, /**< Holes between spots further away than this value will NOT be stuff */
    (int)(1200) decreaseSeenTickCount, /**< every x ms the seenCount of all obstacles will be decreased by 1 */
    (int)(9) maxSeenCount, /**< what the name says :P */
    (int)(3) spotIsObstacleCount, /**< If the seenCount of a spot is higher than this value it is an obstacle */
    (float)(8.0f) mergeCloserWeight, /**< When two obstacles are merged the one that is closer to the robot is weighted with this factor. */
    (float)(2.0f) mergeFurtherWeight, /**< When two obstacles are merged the one that is further away from the robot is weighted with this factor. */
  }),
});

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
