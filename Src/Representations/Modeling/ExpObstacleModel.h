/**
 * @file ExpObstacleModel.h
 *
 * Declaration of class ExpObstacleModel.
 *
 * @author Florin
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Matrix4x4.h"
#include <vector>
/**
 * @class ExpObstacleModel
 *
 * A class that represents all kind of obstacles seen by vision, detected by arm contact,
 * foot bumper contact and measured by ultrasonic.
 */
STREAMABLE(ExpObstacleModel,
{
public:
  // Definition of one single Exp(erimental)Obstacle
  STREAMABLE(ExpObstacle,
  {
  public:
    ENUM(Type, US, GOALPOST, UNKNOWN, SOMEROBOT, ROBOTRED, ROBOTBLUE, SOMEFALLENROBOT, FALLENRED, FALLENBLUE); /**< The type of an expObstacle */
    ExpObstacle() = default;
    Vector2<> getRightFoot() const;
    Vector2<> getLeftFoot() const;
    Vector2<> getLeftShoulder() const;
    Vector2<> getRightShoulder() const;
    ,//end of header
    (Matrix4x4<>) covariance,                /**< Allmighty covariance of an expObstacle */
    (Vector2<>) (10000.f, 10000.f) center,   /**< Center point of an expObstacle */
    (Vector2<>) (10000.f, 10000.f) velocity, /**< Determined via UKF (mm per ms) */
    (Type) (UNKNOWN) type,                   /**< See enumeration 'Type' above */
    (unsigned) (0) lastMeasurement,          /**< Frame of last measurement */
    (unsigned) (0) seenCount,                /**< How many times an obstacle was seen (between minPercepts and maxPercepts in ExpOMProvider)*/
  });

  void draw() const;
  ,//end of header

  (std::vector<ExpObstacle>) eobs, /**< List of expObstacles (all entries are somewhat valid obstacles)*/
});
