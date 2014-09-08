/**
* @file Modules/Infrastructure/OracledPerceptsProvider.h
*
* This file implements a module that provides percepts based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/RobotPercept.h"

MODULE(OracledPerceptsProvider,
{,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(FrameInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES_WITH_MODIFY_AND_DRAW(BallPercept),
  PROVIDES_WITH_MODIFY_AND_DRAW(GoalPercept),
  PROVIDES_WITH_MODIFY_AND_DRAW(LinePercept),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPercept),
  PROVIDES_WITH_MODIFY_AND_DRAW(FieldBoundary),
  LOADS_PARAMETERS(
  {,
    (bool)  applyBallNoise,                    /**< Activate / Deactivate noise for ball percepts */
    (float) ballCenterInImageStdDev,           /**< Standard deviation of error in pixels (x as well as y) */
    (float) ballMaxVisibleDistance,            /**< Maximum distance until which this object can be seen */
    (float) ballRecognitionRate,               /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyCenterCircleNoise,            /**< Activate / Deactivate noise for center circle percepts */
    (float) centerCircleCenterInImageStdDev,   /**< Standard deviation of error in pixels (x as well as y) */
    (float) centerCircleMaxVisibleDistance,    /**< Maximum distance until which this object can be seen */
    (float) centerCircleRecognitionRate,       /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyIntersectionNoise,            /**< Activate / Deactivate noise for intersection percepts */
    (float) intersectionPosInImageStdDev,      /**< Standard deviation of error in pixels (x as well as y) */
    (float) intersectionMaxVisibleDistance,    /**< Maximum distance until which this object can be seen */
    (float) intersectionRecognitionRate,       /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyLineNoise,                    /**< Activate / Deactivate noise for line percepts */
    (float) linePosInImageStdDev,              /**< Standard deviation of error in pixels (x as well as y) */
    (float) lineMaxVisibleDistance,            /**< Maximum distance until which this object can be seen */
    (float) lineRecognitionRate,               /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyRobotNoise,                   /**< Activate / Deactivate noise for robot percepts */
    (float) robotPosInImageStdDev,             /**< Standard deviation of error in pixels (x as well as y) */
    (float) robotMaxVisibleDistance,           /**< Maximum distance until which this object can be seen */
    (float) robotRecognitionRate,              /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyGoalPostNoise,                /**< Activate / Deactivate noise for robot percepts */
    (float) goalPostPosInImageStdDev,          /**< Standard deviation of error in pixels (x as well as y) */
    (float) goalPostMaxVisibleDistance,        /**< Maximum distance until which this object can be seen */
    (float) goalPostRecognitionRate,           /**< Likelihood of actually perceiving this object, when it is in the field of view */
  }),
});

/**
* @class OracledPerceptsProvider
* A module that provides several percepts
*/
class OracledPerceptsProvider: public OracledPerceptsProviderBase
{
public:
  /** Constructor*/
  OracledPerceptsProvider();

private:
  std::vector<Vector2<>> goalPosts;                                /*< The positions of the four goal posts (needed for computing goal percepts)*/
  std::vector<Vector2<>> ccPoints;                                 /*< The positions of five center circle points (needed for computing center circle percept)*/
  std::vector<LinePercept::Intersection> intersections;            /*< The positions of the intersections on the field (needed for computing intersection percepts) */
  std::vector<std::pair<Vector2<>, Vector2<>>> lines;              /*< The lines on the field */
  std::vector<std::pair<Vector2<>, Vector2<>>> fieldBoundaryLines; /*< The boundary of the field */
  Vector2<> viewPolygon[4];                                        /*< A polygon that describes the currently visible area */

  /** One main function, might be called every cycle
  * @param ballPercept The data struct to be filled
  */
  void update(BallPercept& ballPercept);

  /** One main function, might be called every cycle
  * @param goalPercept The data struct to be filled
  */
  void update(GoalPercept& goalPercept);

  /** One main function, might be called every cycle
  * @param linePercept The data struct to be filled
  */
  void update(LinePercept& linePercept);

  /** One main function, might be called every cycle
  * @param robotPercept The data struct to be filled
  */
  void update(RobotPercept& robotPercept);
  
  /** One main function, might be called every cycle
   * @param fieldBoundary The data struct to be filled
   */
  void update(FieldBoundary& fieldBoundary);

  /** Converts a ground truth robot to a perceived robot and adds it to the percept
  * @param robot The ground truth robot
  * @param isBlue true, if the perceived robot belongs to the blue team
  * @param robotPercept The robot percept (What else?)
  */
  void createRobotBox(const GroundTruthWorldState::GroundTruthRobot& robot, bool isBlue, RobotPercept& robotPercept);

  /** Checks, if a point on the field (relative to the robot) is inside the current image
  * @param  p    The point
  * @param  pImg The point projected to the current image
  * @return      true, if the point can be seen by the robot
  */
  bool pointIsInImage(const Vector2<>& p, Vector2<>& pImg) const;

  /** Computes some noise and adds it to the given position
  * @param standardDeviation The standard deviation of the pixel error
  * @param p The point in an image that is subject to noise
  */
  void applyNoise(float standardDeviation, Vector2<>& p) const;

  /** Computes some noise and adds it to the given position (integer version)
  * @param standardDeviation The standard deviation of the pixel error
  * @param p The point in an image that is subject to noise
  */
  void applyNoise(float standardDeviation, Vector2<int>& p) const;

  /** Updates viewPolygon member */
  void updateViewPolygon();

  /** Checks if a line (or parts of it) is inside the view polygon
  * @param line The line to be checked
  * @param start The start of the part that is inside the view polygon (set by this method)
  * @param end The end of the part that is inside the view polygon (set by this method)
  * @return true, if at aleast a part of the line is visible
  */
  bool partOfLineIsVisible(const std::pair<Vector2<>,Vector2<>>& line, Vector2<>& start, Vector2<>& end) const;
};
