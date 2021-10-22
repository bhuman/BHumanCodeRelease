/**
 * @file PerceptRegistration.h
 *
 * This file defines a representation that allows to assign the current perceptions
 * to actual field elements given a pose on the field.
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"


/**
 * @struct RegisteredAbsolutePoseMeasurement
 * Information about a perceived complex landmark that provides
 * information about the absolute position of the robot on the field.
 */
STREAMABLE(RegisteredAbsolutePoseMeasurement,
{,
  (Pose2f) perceivedRelativePose,  /**< The relative pose of the perceived landmark (relative to the robot) */
  (Pose2f) absolutePoseOnField,    /**< The compute absolute pose of the robot on the field (in global coordinates) */
});

/**
 * @struct RegisteredLandmark
 * Information about a perceived landmark and its associated counterpart in
 * the world model.
 */
STREAMABLE(RegisteredLandmark,
{,
  (Vector2f)(Vector2f::Zero())  percept,       /**< The position of the perceived landmark (relative to the robot) */
  (Matrix2f)(Matrix2f::Identity()) covPercept, /**< The covariance of the landmark measurement */
  (Vector2f)(Vector2f::Zero())  model,         /**< The position of original landmark (in global coordinates) */
});

/**
 * @struct RegisteredLine
 * Information about a perceived line and the associated line in
 * the world model. The struct contains some extra information to avoid
 * multiple computations of the same information.
 */
STREAMABLE(RegisteredLine,
{
  /** Constructor, copies the parameter to the member elements
   *  and computes some of the elements.
   * @param perceptStart The perceived start point of the line (relative to the robot)
   * @param perceptEnd   The perceived end point of the line (relative to the robot)
   * @param modelStart The start point of the original field line (in global coordinates)
   * @param modelEnd   The end point of the original field line (in global coordinates)
   * @param covariance The covariance of the perceived line's center point
   * @param partOfCenterCircle Flag to indicate that the original line is a segment of the center circle
   */
  RegisteredLine(const Vector2f& perceptStart, const Vector2f& perceptEnd,
                 const Vector2f& modelStart, const Vector2f& modelEnd,
                 const Matrix2f& covariance, bool partOfCenterCircle = false),

  (Vector2f)(Vector2f::Zero())  perceptStart,        /**< The position of the start of the perceived line (relative to the robot) */
  (Vector2f)(Vector2f::Zero())  perceptEnd,          /**< The position of the end of the perceived line (relative to the robot) */
  (Vector2f)(Vector2f::Zero())  perceptDirection,    /**< The direction vector of the line relative to the robot (= perceptEnd - perceptStart) */
  (Vector2f)(Vector2f::Zero())  perceptCenter,       /**< The point in the middle between perceptStart and perceptEnd */
  (Matrix2f)(Matrix2f::Identity()) covPerceptCenter, /**< The covariance of the line measurement, computed w.r.t. perceptCenter */
  (Vector2f)(Vector2f::Zero())  modelStart,          /**< The position of the start of the original line (in global coordinates) */
  (Vector2f)(Vector2f::Zero())  modelEnd,            /**< The position of the end of the original line (in global coordinates) */
  (bool)(false) parallelToWorldModelXAxis,           /**< true, if the original line is parallel to the x-axis of the robot's global coordinate system */
  (bool)(false) partOfCenterCircle,                  /**< true, if the original line is a segment of the center circle (used for drawing) */
});

/**
 * @struct PerceptRegistration
 * Representation mainly consists of a set of functions (implemented in a module) that perform the
 * assignment of perceptions to parts of the model of the field when called.
 */
STREAMABLE(PerceptRegistration,
{
  /** Function that has an implementation provided by a module.
   *  Assigns perceived absolute poses (provided by FieldFeatures) to the current position on the field.
   * @param pose The robot pose (in global coordinates) that should be used for the assignment process.
   * @param absolutePoseMeasurements A reference to a list for all registered poses. List is cleared at begin of function call.
   * @return The total number of measured poses
   */
  FUNCTION(void(const Pose2f& pose, std::vector<RegisteredAbsolutePoseMeasurement>& absolutePoseMeasurements)) registerAbsolutePoseMeasurements;

  /** Function that has an implementation provided by a module.
   *  Assigns perceived landmarks (such as penalty marks) to landmarks in the field model.
   * @param pose The robot pose (in global coordinates) that should be used for the assignment process.
   * @param landmarks A reference to a list for all registered landmarks. List is cleared at begin of function call.
   * @return The total number of perceived landmarks
   */
  FUNCTION(void(const Pose2f& pose, std::vector<RegisteredLandmark>& landmarks)) registerLandmarks;

  /** Function that has an implementation provided by a module.
   *  Assigns perceived lines to lines in the field model.
   *  Call this function after calling registerAbsolutePoseMeasurements
   * @param pose The robot pose (in global coordinates) that should be used for the assignment process.
   * @param lines A reference to a list for all registered lines. List is cleared at begin of function call.
   * @return The total number of perceived lines (is zero whenever a complex field feature [made of lines] is used in the same frame)
   */
  FUNCTION(void(const Pose2f& pose, std::vector<RegisteredLine>& lines)) registerLines,

  (int)(0) totalNumberOfAvailableAbsolutePoseMeasurements,    /**< The number of available direct measurements of the robot's pose that might be used in the current frame */
  (int)(0) totalNumberOfAvailableLandmarks,                   /**< The number of available landmark measurements (center circle, penalty mark, intersections ...) that might be used in the current frame */
  (int)(0) totalNumberOfAvailableLines,                       /**< The number of available line measurements that might be used in the current frame */
  (int)(0) totalNumberOfIgnoredLines,                         /**< The number of available line measurements that should be ignored when computing the validity, recomputed whenever registerLines is called */
});
