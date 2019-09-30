/**
 * @file FieldBall.h
 *
 * Declaration of a representation that contains additional information
 * about the ball that is required by the behavior.
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct FieldBall
 * A representation that contains the information about the
 * ball in field coordinates.
 */
STREAMABLE(FieldBall,
{
  /** Was the ball seen in the given time interval?
   *  @param timeInterval time since ball must have been seen (in ms)
   */
  bool ballWasSeen(int timeInterval = 500) const;

  /**
   * Returns the ball's position in field coordinates. If ball has been
   * seen by the robot within the given timeout or the global team ball is invalid, this method will
   * return the ball's position based on the estimation made by the local ball model.
   * Otherwise, the position of the global ball will be returned.
   * @param ballSeenTimeout Time in ms during which the ball must have been seen in order
   *     to rely on own ball model.
   * @param ballDisappearedTimeout Time in ms during which the ball should not have been disappeared in order
   *     to rely on own ball model.
   * @return Position of the ball in field coordinates.
   */
  Vector2f recentBallPositionOnField(const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /**
   * Returns the ball's position in relative coordinates.
   * Regarding to which ball model is used, the semantics of recentBallPositionOnField are used.
   * @param ballSeenTimeout See recentBallPositionOnField.
   * @param ballDisappearedTimeout See recentBallPositionOnField.
   * @return Position of the ball in relative coordinates.
   */
  Vector2f recentBallPositionRelative(const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /**
   * Returns the ball's end position in field coordinates, i.e. the position where it will come to a stop.
   * Regarding to which ball model is used, the semantics of recentBallPositionOnField are used.
   * @param ballSeenTimeout See recentBallPositionOnField.
   * @param ballDisappearedTimeout See recentBallPositionOnField.
   * @return Position of the ball's end position in field coordinates.
   */
  Vector2f recentBallEndPositionOnField(const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /**
   * Returns the ball's end position in relative coordinates, i.e. the position where it will come to a stop.
   * Regarding to which ball model is used, the semantics of recentBallPositionOnField are used.
   * @param ballSeenTimeout See recentBallPositionOnField.
   * @param ballDisappearedTimeout See recentBallPositionOnField.
   * @return Position of the ball's end position in relative coordinates.
   */
  Vector2f recentBallEndPositionRelative(const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /**
   * Obtains the ball's position in field coordinates as well as robot coordinates.
   * Regarding to which ball model is used, the semantics of recentBallPositionOnField are used.
   * @param ballPositionOnField Is filled with the ball position in field coordinates.
   * @param ballPositionRelative Is filled with the ball position in robot coordinates.
   * @param ballSeenTimeout See recentBallPositionOnField.
   * @param ballDisappearedTimeout See recentBallPositionOnField.
   */
  void recentBallPositions(Vector2f& ballPositionOnField, Vector2f& ballPositionRelative, const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /**
   * Obtains the ball's end position (i.e. the position where it will come to a stop) in field coordinates as well as robot coordinates.
   * Regarding to which ball model is used, the semantics of recentBallPositionOnField are used.
   * @param ballEndPositionOnField Is filled with the ball's end position in field coordinates.
   * @param ballEndPositionRelative Is filled with the ball's end position in robot coordinates.
   * @param ballSeenTimeout See recentBallPositionOnField.
   * @param ballDisappearedTimeout See recentBallPositionOnField.
   */
  void recentBallEndPositions(Vector2f& ballEndPositionOnField, Vector2f& ballEndPositionRelative, const int ballSeenTimeout = 1000, const int ballDisappearedTimeout = 100) const;

  /** Debug drawings */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) positionOnField,           /**< The ball position in global field coordinates */
  (Vector2f)(Vector2f::Zero()) positionRelative,          /**< The ball position in relative robot coordinates */
  (Vector2f)(Vector2f::Zero()) positionOnFieldClipped,    /**< The ball position in global field coordinates, clipped to the area defined by the outer lines */
  (Vector2f)(Vector2f::Zero()) positionRelativeClipped,   /**< The ball position in relative robot coordinates, clipped to the area defined by the outer lines */
  (Vector2f)(Vector2f::Zero()) endPositionOnField,        /**< The ball end position (i.e. where it comes to a stop) in global field coordinates */
  (Vector2f)(Vector2f::Zero()) endPositionRelative,       /**< The ball end position (i.e. where it comes to a stop) in relative robot coordinates */
  (Vector2f)(Vector2f::Zero()) teamPositionOnField,       /**< The ball position in global field coordinates, as estimated by the whole team */
  (Vector2f)(Vector2f::Zero()) teamPositionRelative,      /**< The ball position in relative robot coordinates, as estimated by the whole team */
  (Vector2f)(Vector2f::Zero()) teamEndPositionOnField,    /**< The ball end position (i.e. where it comes to a stop) in global field coordinates, as estimated by the whole team */
  (Vector2f)(Vector2f::Zero()) teamEndPositionRelative,   /**< The ball end position (i.e. where it comes to a stop) in relative robot coordinates, as estimated by the whole team */
  (int)(0) timeSinceBallWasSeen,                          /**< Yes, you guessed it */
  (int)(0) timeSinceBallDisappeared,                      /**< Yes, you guessed it */
  (int)(0) timeSinceTeamBallWasValid,                     /**< Yes, you guessed it */
  (bool)(false) isRollingTowardsOpponentGoal,             /**< Yes, you guessed it */
  (bool)(false) isRollingTowardsOwnGoal,                  /**< Yes, you guessed it */
  (bool)(false) isInsideOwnPenaltyArea,                   /**< Yes, you guessed it */
  (float)(0.f) distanceToOwnPenaltyArea,                  /**< Yes, you guessed it. If the ball is inside the area, member is set to 0.f. If the position is unknown, member is set to -1.f.*/
  (Vector2f)(Vector2f::Zero()) intersectionPositionWithOwnYAxis,          /**< The position (in local coordinates) at which a rolling ball will pass the robot. Vector2f::Zero(), if this will not happen. */
  (float)(std::numeric_limits<float>::max()) timeUntilIntersectsOwnYAxis, /**< The time until a rolling ball will pass the robot. float::max, if this will not happen. */
});
