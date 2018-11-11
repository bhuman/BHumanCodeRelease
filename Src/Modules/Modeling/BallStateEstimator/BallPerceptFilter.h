/**
 * @file BallPerceptFilter.h
 *
 * For ball tracking in the RoboCup context, two tasks need to be solved:
 *   1. Filtering and clustering the detected balls (i.e. the BallPercepts) to avoid
 *      the use of any false positive, which might lead to severe problems.
 *   2. Estimating a precise position as well as a velocity based on a set of recent
 *      ball observations.
 * In previous implementations, these two tasks have been combined within one module. For more
 * clarity and more flexibility, both tasks are split into two modules now.
 * In addition, collision detection between feet and ball has been moved to a third model.
 *
 * This module provides a solution for task 1: Filtering and clustering BallPercepts.
 *
 * The module declared in this file is based on the basic filtering implementation that has been used by
 * B-Human inside the BallLocator module in recent years.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/BallContactWithRobot.h"
#include "Representations/Modeling/FilteredBallPercepts.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"

MODULE(BallPerceptFilter,
{,
  REQUIRES(BallPercept),
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(Odometer),
  REQUIRES(OdometryData),
  REQUIRES(TeamData),
  REQUIRES(WorldModelPrediction),
  USES(BallContactWithRobot),
  USES(RobotPose),
  PROVIDES(FilteredBallPercepts),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) fieldBorderExclusionDistance,          /**< Stuff that is more far away from the field (not the carpet!) is excluded. */
    (int)(3) ballBorderAdditionalPixelThreshold,          /**< When excluding certain balls at the image's border, this is the required distance of the ball center to the border. */
    (float)(400.f) robotBanRadius,                        /**< If a ball is at the image border and this close to a teammate, it becomes excluded. */
    (bool)(false) banBallsInRobotsAtImageBorder,          /**< If true, some balls at/in robots close to the image border might become excluded. */
    (int)(6000) fieldFeatureTimeout,                      /**< When using self-localization information, there should have been a field feature recently. */
    (Vector2f)(0.04f, 0.06f) robotRotationDeviation,      /**< Deviation of the rotation of the robot's torso */
    (int)(500) farBallIgnoreTimeout,                      /**< If we have seen a ball in the lower image recently, ignore balls in upper image for this time, if they are far away. */
    (float)(2500.f) farBallIgnoreDistance,                /**< If we have seen a ball in the lower image recently, ignore balls in upper image (for some time) that are farther away than this parameter specifies. */
    (int)(600) bufferedSeenBallTimeout,                   /**< The seen ball percepts for verification must have been seen within this amount of time. */
    (int)(2000) bufferedBallTimeout,                      /**< The other perceptions for verification must have been seen within this amount of time. */
    (unsigned)(2) requiredPerceptionCountNear,            /**< For near balls, this many perceptions are required for verification. */
    (unsigned)(1) requiredPerceptionCountFar,             /**< For far balls, this many perceptions are required for verification. */
    (float)(400.f) requiredPerceptionDistanceNear,        /**< Maximum distance to a verification perception, if the ball is near */
    (float)(800.f) requiredPerceptionDistanceFar,         /**< Maximum distance to a verification perception, if the ball is far */
    (float)(1337.f) beginningOfFar,                       /**< Everything farther than this is consideres as "far" */
    (unsigned)(2) neededSeenBallsForAcceptingGuessedOne,  /**< This number of seen balls must be quite close to a guessed ball to accept it. */
    (Angle)(15_deg) maximumAngleBetweenSeenAndGuessed,    /**< When comparing seen and guessed balls, the angles to them should not be larger than this. */
    (float)(200.f) maximumDistanceBetweenSeenAndGuessed,  /**< When comparing seen and guessed balls, the distance between them should not be larger than this. */
    (int)(2000) timeSpanForAcceptingKickedGuessedBalls,   /**< During this time (in ms) after a kick, a sequence of guessed (and / or seen) balls might be accepted. */
    (int)(5) requiredNumberOfGuessBallsForKickDetection,  /**< To accept a guessed ball by assuming it has been kicked, at least this number of percepts is required for checking the hypothesis */
    (int)(1500) timeSpanForGuessedBallsMotionDetection,   /**< For this time span(in ms), a sequence of guessed (and / or seen) balls can be perceived as moving and thereby become accepted. */
    (int)(5) requiredNumberOfGuessBallsForMotionDetection,/**< To accept a guessed ball by assuming it is rolling, at least this number of percepts is required for checking the hypothesis */
    (float)(900) minimumVelocityForMotionDetection,       /**< Guessed percepts are only considered as moving, if the ball seems to have at least this velocity (in mm/s) */
    (float)(100) maxStandardDeviationMotionDetection,     /**< Mean error (in mm) of percepts considered as a rolling ball is not allowed to be larger than this. */
  }),
});

/**
 * @class BallPerceptFilter
 *
 * Some basic techniques to get rid off false positive BallPercepts.
 */
class BallPerceptFilter : public BallPerceptFilterBase
{
public:
  /** Constructor */
  BallPerceptFilter();

private:
  unsigned int timeBallWasBeenSeenInLowerCameraImage;    /**< Yip, exactly the point of time when the ball was seen in the lower camera for the last time */
  unsigned int timeWhenLastKickWasExecuted;              /**< Yip. Your guess about this variable is absolutely right! */
  RingBuffer<FilteredBallPercept, 5> bufferedSeenBalls;  /**< Each percept needs to be verified. This is the verification buffer. However, it contains only perceptions that had the status "seen". */
  RingBuffer<FilteredBallPercept, 10> bufferedBalls;     /**< A buffer for all guessed and seen balls, used for accepting moving guessed balls. */
  unsigned int timeOfLastFilteredPercept;                /**< Point of time when the last percept has been added to the module's output representation */

  void doSomeTestStuff();
  
  /** Check, if the percept is outside the field
   * @return true, if it is outside the field
   */
  bool perceptCanBeExcludedByLocalization();
  
  /** Check, if the percept might be a teammate at the image border
   * @return true, if this is the case
   */
  bool perceptIsAtImageBorderAndCloseToTeammate();

  /**
   * Determines a set of valid BallPercepts
   * @param filteredBallPercepts The data structure that is filled
   */
  void update(FilteredBallPercepts& filteredBallPercepts) override;
  
  /** Check, if the ball has been seen close to a previously detected ball
   * @return true, if this is the case
   */
  bool verifySeenBall();
  
  /** Sometimes, we get ball percepts and the Perceptor is not sure, whether or not they are correct.
   *  If these "guessed" ball percepts are close to recently seen better ones, they should be used, too.
   *  @return true, if the described situation is the case
   */
  bool currentlyPerceivedGuessedBallIsCloseToPreviouslyPerceivedSeenBalls();
  
  /** After kicking a ball, most percepts are only guessed. This function checks, if there is a
   *  sequence of guessed and/or seen balls that moves away from the robot shortly after a kick.
   *  @return true, if such a sequence has been found.
   */
  bool robotRecentlyKickedAndThereAreGuessedBallsRollingAway();
  
  /** If a ball rolls across the field, most percepts are only guessed. This function checks, if there is a
   *  sequence of guessed and/or seen balls that is on a plausible line, like a rolling ball.
   *  @return true, if such a sequence has been found.
   */
  bool perceptsAreOnALineThatIsCompatibleToARollingBall();
  
  /** Computes the mean of an assumed static ball based on a list of observations and returns
   *  the standard deviation of the observations to the mean.
   *  @param indexOfOldestObservation An index into the bufferedBalls buffer, pointing to the oldest observation to be considered
   *  @return The aforementioned standard deviation
   */
  float computeStdDevOfStaticBallHypothesis(unsigned indexOfOldestObservation);
  
  /** Computes the mean of an assumed rolling ball based on a list of observations and returns
   *  the standard deviation of the observations to estimated trajectory.
   *  @param indexOfOldestObservation An index into the bufferedBalls buffer, pointing to the oldest observation to be considered
   *  @param endVelocity Reference to a vector which is filled by this method with the estimated velocity
   *  @return The aforementioned standard deviation
   */
  float computeStdDevOfMovingBallHypothesis(unsigned indexOfOldestObservation, Vector2f& endVelocity);
};
