/**
 * @file Modules/Infrastructure/OracledPerceptsProvider.h
 *
 * This file defines a module that provides percepts based on simulated data.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"

MODULE(OracledPerceptsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(GroundTruthWorldState),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(MeasurementCovariance),
  REQUIRES(RobotDimensions),
  PROVIDES(BallPercept),
  PROVIDES(CirclePercept),
  PROVIDES(LinesPercept),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  PROVIDES(PenaltyMarkPercept),
  PROVIDES(FieldBoundary),
  LOADS_PARAMETERS(
  {,
    (bool)  applyBallNoise,                          /**< Activate / Deactivate noise for ball percepts */
    (float) ballCenterInImageStdDev,                 /**< Standard deviation of error in pixels (x as well as y) */
    (float) ballMaxVisibleDistance,                  /**< Maximum distance until which this object can be seen */
    (float) ballRecognitionRate,                     /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (float) ballFalsePositiveRate,                   /**< Likelihood of a perceiving a false positive when the ball was not recognized */
    (bool)  applyCenterCircleNoise,                  /**< Activate / Deactivate noise for center circle percepts */
    (float) centerCircleCenterInImageStdDev,         /**< Standard deviation of error in pixels (x as well as y) */
    (float) centerCircleMaxVisibleDistance,          /**< Maximum distance until which this object can be seen */
    (float) centerCircleRecognitionRate,             /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyLineNoise,                          /**< Activate / Deactivate noise for line percepts */
    (float) linePosInImageStdDev,                    /**< Standard deviation of error in pixels (x as well as y) */
    (float) lineMaxVisibleDistance,                  /**< Maximum distance until which this object can be seen */
    (float) lineRecognitionRate,                     /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (bool)  applyPlayerNoise,                        /**< Activate / Deactivate noise for player percepts */
    (float) playerPosInImageStdDev,                  /**< Standard deviation of error in pixels (x as well as y) */
    (float) playerMaxVisibleDistance,                /**< Maximum distance until which this object can be seen */
    (float) playerRecognitionRate,                   /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (float) playerFalsePositiveRate,                 /**< Likelihood of perceiving a false positive when no player was seen */
    (bool)  applyPenaltyMarkNoise,                   /**< Activate / Deactivate noise for penalty marks */
    (float) penaltyMarkPosInImageStdDev,             /**< Standard deviation of error in pixels (x as well as y) */
    (float) penaltyMarkMaxVisibleDistance,           /**< Maximum distance until which this object can be seen */
    (float) penaltyMarkRecognitionRate,              /**< Likelihood of actually perceiving this object, when it is in the field of view */
    (float) penaltyMarkFalsePositiveRate,            /**< Likelihood of perceiving a false positive, when no penaltyMark was seen */
    (float) penaltyMarkFalseDeviationFactor,         /**< Factor of how much the false penalty mark deviates from its original position */
    (float) obstacleCoverageThickness,               /**< The obstacle radius assumed when computing whether another object is hidden behind it. */
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
  std::vector<Vector2f> goalPosts;                               /**< The positions of the four goal posts (needed for computing goal percepts)*/
  std::vector<Vector2f> penaltyMarks;                            /**< The positions of the two penalty marks (needed for computing penalty mark percepts)*/
  std::vector<Vector2f> ccPoints;                                /**< The positions of five center circle points (needed for computing center circle percept)*/
  std::vector<IntersectionsPercept::Intersection> intersections; /**< The positions of the intersections on the field (needed for computing intersection percepts) */
  std::vector<std::pair<Vector2f, Vector2f>> lines;              /**< The lines on the field */
  std::vector<std::pair<Vector2f, Vector2f>> fieldBoundaryLines; /**< The boundary of the field */
  Vector2f viewPolygon[4];                                       /**< A polygon that describes the currently visible area */

  /** One main function, might be called every cycle
   * @param ballPercept The data struct to be filled
   */
  void update(BallPercept& ballPercept) override;
  void trueBallPercept(BallPercept& ballPercept);
  void falseBallPercept(BallPercept& ballPercept);

  /** One main function, might be called every cycle
   * @param linesPercept The data struct to be filled
   */
  void update(LinesPercept& linesPercept) override;

  /** One main function, might be called every cycle
   * @param circlePercept The data struct to be filled
   */
  void update(CirclePercept& circlePercept) override;

  /** One main function, might be called every cycle
   * @param penaltyMarkPercept The data struct to be filled
   */
  void update(PenaltyMarkPercept& penaltyMarkPercept) override;
  void falsePenaltyMarkPercept(PenaltyMarkPercept& penaltyMarkPercept);

  /** One main function, might be called every cycle
   * @param obstaclesImagePercept The data struct to be filled
   */
  void update(ObstaclesImagePercept& obstaclesImagePercept) override;

  /** One main function, might be called every cycle
   * @param obstaclesFieldPercept The data struct to be filled
   */
  void update(ObstaclesFieldPercept& obstaclesFieldPercept) override;

  /** One main function, might be called every cycle
   * @param fieldBoundary The data struct to be filled
   */
  void update(FieldBoundary& fieldBoundary) override;

  /** Converts a ground truth player to a perceived player and adds it to the percept
   * @param player The ground truth player
   * @param obstaclesImagePercept The obstacles percept in the image (What else?)
   */
  void createPlayerBox(const GroundTruthWorldState::GroundTruthPlayer& player, ObstaclesImagePercept& obstaclesImagePercept);

  /** Converts a ground truth player to a perceived player and adds it to the percept
   * @param player The ground truth player
   * @param isOpponent true, if the perceived player belongs to the opponent team
   * @param obstaclesFieldPercept The obstacles percept on the field (What else?)
   */
  void createPlayerOnField(const GroundTruthWorldState::GroundTruthPlayer& player, bool isOpponent, ObstaclesFieldPercept& obstaclesFieldPercept);

  /** Computes some noise and adds it to the given position
   * @param standardDeviation The standard deviation of the pixel error
   * @param p The point in an image that is subject to noise
   */
  void applyNoise(float standardDeviation, Vector2f& p) const;

  /** Computes some noise and adds it to the given position (integer version)
   * @param standardDeviation The standard deviation of the pixel error
   * @param p The point in an image that is subject to noise
   */
  void applyNoise(float standardDeviation, Vector2i& p) const;

  /** Computes some noise and adds it to the given angle
   * @param standardDeviation The standard deviation of the angle error (in degrees)
   * @param angle The angle that is subject to noise
   */
  void applyNoise(float standardDeviation, float& angle) const;

  /** Updates viewPolygon member */
  void updateViewPolygon();

  /** Checks if a line (or parts of it) is inside the view polygon
   * @param line The line to be checked
   * @param start The start of the part that is inside the view polygon (set by this method)
   * @param end The end of the part that is inside the view polygon (set by this method)
   * @return true, if at aleast a part of the line is visible
   */
  bool partOfLineIsVisible(const std::pair<Vector2f, Vector2f>& line, Vector2f& start, Vector2f& end) const;

  bool isPointBehindObstacle(const Vector2f& point) const;
};
