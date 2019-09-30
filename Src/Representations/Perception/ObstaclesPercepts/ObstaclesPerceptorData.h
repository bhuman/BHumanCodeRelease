/**
 * @file ObstaclesPerceptorData.h
 *
 * This file defines a representation that the ObstaclePerceptor sends from its
 * upper image to its lower image instance.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "ObstaclesFieldPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

STREAMABLE(ObstaclesPerceptorData,
{
  /** Scores at the lower end of a vertical scan. */
  STREAMABLE(ScanScore,
  {
    ScanScore() = default;

    /**
     * Constructor.
     * @param shortRangeScore The short range score at the lower end of the scan.
     * @param longRangeScore The long range score at lower end of the scan.
     */
    ScanScore(int shortRangeScore, int longRangeScore),

    (int) shortRangeScore, /**< The short range score at the lower end of the scan. */
    (int) longRangeScore, /**< The long range score at lower end of the scan. */
  }),

  (CameraInfo) cameraInfo,
  (CameraMatrix) cameraMatrix,
  (ImageCoordinateSystem) imageCoordinateSystem,
  (std::vector<ScanScore>) scanScores,
  (std::vector<ObstaclesFieldPercept::Obstacle>) incompleteObstacles,
});

inline ObstaclesPerceptorData::ScanScore::ScanScore(int shortRangeScore, int longRangeScore)
  : shortRangeScore(shortRangeScore), longRangeScore(longRangeScore) {}

STREAMABLE_WITH_BASE(OtherObstaclesPerceptorData, ObstaclesPerceptorData, {,});
