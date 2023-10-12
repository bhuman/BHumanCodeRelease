/**
 * @file BOPPerceptor.h
 *
 * This file declares a module that runs a neural network on a full image
 * to detect balls, obstacles and penalty marks.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Representations/Perception/ImagePreprocessing/SegmentedObstacleImage.h"
#include "Representations/Perception/ObstaclesPercepts/ObstacleScan.h"
#include "Math/Boundary.h"
#include "Math/Eigen.h"
#include "Framework/Module.h"
#include <CompiledNN/CompiledNN.h>
#include <memory>
#include <vector>

MODULE(BOPPerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES(BallSpots),
  PROVIDES(PenaltyMarkRegions),
  PROVIDES(ObstacleScan),
  PROVIDES_WITHOUT_MODIFY(SegmentedObstacleImage),
  DEFINES_PARAMETERS(
  {,
    (float)(0.1f) ballThreshold, /**< Threshold from which a ball spot is created. */
    (float)(0.5f) penaltyMarkThreshold, /**< Threshold from which a penalty mark region is created. */
    (float)(0.8f) obstaclesThreshold, /**< Threshold from which an obstacle is created. */
  }),
});

class BOPPerceptor : public BOPPerceptorBase
{
public:
  /** Constructor. */
  BOPPerceptor();

private:
  /**
   * Runs the neural network on the current camera image if it hasn't been done this frame.
   * @return Whether there is a valid prediction for the current image.
   */
  bool apply();

  void update(BallSpots& ballSpots) override;

  void update(PenaltyMarkRegions& penaltyMarkRegions) override;

  void update(ObstacleScan& obstacleScan) override;

  void update(SegmentedObstacleImage& segmentedObstacleImage) override;

  static constexpr std::size_t ballIndex = 0; /**< Index of the ball channel. */
  static constexpr std::size_t penaltyMarkIndex = 1; /**< Index of the penalty mark channel. */
  static constexpr std::size_t obstaclesIndex = 2; /**< Index of the obstacles channel. */
  static constexpr std::size_t numOfChannels = 4; /**< Number of channels per neural network output pixel. */

  std::unique_ptr<NeuralNetwork::Model> model; /**< The model of the neural network. */
  NeuralNetwork::CompiledNN network; /**< The compiled neural network. */
  Vector2i inputSize; /**< Input size of the neural network. */
  Vector2i outputSize; /**< Output size of the neural network. */
  Vector2i scale; /**< Scale of the neural network (input size / output size). */

  unsigned lastPrediction = 0; /**< Timestamp of the last image on which the network has been run. */
};
