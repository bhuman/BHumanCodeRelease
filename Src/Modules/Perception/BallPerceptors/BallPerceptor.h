/**
 * @file BallPerceptor.h
 *
 * This file declares a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 */

#pragma once

#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(BallPerceptor,
{,
  REQUIRES(BallSpots),
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  PROVIDES(BallPercept),
  LOADS_PARAMETERS(
  {,
    (std::string) encoderName, /**< The file name (relative to "NeuralNetworks/BallPerceptor") from which to load the model.  */
    (std::string) classifierName,
    (std::string) correctorName,
    (float) guessedThreshold, /**< Limit from which a ball is guessed. */
    (float) acceptThreshold, /**< Limit from which a ball is accepted. */
    (float) ensureThreshold, /**< Limit from which a ball is detected for sure. */
    (bool) useContrastNormalization,
    (float) ballAreaFactor,
    (float) contrastNormalizationPercent,
    (bool) useFloat,
    (PatchUtilities::ExtractionMode) extractionMode,
  }),
});

class BallPerceptor : public BallPerceptorBase
{
public:
  BallPerceptor();

private:
  NeuralNetwork::CompiledNN encoder;
  NeuralNetwork::CompiledNN classifier;
  NeuralNetwork::CompiledNN corrector;

  std::unique_ptr<NeuralNetwork::Model> encModel;
  std::unique_ptr<NeuralNetwork::Model> clModel;
  std::unique_ptr<NeuralNetwork::Model> corModel;

  std::size_t patchSize = 0;

  void update(BallPercept& theBallPercept) override;
  float apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius);
  void compile();
};