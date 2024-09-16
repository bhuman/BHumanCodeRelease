/**
 * @file BallAndPenaltyMarkPerceptor.h
 *
 * This file declares a module that detects balls and penalty marks in images
 * with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 * @author Lukas Gittner
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Math/Eigen.h"
#include "Framework/Module.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(BallAndPenaltyMarkPerceptor,
{,
  REQUIRES(BallSpots),
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(MeasurementCovariance),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPose),
  PROVIDES(BallPercept),
  REQUIRES(BallPercept),
  REQUIRES(PenaltyMarkRegions),
  PROVIDES(PenaltyMarkPercept),
  LOADS_PARAMETERS(
  {
    ENUM(NormalizationMode,
    {,
      none,
      normalizeContrast,
      normalizeBrightness,
    }),

    /**< The file name (relative to "NeuralNetworks/BallAndPenaltyMarkPerceptor") from which to load the model.  */
    (std::string) multiheadName,
    (float) guessedThreshold, /**< Limit from which a ball is guessed. */
    (float) acceptThreshold, /**< Limit from which a ball is accepted. */
    (float) ensureThreshold, /**< Limit from which a ball is detected for sure. */
    (NormalizationMode) normalizationMode, /**< The kind of normalization used for patches. */
    (float) normalizationOutlierRatio, /**< The ratio of pixels ignored when determining the value range that is scaled to 0..255. */
    (float) ballAreaFactor,
    (bool) useFloat,
    (PatchUtilities::ExtractionMode) extractionMode,
    (float) penaltyThreshold, /**< Limit from which a penalty mark is accepted. */
  }),
});

class BallAndPenaltyMarkPerceptor : public BallAndPenaltyMarkPerceptorBase
{
public:
  BallAndPenaltyMarkPerceptor();

private:
  NeuralNetwork::CompiledNN multihead;

  std::unique_ptr<NeuralNetwork::Model> multiheadModel;

  std::size_t patchSize = 0;

  float bestRadius, bestProbPenalty, bestProbBall;
  Vector2f bestBallPosition;
  Vector2f bestPenaltyPosition;
  unsigned lastFrameTime = 0;

  void update(BallPercept& theBallPercept) override;
  void update(PenaltyMarkPercept& thePenaltyMarkPercept) override;
  void updateBallAndPenaltyMarkPerceptor();
  std::pair<float, float> apply(const Vector2i& ballSpot, Vector2f& ballPosition, Vector2f& penaltyPosition, float& predRadius);
  void compile();
  void savePatch(std::vector<float> data, const std::string filename);
};
