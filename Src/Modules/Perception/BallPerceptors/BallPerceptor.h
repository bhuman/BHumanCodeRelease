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

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Perception/BallPercepts/ConfirmedBallSpot.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include "Tools/NeuralNetwork/Model.h"

#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

MODULE(BallPerceptor,
{,
  REQUIRES(BallSpots),
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(DemoConfirmedBallSpots),
  REQUIRES(ECImage),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(LabelImage),
  PROVIDES(BallPercept),
  LOADS_PARAMETERS(
  {,
    (std::string) encoderName, /**< The file name (relative to "NeuralNetworks/BallPerceptor") from which to load the model.  */
    (std::string) classifierName,
    (std::string) correctorName,
    (float) guessedThreshold, /**< Limit from which a ball is guessed. */
    (float) acceptThreshold, /**< Limit from which a ball is accepted. */
    (float) ensureThreshold, /**< Limit from which a ball is detected for sure. */
    (bool) prioritizeBallPrediction,
    (bool) useContrastNormalization,
    (float) ballAreaFactor,
    (float) contrastNormalizationPercent,
    (bool) useFloat,
    (PatchUtilities::ExtractionMode) extractionMode,
    (bool) logImages,
    (bool) logOnlyFalse,
    (bool) useVerification,
    (float) resampleThreshold,
    (bool) useResampling,
  }),
});

class BallPerceptor : public BallPerceptorBase
{
public:
  BallPerceptor();

private:
  STREAMABLE(NNStats,
  {
    void add(const float y_true, const float y_pred)
    {
      if(y_true > 0.5f)
        if(y_pred > 0.5f)
          truePositive++;
        else
          falseNegative++;
      else if(y_pred > 0.5f)
        falsePositive++;
      else
        trueNegative++;
      recall = (truePositive + falseNegative) == 0 ? 0.f : 1.f * truePositive / (truePositive + falseNegative);
      precision = (truePositive + falsePositive) == 0 ? 0.f : 1.f * truePositive / (truePositive + falsePositive);
    },

    (float)(0.f) recall, /**< Limit from which a ball is guessed. */
    (float)(0.f) precision, /**< Limit from which a ball is accepted. */
    (int)(0) truePositive,
    (int)(0) trueNegative,
    (int)(0) falsePositive,
    (int)(0) falseNegative,
    (int)(0) total,
  });

  NeuralNetwork::CompiledNN encoder;
  NeuralNetwork::CompiledNN classifier;
  NeuralNetwork::CompiledNN corrector;

  std::unique_ptr<NeuralNetwork::Model> encModel;
  std::unique_ptr<NeuralNetwork::Model> clModel;
  std::unique_ptr<NeuralNetwork::Model> corModel;

  VectorXf probs;
  size_t patchSize;
  void update(BallPercept& theBallPercept) override;
  float apply(const Vector2i& ballSpot, int i, const Vector2f& offset, std::vector<Vector2f>& ballPosition, std::vector<float>& predRadius);
  void compile();
  NNStats stats;
};
