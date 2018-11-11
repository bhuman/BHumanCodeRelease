/**
 * @file NNBallPerceptor.h
 *
 * This file declares a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BallPercepts/ConfirmedBallSpot.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include "Tools/NeuralNetwork/NNUtilities.h"
#include "Representations/Modeling/LabelImage.h"
#include "Tools/NeuralNetwork/Model.h"

MODULE(NNBallPerceptor,
{,
  REQUIRES(BallSpots),
  REQUIRES(ECImage),
  REQUIRES(FrameInfo),
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(LabelImage),
  REQUIRES(CameraMatrix),
  PROVIDES(ConfirmedBallSpot),
  LOADS_PARAMETERS(
  {
    ENUM(NNType,
    {,
      conv_only,
      meta,
    }
    ),
    (NNType) path,
  }),
});

class NNBallPerceptor : public NNBallPerceptorBase
{
public:
  NNBallPerceptor();

private:
  STREAMABLE(NNConfig,
  {,
    (float) guessedThreshold, /**< Limit from which a ball is guessed. */
    (float) acceptThreshold, /**< Limit from which a ball is accepted. */
    (float) ensureThreshold, /**< Limit from which a ball is detected for sure. */
    (bool) prioritizeBallPrediction,
    (bool) useContrastNormalization,
    (float) ballAreaFactor,
    (float) contrastNormalizationPercent,
    (bool) useMeta,
    (bool) useFloat,
    (NNUtilities::ExtractionMode) extractionMode,
    (bool) logImages,
    (bool) logOnlyFalse,
    (bool) useVerification,
    (float) resampleThreshold,
    (bool) useResampling,
  });

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

  std::vector<NeuralNetwork::CompiledNN*> models;
  NeuralNetwork::CompiledNN convModel, metaModel;
  std::unique_ptr<NeuralNetwork::Model> model;
  VectorXf probs;
  size_t patchSize;
  void update(ConfirmedBallSpot& ballSpot) override;
  float apply(const Vector2i& ballSpot, int i, const Vector2f& offset);
  void compile();
  float handleReturn(const NeuralNetwork::Tensor3& output);
  NNType lastType;
  NNConfig config;
  NNStats stats;
};
