/**
 * @file PenaltyMarkClassifier.h
 *
 * This file declares a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 */

#pragma once

#include "ImageProcessing/PatchUtilities.h"
#include "Modules/Perception/FieldPerceptors/PenaltyMarkRegionsProvider.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(PenaltyMarkClassifier,
  { ,
    REQUIRES(BallModel),
    REQUIRES(CameraMatrix),
    REQUIRES(CameraInfo),
    REQUIRES(ECImage),
    REQUIRES(FieldDimensions),
    REQUIRES(FrameInfo),
    REQUIRES(GameState),
    REQUIRES(ImageCoordinateSystem),
    REQUIRES(PenaltyMarkRegions),
    PROVIDES(PenaltyMarkPercept),
    DEFINES_PARAMETERS(
    {,
      (float)(8.f) threshold,
    }),
  });

class PenaltyMarkClassifier : public PenaltyMarkClassifierBase
{
public:
  PenaltyMarkClassifier();
private:
  NeuralNetwork::CompiledNN network;
  std::unique_ptr<NeuralNetwork::Model> model;

  void update(PenaltyMarkPercept& thePenaltyMarkPercept) override;
};
