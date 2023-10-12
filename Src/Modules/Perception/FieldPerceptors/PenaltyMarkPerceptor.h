/**
 * @file PenaltyMarkPerceptor.h
 *
 * This file declares a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 */

#pragma once

#include "ImageProcessing/PatchUtilities.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(PenaltyMarkPerceptor,
{,
  REQUIRES(BallPercept),
  REQUIRES(BallSpecification),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(MeasurementCovariance),
  REQUIRES(PenaltyMarkRegions),
  PROVIDES(PenaltyMarkPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(8.f) threshold,
  }),
});

class PenaltyMarkPerceptor : public PenaltyMarkPerceptorBase
{
public:
  PenaltyMarkPerceptor();

private:
  void update(PenaltyMarkPercept& thePenaltyMarkPercept) override;

  NeuralNetwork::CompiledNN network;
  std::unique_ptr<NeuralNetwork::Model> model;
};
