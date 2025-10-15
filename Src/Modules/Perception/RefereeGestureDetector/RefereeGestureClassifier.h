/**
 * @file RefereeGestureClassifier.h
 *
 * This file declares a module that classifies referee gestures and provides the result to the RefereeGesture representation.
 *
 * @author Roman Sablotny, Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "ImageProcessing/Image.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Platform/File.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"
#include "Representations/Perception/RefereeGestures/RefereeGesture.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

STREAMABLE_WITH_BASE(UpperCameraInfo, CameraInfo, {,});

MODULE(RefereeGestureClassifier,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(UpperCameraInfo),
  REQUIRES(RobotPose),
  REQUIRES(OptionalCameraImage),
  PROVIDES(RefereeGesture),
  LOADS_PARAMETERS(
  {,
    (unsigned) patchSize,  /**< size of the patch */
    (unsigned) patchWidthStandby, /**< width of the patch when in standby */
    (float) refereeHeight, /**< The expected height of the referee in mm to determine the dynamic patch size. */
    (float) normMinDist,  /**< lowest distance for normalization */
    (float) normMaxDist,  /**< highest distance for normalization */
    (float) netThreshold, /**< threshold value for the confidence value of the neural net. */
    (bool) useDynamicScaling, /**< Use dynamic patch scaling. */
  }),
});

class RefereeGestureClassifier : public RefereeGestureClassifierBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRefereeGesture The representation updated.
   */
  void update(RefereeGesture& theRefereeGesture) override;

  void copyPatchToInput(YUVImage& img, float* input) const;

  float normalizeDistance(const float distance, const float minVal, const float maxVal) const;

  void softmaxNetworkOutput(const Vector3f& networkOutput, const bool useThirdOutput, RefereeGesture& theRefereeGesture);

  NeuralNetwork::CompiledNN networkKickIn;
  NeuralNetwork::CompiledNN networkStandby;

public:
  /** Constructor. */
  RefereeGestureClassifier();
};
