/**
 * @file KeypointsProvider.h
 *
 * This file declares a module that subsamples a centered patch from the
 * camera image and uses MoveNet to detect keypoints in it. The keypoints
 * correspond to different body parts of a single person.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Math/Range.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include <CompiledNN2ONNX/CompiledNN.h>

MODULE(KeypointsProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(OptionalCameraImage),
  REQUIRES(RobotPose),
  PROVIDES(Keypoints),
  LOADS_PARAMETERS(
  {,
    (std::string) filename, /**< The network to to load. */
    (int) patchSize, /**< The size of the patch (192, 256, 384). */
    (bool) patchAtTop, /**< Is the patch aligned with the top rather than centered? */
    (float) maskCenterY, /**< y coordinate of a disc-shaped image area not to mask out (for observer distance of 3 m ). */
    (float) maskRadius, /**< radius of a disc-shaped image area not to mask out (for observer distance of 3 m ). */
    (int) maskWidth, /**< Width of a centered column in the image not to mask out (for observer distance of 3 m ). */
    (float) maskRecomputeThreshold, /**< If the robot's position changed more than this threshold, the masks is recomputed (in mm). */
    (float) minConfidence, /**< Minimum confidence required to accept a keypoint as valid. */
  }),
});

class KeypointsProvider : public KeypointsProviderBase
{
  /** The format of each keypoint in the network output. */
  struct Output
  {
    float y;
    float x;
    float confidence;
  };

  NeuralNetworkONNX::CompiledNN detector; /**< The network that detects keypoints. */
  std::vector<Rangei> mask; /**< The x ranges to keep (max exclusive) per network input row. */
  Vector2f lastRobotPosition = Vector2f::Zero(); /**< The last position of the robot when this module was used. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeypoints The representation updated.
   */
  void update(Keypoints& theKeypoints) override;

  /** Compiles the neural network. */
  void compileNetwork();

  /**
   * Create a mask which parts of the network input should be kept and which
   * should be reset.
   * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
   * @param centerY The vertical center of the patch in the image in pixels.
   * @param height The height of the patch in the image in pixels.
   * @param width The width of the patch in the image in pixels.
   * @param patchHeight The height of the patch i.e. network input.
   * @param patchWidth The width of the patch i.e. network input.
   */
  void createMask(const unsigned centerX, const unsigned centerY,
                  const int height, const int width, const int patchHeight, const int patchWidth);

  /**
   * Apply the mask by setting selected values of the network input to 0.
   * @param patchHeight The height of the patch i.e. network input.
   * @param patchWidth The width of the patch i.e. network input.
   * @param data The network input that is changed.
   */
  void applyMask(const int patchHeight, const int patchWidth, float* data) const;

  /**
   * Extract image data from a centered patch and copy it to a float array.
   * YUV pixels are converted to RGB network input.
   * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
   * @param centerY The vertical center of the patch in the image in pixels.
   * @param height The height of the patch in pixels.
   * @param width The width of the patch in pixels.
   * @param channel The data is copied to this array.
   */
  void extractPatch1to1(const unsigned centerX, const unsigned centerY,
                        const int height, const int width, float* channel) const;

  /**
   * Extract image data from a centered patch and copy it to a float array.
   * In both dimensions, 2 YUV pixels are scaled down to a single RGB network input.
   * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
   * @param centerY The vertical center of the patch in the image in pixels.
   * @param height The height of the centered patch in pixels.
   * @param width The width of the centered patch in pixels.
   * @param channel The data is copied to this array.
   */
  void extractPatch2to1(const unsigned centerX, const unsigned centerY,
                        const int height, const int width, float* channel) const;

  /**
   * Extract image data from a centered patch and copy it to a float array.
   * In both dimensions, 3 YUV pixels are scaled down to 2 RGB network inputs.
   * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
   * @param centerY The vertical center of the patch in the image in pixels.
   * @param height The height of the centered patch in pixels.
   * @param width The width of the centered patch in pixels.
   * @param channel The data is copied to this array.
   */
  void extractPatch3to2(const unsigned centerX, const unsigned centerY,
                        const int height, const int width, float* channel) const;

public:
  /** The constructor loads the network. */
  KeypointsProvider();
};
