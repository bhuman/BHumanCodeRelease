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
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include <CompiledNN2ONNX/CompiledNN.h>

MODULE(KeypointsProvider,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  PROVIDES(Keypoints),
  LOADS_PARAMETERS(
  {,
    (std::string) filename, /**< The network to to load. */
    (float) maskCenterY, /**< y coordinate of a disc-shaped image area not to mask out. */
    (float) maskRadius, /**< radius of a disc-shaped image area not to mask out. */
    (int) maskWidth, /**< Width of a centered column in the image not to mask out. */
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

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeypoints The representation updated.
   */
  void update(Keypoints& theKeypoints) override;

  /**
   * Create a mask which parts of the network input should be kept and which
   * should be reset.
   * @param height The height of the patch in the image in pixels.
   * @param width The width of the patch in the image in pixels.
   * @param patchHeight The height of the patch i.e. network input.
   * @param patchWidth The width of the patch i.e. network input.
   */
  void createMask(const int height, const int width, const int patchHeight, const int patchWidth);

  /**
   * Apply the mask by setting selected values of the network input to 0.
   * @param patchHeight The height of the patch i.e. network input.
   * @param patchWidth The width of the patch i.e. network input.
   * @param data The network input that is changed.
   */
  void applyMask(const int patchHeight, const int patchWidth, float* data) const;

  /**
   * Extract image data from a centered patch and copy it to float array.
   * In both dimensions, 2 YUV pixels are scaled down to a single RGB network input.
   * @param height The height of the centered patch in pixels.
   * @param width The width of the centered patch in pixels.
   * @param channel The data is copied to this array.
   */
  void extractPatch2to1(const int height, const int width, float* channel) const;

  /**
   * Extract image data from a centered patch and copy it to float array.
   * In both dimensions, 3 YUV pixels are scaled down to 2 RGB network inputs.
   * @param height The height of the centered patch in pixels.
   * @param width The width of the centered patch in pixels.
   * @param channel The data is copied to this array.
   */
  void extractPatch3to2(const int height, const int width, float* channel) const;

public:
  /** The constructor loads the network. */
  KeypointsProvider();
};
