/**
 * @file ECImageProvider.h
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Framework/Module.h"

MODULE(ECImageProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  REQUIRES(ECImage),
  PROVIDES(ECImage),
  PROVIDES(OptionalECImage),
  LOADS_PARAMETERS(
  {,
    (bool) disableColor,
    (bool) extractChroma,
  }),
});

/**
 * @class ECImageProvider
 */
class ECImageProvider : public ECImageProviderBase
{
private:
  using EcFunc = void (*)(unsigned int, const void*, void*, void*, void*);
  using EFunc = void (*)(unsigned int, const void*, void*);

  EcFunc ecFunc = nullptr;
  EFunc eFunc = nullptr;

  void update(ECImage& ecImage) override;
  void update(OptionalECImage& theOptionalECImage) override;
  void compileE();
  void compileEC();
  /**
   * Extracts single channel chromacity images.
   * As chromacity is only in half resolution in width dimension due to YUYV encoding of the camera,
   * the extracted image get provided in half resolution (width and height).
   * Therefore the values are linearly interpolated in the height dimension.
   * @param eCImage
   */
  void extractChromaticity(ECImage& eCImage);

public:
  ~ECImageProvider();
};
