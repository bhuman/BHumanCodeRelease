/**
 * @file CameraSettings.h
 * Declaration of a struct representing the settings of the PDA camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/FixedPoint.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <array>

STREAMABLE(CameraSettings,
{
  ENUM(CameraSetting,
  {,
    autoExposure, /* 1: Use auto exposure, 0: disable auto exposure. */
    autoExposureAlgorithm, /* See enum ExposureAlgorithm. */
    autoExposureBrightness, /* The picture brightness [0 .. 255] auto exposure tries to achieve. */
    //autoExposureBrightnessDark, /* The picture brightness [0 .. 255] auto exposure tries to achieve in dark environments. */
    autoExposureMinVirtAnalogGain, /* [0 .. 32767] ufixed5: 32 == 1.0 */
    autoExposureMaxVirtAnalogGain, /* [0 .. 32767] ufixed5: 32 == 1.0 */
    autoExposureMinVirtDigitalGain, /* [0 .. 32767] ufixed7: 128 == 1.0 */
    autoExposureMaxVirtDigitalGain, /* [0 .. 32767] ufixed7: 128 == 1.0 */
    autoExposureTargetGain, /* [0 .. 32767] ufixed5: 32 == 1.0. Muss > als minAnalogGain * minDigitalGain * 2 sein (in float).*/
    autoWhiteBalance, /* 1: Use auto white balance, 0: disable auto white balance. */
    contrast,   /* The contrast in range of [16 .. 64]. Gradients from 0.5 (16) to 2.0 (64).*/
    exposure, /**< The exposure time in the range of [0 .. 1000]. Time is measured in increments of 100µs. */
    fadeToBlack, /**< Fade to black under low light conditions. 1: enabled, 0: disabled. */
    gain, /**< The gain level in the range of [0 .. 255]. */
    hue, /* The hue in range [-22 .. 22] */
    powerLineFrequency, /* The powerlie frequency, so that auto exposure can avoid flicker (1 == 50Hz, 2 == 60Hz). */
    saturation, /* The saturation in range of [0 .. 255] */
    sharpness, /* The sharpness in range of [-7 .. 7] */
    whiteBalanceTemperature, /**< The white balance temperature in Kelvin [2700 .. 6500] */
  });

  ENUM(ExposureAlgorithm,
  {,
    averageY, /* Average scene brightness. */
    weighted, /* Weighted average brightness (the weights can be controlled by the autoExposureWeightTable). */
    highlights, /* Adaptive weighted auto exposure for highlights. */
    lowlights, /* Adaptive weighted auto exposure for lowlights. */
  });

  ENUM(PowerLineFrequency,
  {,
    _50Hz,
    _60Hz,
  });

  struct CameraSettingsCollection : public Streamable
  {
    std::array<int COMMA numOfCameraSettings> settings;

    CameraSettingsCollection();

  protected:
    void serialize(In* in, Out* out) override;

  private:
    static void reg();
  };

  CameraSettingsCollection& operator[](const CameraInfo::Camera camera)
  {
    return camera == CameraInfo::upper ? upper : lower;
  },

  (CameraSettingsCollection) upper,
  (CameraSettingsCollection) lower,
});
