/**
 * @file CameraSettings.h
 *
 * This file declares a representation that stores the settings for NAO V6 cameras.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <array>

STREAMABLE(CameraSettings,
{
  struct Collection : public Streamable
  {
    ENUM(CameraSetting,
    {,
      autoExposure, /**< Auto exposure mode (0: on, 1: off). */
      autoExposureBrightness, /**< The brightness the auto exposure tries to achieve in the range [-255 .. 255]. */
      exposure, /**< The exposure time in the range [0 .. 1048575] when auto exposure is disabled. */
      gain, /**< The gain level in the range [0 .. 1023]. */
      autoWhiteBalance, /**< 1: Use auto white balance, 0: disable auto white balance, current white balance is frozen. */
      autoFocus, /**< 1: Use auto focus, 0: disable auto focus. */
      focus, /**< The focus in the range [0 .. 250] in steps of 25 when auto focus is disabled. */
      autoHue, /**< 1: Use auto hue, 0: disable auto hue. */
      hue, /**< The hue in the range [-180 .. 180] when auto hue is disabled. */
      saturation, /**< The saturation in the range [0 .. 255]. */
      contrast, /**< The contrast in the range [0 .. 255]. */
      sharpness, /**< The sharpness in the range [0 .. 9]. */
      redGain, /**< The red gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */
      greenGain, /**< The green gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */
      blueGain, /**< The blue gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */
    });

    std::array<int COMMA numOfCameraSettings> settings;

    Collection();

  protected:
    void serialize(In* in, Out* out) override;

  private:
    static void reg();
  },

  (ENUM_INDEXED_ARRAY(Collection, CameraInfo::Camera)) cameras,
});
