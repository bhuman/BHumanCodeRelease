/**
 * @file CameraSettings.h
 * Declaration of a struct representing the settings of the PDA camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <array>

STREAMABLE(CameraSettings,
{
  ENUM(CameraSetting,
  {,
    autoExposure, /* 1: Use auto exposure, 0: disable auto exposure. */
    autoExposureAlgorithm, /* See enum ExposureAlgorithm. */
    autoWhiteBalance, /* 1: Use auto white balance, 0: disable auto white balance. */
    brightness, /* The picture brightness [0 .. 255] auto exposure algorithms try to achieve. */
    contrast,   /* The contrast in range of [16 .. 64]. Gradients from 0.5 (16) to 2.0 (64).*/
    exposure, /**< The exposure time in the range of [0 .. 1000]. Time is measured in increments of 100µs. */
    fadeToBlack, /**< Fade to black under low light conditions. 1: enabled, 0: disabled. */
    gain, /**< The gain level in the range of [0 .. 255]. */
    hue, /* The hue in range [-22 .. 22] */
    saturation, /* The saturation in range of [0 .. 255] */
    sharpness, /* The sharpness in range of [-7 .. 7] */
    whiteBalance, /**< The white balance in Kelvin [2700 .. 6500] */
  });

  ENUM(ExposureAlgorithm,
  {,
    averageY,       /* Average scene brightness. */
    centerWeighted, /* Weighted average brightness with highest weights in the center of the image. */
    highlights,     /* Adaptive weighted auto exposure for highlights. */
    lowlights,      /* Adaptive weighted auto exposure for lowlights. */
  });

  STREAMABLE(V4L2Setting,
  {
    int command;

  private:
    int min;
    int max;

  public:
    std::array<CameraSetting COMMA 2> influencingSettings;

    V4L2Setting();
    V4L2Setting(int command, int value, int min, int max);
    virtual ~V4L2Setting() = default;

    bool operator==(const V4L2Setting & other) const;
    bool operator!=(const V4L2Setting & other) const;

    void enforceBounds(),

    (int) value,
  });

  struct CameraSettingCollection : public Streamable
  {
    std::array<V4L2Setting COMMA numOfCameraSettings> settings;

    CameraSettingCollection();
    virtual ~CameraSettingCollection() = default;

    bool operator==(const CameraSettingCollection & other) const;
    bool operator!=(const CameraSettingCollection & other) const;

    void enforceBounds();

    virtual void serialize(In * in, Out * out);
  };

  CameraSettingCollection& operator[](const CameraInfo::Camera camera)
  {
    return camera == CameraInfo::upper ? upper : lower;
  },

  (CameraSettingCollection) upper,
  (CameraSettingCollection) lower,
});
