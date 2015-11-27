/**
 * @file CameraSettings.h
 * Declaration of a struct representing the settings of the PDA camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include <array>

struct CameraSettings : public Streamable
{
  ENUM(CameraSetting,
  {,
    AutoExposure, /* 1: Use auto exposure, 0: disable auto exposure. */
    AutoWhiteBalance, /* 1: Use auto white balance, 0: disable auto white balance. */
    Contrast,   /* The contrast in range of [16 .. 64]. Gravients from 0.5 (16) to 2.0 (64).*/
    Exposure, /**< The exposure time in the range of [0 .. 1000]. Time is measured in increments of 100µs. */
    FadeToBlack, /**< Fade to black under low light conditions. 1: enabled, 0: disabled. */
    Gain, /**< The gain level in the range of [0 .. 255]. */
    Hue, /* The hue in range [-22 .. 22] */
    Saturation, /* The saturation in range of [0 .. 255] */
    Sharpness, /* The sharpness in range of [-7 .. 7] */
    WhiteBalance, /**< The white balance in Kelvin [2700 .. 6500] */
  });

  STREAMABLE(V4L2Setting,
  {
    int command;

  private:
    int min;
    int max;

  public:
    PROTECT(std::array<CameraSetting, 2>) influencingSettings;

    V4L2Setting();
    V4L2Setting(int command, int value, int min, int max);
    virtual ~V4L2Setting() = default;

    bool operator==(const V4L2Setting& other) const;
    bool operator!=(const V4L2Setting& other) const;

    void enforceBounds(),

    (int) value,
  });

  CameraInfo::Camera camera;
  std::array<V4L2Setting, numOfCameraSettings> settings;

  CameraSettings() = default;
  CameraSettings(CameraInfo::Camera camera);
  virtual ~CameraSettings() = default;

  bool operator==(const CameraSettings& other) const;
  bool operator!=(const CameraSettings& other) const;

  void enforceBounds();

  virtual void serialize(In* in, Out* out);
};
