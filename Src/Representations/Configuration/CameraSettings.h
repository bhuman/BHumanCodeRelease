/**
* @file CameraSettings.h
* Declaration of a class representing the settings of the PDA camera.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <list>
#include "Tools/Streams/AutoStreamable.h"

/**
* @class Properties
* The class represents the properties of the camera.
*/
STREAMABLE(CameraSettings,
{
private:
  /**
   * Initializes everything with invalid values except the settings for auto features.
   * The settings for auto features are initialized so that they disable the
   * features by default.
   */
  void init();

public:
  STREAMABLE(V4L2Setting,
  {
  public:
    inline V4L2Setting(int command, int value);
    bool operator==(const V4L2Setting& o) const {return command == o.command && value == o.value;}
    bool operator!=(const V4L2Setting& o) const {return !(*this == o);}

    int command,
    (int) value, // only stream value
  });

  bool operator==(const CameraSettings& o) const;
  bool operator!=(const CameraSettings& o) const {return !(*this == o);}
  std::list<V4L2Setting> getChangesAndAssign(const CameraSettings& other);
  std::list<V4L2Setting> getSettings() const;
  void setSetting(const V4L2Setting& setting);

  static const int numSettings = 10,
  (V4L2Setting) autoWhiteBalance, /* 1: Use auto white balance, 0: disable auto white balance. */
  (V4L2Setting) autoExposure, /* 1: Use auto exposure, 0: disable auto exposure. */
  (V4L2Setting) contrast,   /* The contrast in range of [0 .. 127] */
  (V4L2Setting) saturation, /* The saturation in range of [0 .. 255] */
  (V4L2Setting) hue, /* The hue in range [-180 .. 180] */
  (V4L2Setting) sharpness, /* The sharpness in range of [0 .. 31] */
  (V4L2Setting) exposure, /**< The exposure time in the range of [0 .. 1023]. */
  (V4L2Setting) gain, /**< The gain level in the range of [0 .. 127]. */
  (V4L2Setting) whiteBalance, /**< The white balance in Kelvin [2700 .. 6500] */
  (V4L2Setting) fadeToBlack, /**< Fade to black under low light conditions */

  // Initialization
  init();
});

CameraSettings::V4L2Setting::V4L2Setting(int command, int value)
: command(command),
  value(value) {}
