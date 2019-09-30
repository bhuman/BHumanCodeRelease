/**
 * @file CameraSettings.cpp
 *
 * This file implements a representation that stores the settings for NAO V6 cameras.
 *
 * @author Thomas Röfer
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

CameraSettings::Collection::Collection()
{
  settings.fill(-1000);
}

void CameraSettings::Collection::serialize(In* in, Out* out)
{
  bool autoExposure = settings[Collection::autoExposure] == 0;
  int& autoExposureBrightness = settings[Collection::autoExposureBrightness];
  int& exposure = settings[Collection::exposure];
  int& gain = settings[Collection::gain];
  bool autoWhiteBalance = settings[Collection::autoWhiteBalance] != 0;
  bool autoFocus = settings[Collection::autoFocus] != 0;
  int& focus = settings[Collection::focus];
  bool autoHue = settings[Collection::autoHue] != 0;
  int& hue = settings[Collection::hue];
  int& saturation = settings[Collection::saturation];
  int& contrast = settings[Collection::contrast];
  int& sharpness = settings[Collection::sharpness];
  int& redGain = settings[Collection::redGain];
  int& greenGain = settings[Collection::greenGain];
  int& blueGain = settings[Collection::blueGain];

  STREAM(autoExposure);
  STREAM(autoExposureBrightness);
  STREAM(exposure);
  STREAM(gain);
  STREAM(autoWhiteBalance);
  STREAM(autoFocus);
  STREAM(focus);
  STREAM(autoHue);
  STREAM(hue);
  STREAM(saturation);
  STREAM(contrast);
  STREAM(sharpness);
  STREAM(redGain);
  STREAM(greenGain);
  STREAM(blueGain);

  if(in)
  {
    settings[Collection::autoExposure] = autoExposure ? 0 : 1;
    settings[Collection::autoWhiteBalance] = autoWhiteBalance ? 1 : 0;
    settings[Collection::autoFocus] = autoFocus ? 1 : 0;
    settings[Collection::autoHue] = autoHue ? 1 : 0;
  }
}

void CameraSettings::Collection::reg()
{
  PUBLISH(reg);
  REG_CLASS(Collection);
  REG(bool, autoExposure);
  REG(int, autoExposureBrightness);
  REG(int, exposure);
  REG(int, gain);
  REG(bool, autoWhiteBalance);
  REG(bool, autoFocus);
  REG(int, focus);
  REG(bool, autoHue);
  REG(int, hue);
  REG(int, saturation);
  REG(int, contrast);
  REG(int, sharpness);
  REG(int, redGain);
  REG(int, greenGain);
  REG(int, blueGain);
}
