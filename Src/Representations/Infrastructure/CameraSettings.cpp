/**
 * @file CameraSettings.cpp
 * Implementation of struct CameraSettings.
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

#include <limits>

#ifdef CAMERA_INCLUDED
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#define __STRICT_ANSI__

#define V4L2_MT9M114_FADE_TO_BLACK V4L2_CID_PRIVATE_BASE
#endif

CameraSettings::V4L2Setting::V4L2Setting() :
  V4L2Setting(0, 0, std::numeric_limits<int>::min(), std::numeric_limits<int>::max())
{}

CameraSettings::V4L2Setting::V4L2Setting(int command, int value, int min, int max) :
  command(command), min(min), max(max), value(value)
{
  ASSERT(min <= max);
  for(CameraSetting& influenced : influencingSettings)
    influenced = numOfCameraSettings;
}

bool CameraSettings::V4L2Setting::operator==(const V4L2Setting& other) const
{
  return command == other.command && value == other.value;
}

bool CameraSettings::V4L2Setting::operator!=(const V4L2Setting& other) const
{
  return !(*this == other);
}

void CameraSettings::V4L2Setting::enforceBounds()
{
  if(value < min)
    value = min;
  else if(value > max)
    value = max;
}

CameraSettings::CameraSettingCollection::CameraSettingCollection()
{
#ifdef CAMERA_INCLUDED
  settings[autoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000, 0, 1);
  settings[autoExposure].influencingSettings[0] = exposure;
  settings[autoExposure].influencingSettings[1] = gain;
  settings[autoExposureAlgorithm] = V4L2Setting(V4L2_CID_EXPOSURE_ALGORITHM, -1000, 0, 3);
  settings[autoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, -1000, 0, 1);
  settings[autoWhiteBalance].influencingSettings[0] = whiteBalance;
  settings[brightness] = V4L2Setting(V4L2_CID_BRIGHTNESS, -1000, 0, 255);
  settings[contrast] = V4L2Setting(V4L2_CID_CONTRAST, -1000, 16, 64);
  settings[exposure] = V4L2Setting(V4L2_CID_EXPOSURE, -1000, 0, 333);
  settings[fadeToBlack] = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, -1000, 0, 1);
  settings[gain] = V4L2Setting(V4L2_CID_GAIN, -1000, 32, 255);
  settings[hue] = V4L2Setting(V4L2_CID_HUE, -1000, -22, 22);
  settings[saturation] = V4L2Setting(V4L2_CID_SATURATION, -1000, 0, 255);
  settings[sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, -1000, -7, 7);
  settings[whiteBalance] = V4L2Setting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, -1000, 2700, 6500);
#endif
}

bool CameraSettings::CameraSettingCollection::operator==(const CameraSettingCollection& other) const
{
  FOREACH_ENUM(CameraSetting, i)
  {
    if(settings[i] != other.settings[i])
      return false;
  }
  return true;
}

bool CameraSettings::CameraSettingCollection::operator!=(const CameraSettingCollection& other) const
{
  return !(*this == other);
}

void CameraSettings::CameraSettingCollection::enforceBounds()
{
  FOREACH_ENUM(CameraSetting, i)
    settings[i].enforceBounds();
}

void CameraSettings::CameraSettingCollection::serialize(In* in, Out* out)
{
  V4L2Setting& autoExposure = settings[CameraSettings::autoExposure];
  ExposureAlgorithm autoExposureAlgorithm = (ExposureAlgorithm)settings[CameraSettings::autoExposureAlgorithm].value;
  V4L2Setting& autoWhiteBalance = settings[CameraSettings::autoWhiteBalance];
  V4L2Setting& brightness = settings[CameraSettings::brightness];
  V4L2Setting& contrast = settings[CameraSettings::contrast];
  V4L2Setting& exposure = settings[CameraSettings::exposure];
  V4L2Setting& fadeToBlack = settings[CameraSettings::fadeToBlack];
  V4L2Setting& gain = settings[CameraSettings::gain];
  V4L2Setting& hue = settings[CameraSettings::hue];
  V4L2Setting& saturation = settings[CameraSettings::saturation];
  V4L2Setting& sharpness = settings[CameraSettings::sharpness];
  V4L2Setting& whiteBalance = settings[CameraSettings::whiteBalance];

  STREAM_REGISTER_BEGIN;
  STREAM(autoExposure);
  STREAM(autoExposureAlgorithm, CameraSettings);
  STREAM(autoWhiteBalance);
  STREAM(brightness);
  STREAM(contrast);
  STREAM(exposure);
  STREAM(fadeToBlack);
  STREAM(gain);
  STREAM(hue);
  STREAM(saturation);
  STREAM(sharpness);
  STREAM(whiteBalance);
  STREAM_REGISTER_FINISH;

  settings[CameraSettings::autoExposureAlgorithm].value = (int)autoExposureAlgorithm;

  if(in)
    enforceBounds();
}
