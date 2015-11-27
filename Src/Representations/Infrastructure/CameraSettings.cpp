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

CameraSettings::CameraSettings(CameraInfo::Camera camera) :
  camera(camera)
#ifdef CAMERA_INCLUDED
{
  settings[AutoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000, 0, 1);
  settings[AutoExposure].influencingSettings[0] = Exposure;
  settings[AutoExposure].influencingSettings[1] = Gain;
  settings[AutoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, -1000, 0, 1);
  settings[AutoWhiteBalance].influencingSettings[0] = WhiteBalance;
  settings[Contrast] = V4L2Setting(V4L2_CID_CONTRAST, -1000, 16, 64);
  settings[Exposure] = V4L2Setting(V4L2_CID_EXPOSURE, -1000, 0, 1000);
  settings[FadeToBlack] = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, -1000, 0, 1);
  settings[Gain] = V4L2Setting(V4L2_CID_GAIN, -1000, 0, 255);
  settings[Hue] = V4L2Setting(V4L2_CID_HUE, -1000, -22, 22);
  settings[Saturation] = V4L2Setting(V4L2_CID_SATURATION, -1000, 0, 255);
  settings[Sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, -1000, -7, 7);
  settings[WhiteBalance] = V4L2Setting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, -1000, 2700, 6500);
}
#else
{}
#endif

bool CameraSettings::operator==(const CameraSettings& other) const
{
  for(int i = 0; i < numOfCameraSettings; ++i)
  {
    if(settings[i] != other.settings[i])
      return false;
  }
  return true;
}

bool CameraSettings::operator!=(const CameraSettings& other) const
{
  return !(*this == other);
}

void CameraSettings::enforceBounds()
{
  for(int i = 0; i < numOfCameraSettings; ++i)
    settings[i].enforceBounds();
}

void CameraSettings::serialize(In* in, Out* out)
{
  V4L2Setting& autoExposure = settings[AutoExposure];
  V4L2Setting& autoWhiteBalance = settings[AutoWhiteBalance];
  V4L2Setting& contrast = settings[Contrast];
  V4L2Setting& exposure = settings[Exposure];
  V4L2Setting& fadeToBlack = settings[FadeToBlack];
  V4L2Setting& gain = settings[Gain];
  V4L2Setting& hue = settings[Hue];
  V4L2Setting& saturation = settings[Saturation];
  V4L2Setting& sharpness = settings[Sharpness];
  V4L2Setting& whiteBalance = settings[WhiteBalance];

  STREAM_REGISTER_BEGIN;
  STREAM(camera, CameraInfo);
  STREAM(autoExposure);
  STREAM(autoWhiteBalance);
  STREAM(contrast);
  STREAM(exposure);
  STREAM(fadeToBlack);
  STREAM(gain);
  STREAM(hue);
  STREAM(saturation);
  STREAM(sharpness);
  STREAM(whiteBalance);
  STREAM_REGISTER_FINISH;

  if(in)
    enforceBounds();
}
