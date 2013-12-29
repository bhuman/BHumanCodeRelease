/**
* @file CameraSettings.cpp
* Implementation of class CameraSettings.
*
* @author Felix Wenk
*/

#include "CameraSettings.h"
#include "Platform/Camera.h"

#ifdef CAMERA_INCLUDED
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#define __STRICT_ANSI__

#define V4L2_MT9M114_FADE_TO_BLACK   V4L2_CID_PRIVATE_BASE

void CameraSettings::init()
{
  autoWhiteBalance = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, -1000);
  autoExposure = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, -1000);
  contrast = V4L2Setting(V4L2_CID_CONTRAST, -1000);
  saturation = V4L2Setting(V4L2_CID_SATURATION, -1000);
  hue = V4L2Setting(V4L2_CID_HUE, -1000);
  sharpness = V4L2Setting(V4L2_CID_SHARPNESS, -1000);
  exposure = V4L2Setting(V4L2_CID_EXPOSURE, -1000);
  gain = V4L2Setting(V4L2_CID_GAIN, -1000);
  whiteBalance = V4L2Setting(V4L2_CID_DO_WHITE_BALANCE, -1000);
  fadeToBlack = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, -1000);
}
#else // !CAMERA_INCLUDED
void CameraSettings::init() {}
#endif

bool CameraSettings::operator==(const CameraSettings& o) const
{
  const V4L2Setting* thisSettings = &autoWhiteBalance;
  const V4L2Setting* otherSettings = &o.autoWhiteBalance;
  for(int i = 0; i < numSettings; i++)
    if(thisSettings[i] != otherSettings[i])
      return false;
  return true;
}

std::list<CameraSettings::V4L2Setting> CameraSettings::getChangesAndAssign(const CameraSettings& other)
{
  std::list<V4L2Setting> changes;
  V4L2Setting* thisSettings = &autoWhiteBalance;
  const V4L2Setting* otherSettings = &other.autoWhiteBalance;
  for(int i = 0; i < numSettings; i++)
  {
    if(thisSettings[i] != otherSettings[i])
    {
      thisSettings[i] = otherSettings[i];
      changes.push_back(thisSettings[i]);
    }
  }
  return changes;
}

std::list<CameraSettings::V4L2Setting> CameraSettings::getSettings() const
{
  std::list<V4L2Setting> settings;
  const V4L2Setting* start = &autoWhiteBalance;
  for(int i = 0; i < numSettings; i++)
    settings.push_back(start[i]);
  return settings;
}

void CameraSettings::setSetting(const V4L2Setting& setting)
{
  V4L2Setting* start = &autoWhiteBalance;
  for(int i = 0; i < numSettings; ++i)
    if(start[i].command == setting.command)
    {
      start[i].value = setting.value;
      break;
    }
}
