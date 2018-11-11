/**
 * @file CameraSettings.cpp
 * Implementation of CameraSettings.
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

CameraSettings::CameraSettingsCollection::CameraSettingsCollection()
{
  settings.fill(-1000);
}

void CameraSettings::CameraSettingsCollection::serialize(In* in, Out* out)
{
  bool autoExposure = settings[CameraSettings::autoExposure] != 0;
  ExposureAlgorithm autoExposureAlgorithm = static_cast<ExposureAlgorithm>(settings[CameraSettings::autoExposureAlgorithm]);
  int& autoExposureBrightness = settings[CameraSettings::autoExposureBrightness];
  //int& autoExposureBrightnessDark = settings[CameraSettings::autoExposureBrightnessDark];
  float autoExposureMinVirtAnalogGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureMinVirtAnalogGain]);
  float autoExposureMaxVirtAnalogGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureMaxVirtAnalogGain]);
  float autoExposureMinVirtDigitalGain = FixedPoint7::fromRaw(settings[CameraSettings::autoExposureMinVirtDigitalGain]);
  float autoExposureMaxVirtDigitalGain = FixedPoint7::fromRaw(settings[CameraSettings::autoExposureMaxVirtDigitalGain]);
  float autoExposureTargetGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureTargetGain]);
  bool autoWhiteBalance = settings[CameraSettings::autoWhiteBalance] != 0;
  float contrast = FixedPoint5::fromRaw(settings[CameraSettings::contrast]);
  int& exposure = settings[CameraSettings::exposure];
  bool fadeToBlack = settings[CameraSettings::fadeToBlack] != 0;
  //float gain = FixedPoint5::fromRaw(settings[CameraSettings::gain]);
  int& gain = settings[CameraSettings::gain];
  int& hue = settings[CameraSettings::hue];
  PowerLineFrequency powerLineFrequency = static_cast<PowerLineFrequency>(settings[CameraSettings::powerLineFrequency] - 1);
  //float saturation = FixedPoint7::fromRaw(settings[CameraSettings::saturation]);
  int& saturation = settings[CameraSettings::saturation];
  int& sharpness = settings[CameraSettings::sharpness];
  int& whiteBalanceTemperature = settings[CameraSettings::whiteBalanceTemperature];

  STREAM(autoExposure);
  STREAM(autoExposureAlgorithm);
  STREAM(autoExposureBrightness);
  //STREAM(autoExposureBrightnessDark);
  STREAM(autoExposureMinVirtAnalogGain);
  STREAM(autoExposureMaxVirtAnalogGain);
  STREAM(autoExposureMinVirtDigitalGain);
  STREAM(autoExposureMaxVirtDigitalGain);
  STREAM(autoExposureTargetGain);
  STREAM(autoWhiteBalance);
  STREAM(contrast);
  STREAM(exposure);
  STREAM(fadeToBlack);
  STREAM(gain);
  STREAM(hue);
  STREAM(powerLineFrequency);
  STREAM(saturation);
  STREAM(sharpness);
  STREAM(whiteBalanceTemperature);

  if(in)
  {
    settings[CameraSettings::autoExposure] = autoExposure ? 1 : 0;
    settings[CameraSettings::autoExposureAlgorithm] = static_cast<int>(autoExposureAlgorithm);
    settings[CameraSettings::autoExposureMinVirtAnalogGain] = FixedPoint5(autoExposureMinVirtAnalogGain).getRaw();
    settings[CameraSettings::autoExposureMaxVirtAnalogGain] = FixedPoint5(autoExposureMaxVirtAnalogGain).getRaw();
    settings[CameraSettings::autoExposureMinVirtDigitalGain] = FixedPoint7(autoExposureMinVirtDigitalGain).getRaw();
    settings[CameraSettings::autoExposureMaxVirtDigitalGain] = FixedPoint7(autoExposureMaxVirtDigitalGain).getRaw();
    settings[CameraSettings::autoExposureTargetGain] = FixedPoint5(autoExposureTargetGain).getRaw();
    settings[CameraSettings::autoWhiteBalance] = autoWhiteBalance ? 1 : 0;
    settings[CameraSettings::contrast] = FixedPoint5(contrast).getRaw();
    settings[CameraSettings::fadeToBlack] = fadeToBlack ? 1 : 0;
    //settings[CameraSettings::gain] = FixedPoint5(gain).getRaw();
    settings[CameraSettings::powerLineFrequency] = powerLineFrequency + 1;
    //settings[CameraSettings::saturation] = FixedPoint7(saturation).getRaw();
  }
}

void CameraSettings::CameraSettingsCollection::reg()
{
  PUBLISH(reg);
  REG_CLASS(CameraSettingsCollection);
  REG(bool, autoExposure);
  REG(ExposureAlgorithm, autoExposureAlgorithm);
  REG(int, autoExposureBrightness);
  //REG(int, autoExposureBrightnessDark);
  REG(float, autoExposureMinVirtAnalogGain);
  REG(float, autoExposureMaxVirtAnalogGain);
  REG(float, autoExposureMinVirtDigitalGain);
  REG(float, autoExposureMaxVirtDigitalGain);
  REG(float, autoExposureTargetGain);
  REG(bool, autoWhiteBalance);
  REG(float, contrast);
  REG(int, exposure);
  REG(bool, fadeToBlack);
  REG(int, gain);
  REG(int, hue);
  REG(PowerLineFrequency, powerLineFrequency);
  REG(int, saturation);
  REG(int, sharpness);
  REG(int, whiteBalanceTemperature);
}
