/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Infrastructure/CameraSettings.h"

#include <array>

MODULE(CameraSettingsOptimizer,
{,
  REQUIRES(RobotInfo),
  USES(CameraInfo),
  USES(CameraMatrix),
  USES(ECImage),
  USES(Image),
  PROVIDES(CameraSettings),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(250) optimizingDelay,
    (float)(2500.f) maxScanpointDistance,
    (bool)(true) doAutoWhiteBalance,
    (bool)(true) doAutoExposure,
    (unsigned short)(256) whiteBalanceScanPoints,
    (unsigned char)(160) minWhiteLuminance,
    (unsigned char)(30) whiteLuminanceRange,
    (unsigned char)(4) whiteBalanceChangeEffectExponent,
    (unsigned short)(128) exposureScanPoints,
    (unsigned short)(0) minExposure,
    (unsigned short)(160) maxExposure,
    (unsigned char)(64) minGain,
    (unsigned char)(200) maxGain,
    (unsigned char)(2) exposureChangeEffectExponent,
    (unsigned char)(2) gainChangeEffectExponent,
  }),
});

class CameraSettingsOptimizer : public CameraSettingsOptimizerBase
{
private:
  std::array<unsigned int, CameraInfo::numOfCameras> lastOptimizationTimestamps;
  unsigned short highestScanline;

  void update(CameraSettings& cameraSettings);

  bool calculateHighestScanline();

  void autoWhiteBalance(CameraSettings::CameraSettingCollection& settings) const;
  void autoExposure(CameraSettings::CameraSettingCollection& settings) const;

public:
  CameraSettingsOptimizer()
  {
    lastOptimizationTimestamps[CameraInfo::upper] = lastOptimizationTimestamps[CameraInfo::lower] = 0;
  }
};
