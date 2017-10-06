/**
 * @author Felix Thielke
 */

#include "CameraSettingsOptimizer.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

MAKE_MODULE(CameraSettingsOptimizer, cognitionInfrastructure)

void CameraSettingsOptimizer::update(CameraSettings& cameraSettings)
{
  // Load config file if no settings were set yet
  if(cameraSettings.upper.settings[CameraSettings::saturation] == -1000 || cameraSettings.lower.settings[CameraSettings::saturation] == -1000)
  {
    InMapFile stream("cameraSettings" + std::string(RobotInfo::getName(theRobotInfo.headVersion)) + ".cfg");
    if(stream.exists())
      stream >> cameraSettings;
  }

  // Only do adjustments if there actually is an image and enough time has passed
  if(theImage.timeStamp - lastOptimizationTimestamps[theCameraInfo.camera] > optimizingDelay && calculateHighestScanline())
  {
    if(doAutoWhiteBalance)
      autoWhiteBalance(cameraSettings[theCameraInfo.camera]);
    if(doAutoExposure)
      autoExposure(cameraSettings[theCameraInfo.camera]);

    lastOptimizationTimestamps[theCameraInfo.camera] = theImage.timeStamp;
  }
}

bool CameraSettingsOptimizer::calculateHighestScanline()
{
  Vector2f highestScanPoint;
  if(!Transformation::robotWithCameraRotationToImage(Vector2f(maxScanpointDistance, 0.f), theCameraMatrix, theCameraInfo, highestScanPoint) ||
     static_cast<int>(highestScanPoint.y()) >= theCameraInfo.height)
    return false;

  highestScanline = static_cast<unsigned short>(std::max<int>(0, static_cast<int>(highestScanPoint.y())));
  return true;
}

void CameraSettingsOptimizer::autoWhiteBalance(CameraSettings::CameraSettingsCollection& settings) const
{
  // Disable camera auto white balance
  if(settings.settings[CameraSettings::autoWhiteBalance])
  {
    settings.settings[CameraSettings::autoWhiteBalance] = 0;
    return;
  }

  // Find lightest pixels
  const Image::Pixel* px = theImage[0] + theImage.width * highestScanline;
  const Image::Pixel* const imgEnd = theImage[theImage.height];
  const ptrdiff_t stepSize = std::max<ptrdiff_t>(1, (imgEnd - px) / whiteBalanceScanPoints);
  unsigned char maxY = 0;
  for(; px < imgEnd; px += stepSize)
  {
    if(px->y)
      maxY = px->y;
  }
  if(maxY < minWhiteLuminance)
    return;

  // Get average hue and saturation of lightest (supposedly white) pixels
  unsigned int sumH = 0;
  unsigned int sumS = 0;
  unsigned int count = 0;
  for(px = theImage[0] + theImage.width * highestScanline; px < imgEnd; px += stepSize)
  {
    if(px->y >= maxY - whiteLuminanceRange)
    {
      unsigned char h, s, i;
      ColorModelConversions::fromYUVToHSI(px->y, px->cb, px->cr, h, s, i);
      sumH += h;
      sumS += s;
      count++;
    }
  }
  const int avgH = sumH / count;
  const int avgS = sumS / count;

  // Shift color temperature towards supposedly real white
  settings.settings[CameraSettings::whiteBalanceTemperature] = settings.settings[CameraSettings::whiteBalanceTemperature] + ((((avgH > 213 ? 213 - avgH : avgH) - 85) * avgS) >> whiteBalanceChangeEffectExponent);
}

void CameraSettingsOptimizer::autoExposure(CameraSettings::CameraSettingsCollection& settings) const
{
  // Disable camera auto exposure
  if(settings.settings[CameraSettings::autoExposure])
  {
    settings.settings[CameraSettings::autoExposure] = 0;
    return;
  }

  // Get old exposure and gain values
  const unsigned short oldExposure = static_cast<unsigned short>(settings.settings[CameraSettings::exposure]);
  const unsigned char oldGain = static_cast<unsigned char>(settings.settings[CameraSettings::gain]);

  // Compute weighted average luminance
  const unsigned char* px = theECImage.grayscaled[highestScanline];
  const unsigned char* const imgEnd = theECImage.grayscaled[theECImage.grayscaled.height];
  const ptrdiff_t stepSize = std::max<ptrdiff_t>(1, (imgEnd - px) / exposureScanPoints);
  unsigned char min = 255;
  unsigned char max = 0;
  unsigned int avg = 0;
  size_t pos = 0;
  float weightSum = 0;
  for(; px < imgEnd; px += stepSize, pos += stepSize)
  {
    const short x = static_cast<short>((((pos << 9) / theECImage.grayscaled.width) & 0x1FF) - 256);
    const short y = static_cast<short>((pos / theECImage.grayscaled.width) * 256 / theECImage.grayscaled.height);
    const short weight = static_cast<short>(sqrt(sqr(x) + sqr(y)));
    weightSum += weight / 256.f;
    avg += (*px * weight) >> 8;
    if(*px < min)
      min = *px;
    if(*px > max)
      max = *px;
  }
  avg = static_cast<unsigned int>(static_cast<float>(avg) / weightSum);
  MODIFY("avgLum", avg);

  // Determine target value
  int diff = 128 - avg; // balance image contrast

  // Clamp exposure and gain values, modifying difference to target value
  unsigned short newExposure = static_cast<unsigned short>(std::max<int>(std::min<int>(oldExposure + diff / 2, maxExposure), minExposure));
  unsigned char newGain = static_cast<unsigned char>(std::max<int>(std::min<int>(oldGain + diff / 2, maxGain), minGain));

  // Set new settings
  settings.settings[CameraSettings::exposure] = (newExposure >> exposureChangeEffectExponent) + oldExposure - (oldExposure >> exposureChangeEffectExponent);
  settings.settings[CameraSettings::gain] = (newGain >> gainChangeEffectExponent) + oldGain - (oldGain >> gainChangeEffectExponent);
}
