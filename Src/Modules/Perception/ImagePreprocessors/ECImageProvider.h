/**
 * @file ECImageProvider.h
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Configuration/FieldColors.h"

MODULE(ECImageProvider,
{,
  REQUIRES(FieldColors),
  REQUIRES(CameraInfo),
  REQUIRES(Image),
  PROVIDES(ECImage),
  DEFINES_PARAMETERS(
  {,
    (bool)(true) hueIsNeeded,
  }),
});

/**
 * @class ECImageProvider
 */
class ECImageProvider : public ECImageProviderBase
{
private:
  void update(ECImage& ecImage);
  template<bool saveHue> void update(ECImage& ecImage);
};
