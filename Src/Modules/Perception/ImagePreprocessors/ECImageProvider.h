/**
 * @file ColorClassifier.h
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
  LOADS_PARAMETERS(
  {
    ENUM(Mode,
    {,
      yhs,
      yhs2,
      hsi,
    }),
    (bool) sseOptimized,
    (bool) simpleClassification,
    (Mode) mode,
  }),
});

/**
 * @class ECImageProvider
 */
class ECImageProvider : public ECImageProviderBase
{
private:
  void update(ECImage& ecImage);
  template<bool simple> void updateSSE(ECImage& ecImage);

  template<bool isSmall> void classifyByYHSFieldColor(ECImage& ecImage) const;

  template<bool aligned, bool avx> void classifyByYHSFieldColorSSE(ECImage& ecImage) const;
  template<bool aligned, bool avx, bool simple> void classifyByYHS2FieldColorSSE(ECImage& ecImage) const;
  template<bool aligned, bool avx, bool simple> void classifyByHSIFieldColorSSE(ECImage& ecImage) const;
};
