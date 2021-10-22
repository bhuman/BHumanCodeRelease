/**
 * @file RelativeFieldColorsProvider.h
 *
 * This file declares a module that approximates the current image's average
 * luminance and saturation for discerning field and white.
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once

#include "Representations/Configuration/RelativeFieldColorsParameters.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Tools/Module/Module.h"

MODULE(RelativeFieldColorsProvider,
{,
 REQUIRES(ECImage),
 REQUIRES(ScanGrid),
 REQUIRES(RelativeFieldColorsParameters),
 PROVIDES(RelativeFieldColors),
});

class RelativeFieldColorsProvider : public RelativeFieldColorsProviderBase
{
  /**
   * Updates the RelativeFieldColors.
   * @param theRelativeFieldColors The provided representation.
   */
  void update(RelativeFieldColors& theRelativeFieldColors) override;

  /**
   * Approximates an average value over the image.
   * @param image The image.
   * @return The approximated average.
   */
  [[nodiscard]] float approximateAverage(const Image<PixelTypes::GrayscaledPixel>& image) const;

private:
  bool rfcParametersSet = false;
};