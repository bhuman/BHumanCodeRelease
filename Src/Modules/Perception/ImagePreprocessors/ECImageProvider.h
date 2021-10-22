/**
 * @file ECImageProvider.h
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Tools/Module/Module.h"

MODULE(ECImageProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  PROVIDES(ECImage),
  LOADS_PARAMETERS(
  {,
    (bool) disableColor,
  }),
});

/**
 * @class ECImageProvider
 */
class ECImageProvider : public ECImageProviderBase
{
private:
  using EcFunc = void (*)(unsigned int, const void*, void*, void*, void*);
  using EFunc = void (*)(unsigned int, const void*, void*);

  EcFunc ecFunc = nullptr;
  EFunc eFunc = nullptr;

  void update(ECImage& ecImage) override;
  void compileE();
  void compileEC();

public:
  ~ECImageProvider();
};
