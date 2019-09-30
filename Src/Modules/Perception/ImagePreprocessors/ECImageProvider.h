/**
 * @file ECImageProvider.h
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Tools/Module/Module.h"

MODULE(ECImageProvider,
{,
  REQUIRES(FieldColors),
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  PROVIDES(ECImage),
  LOADS_PARAMETERS(
  {,
    (bool) disableClassification,
  }),
});

/**
 * @class ECImageProvider
 */
class ECImageProvider : public ECImageProviderBase
{
private:
  using EcFunc = void (*)(unsigned int, const void*, void*, void*, void*, void*);
  using EFunc = void (*)(unsigned int, const void*, void*);
  uint8_t* currentMaxNonColorSaturation = nullptr;
  uint8_t* currentBlackWhiteDelimiter = nullptr;
  uint8_t* currentFieldHueMin = nullptr;
  uint8_t* currentFieldHueMax = nullptr;

  EcFunc ecFunc;
  EFunc eFunc;

  void update(ECImage& ecImage) override;
  void compileE();
  void compileEC();

public:
  ECImageProvider() : ecFunc(nullptr), eFunc(nullptr) {}
  ~ECImageProvider();
};
