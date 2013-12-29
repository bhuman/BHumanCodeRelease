/**
* @author marcel
* @author arne
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Perception/ColorReference.h"

class Image;

MODULE(ColorProvider)
  REQUIRES(Image)
  PROVIDES_WITH_OUTPUT(ColorReference)

  /** green */
  LOADS_PARAMETER(float, minHGreen)
  LOADS_PARAMETER(float, maxHGreen)
  LOADS_PARAMETER(float, minSGreen)
  LOADS_PARAMETER(float, maxSGreen)
  LOADS_PARAMETER(float, minVGreen)
  LOADS_PARAMETER(float, maxVGreen)

  /** yellow */
  LOADS_PARAMETER(float, minHYellow)
  LOADS_PARAMETER(float, maxHYellow)
  LOADS_PARAMETER(float, minSYellow)
  LOADS_PARAMETER(float, maxSYellow)
  LOADS_PARAMETER(float, minVYellow)
  LOADS_PARAMETER(float, maxVYellow)

  /** orange */
  LOADS_PARAMETER(float, minHOrange)
  LOADS_PARAMETER(float, maxHOrange)
  LOADS_PARAMETER(float, minSOrange)
  LOADS_PARAMETER(float, maxSOrange)
  LOADS_PARAMETER(float, minVOrange)
  LOADS_PARAMETER(float, maxVOrange)

  /** red */
  LOADS_PARAMETER(float, minHRed)
  LOADS_PARAMETER(float, maxHRed)
  LOADS_PARAMETER(float, minSRed)
  LOADS_PARAMETER(float, maxSRed)
  LOADS_PARAMETER(float, minVRed)
  LOADS_PARAMETER(float, maxVRed)

  /** blue */
  LOADS_PARAMETER(float, minHBlue)
  LOADS_PARAMETER(float, maxHBlue)
  LOADS_PARAMETER(float, minSBlue)
  LOADS_PARAMETER(float, maxSBlue)
  LOADS_PARAMETER(float, minVBlue)
  LOADS_PARAMETER(float, maxVBlue)

  /** white */
  LOADS_PARAMETER(int, minRWhite)
  LOADS_PARAMETER(int, minBWhite)
  LOADS_PARAMETER(int, minRBWhite)

  /** black */
  LOADS_PARAMETER(int, cbBlack)
  LOADS_PARAMETER(int, crBlack)
  LOADS_PARAMETER(int, maxYBlack)
END_MODULE

/**
 * Classify colors by using hsv.
 */
class ColorProvider : public ColorProviderBase
{
public:
  bool setupColorTable;

  ColorProvider() : setupColorTable(true) {}

private:
  bool compareAndSet(float& toSet, const float toCompare);

  bool compareAndSet(int& toSet, const int toCompare);

  bool compareAndSet(Range<float>& toSet,
                     const float min,
                     const float max);

  bool compareAndSet(ColorReference::HSVColorDefinition& toSet,
                     const float minH, const float maxH,
                     const float minS, const float maxS,
                     const float minV, const float maxV);

  /* the update method for ColorReference. */
  void update(ColorReference& colorReference);

  /* calculate the expected color green. */
  bool calculateGreen(ColorReference& colorReference);

  /** calculate the expected color white. */
  bool calculateWhite(ColorReference& colorReference);

  /** calculate the expected color yellow. */
  bool calculateYellow(ColorReference& colorReference);

  /** calculates the expected color orange. */
  bool calculateOrange(ColorReference& colorReference);

  /** calculates the expected color red. */
  bool calculateRed(ColorReference& colorReference);

  /** calculates the expected color blue. */
  bool calculateBlue(ColorReference& colorReference);

  /** calculates the expected color black. */
  bool calculateBlack(ColorReference& colorReference);
  
  /* Streams the ColorReference if necessary. */
  void streamColorReference(ColorReference& colorReference);
};
