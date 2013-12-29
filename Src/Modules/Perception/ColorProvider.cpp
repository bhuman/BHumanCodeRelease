/**
* @author marcel
*/

#include "ColorProvider.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Global.h"

MAKE_MODULE(ColorProvider, Perception)

bool ColorProvider::compareAndSet(float& toSet, const float toCompare)
{
  if (toSet != toCompare)
  {
    toSet = toCompare;
    return true;
  }
  return false;
}

bool ColorProvider::compareAndSet(int& toSet, const int toCompare)
{
  if (toSet != toCompare)
  {
    toSet = toCompare;
    return true;
  }
  return false;
}

bool ColorProvider::compareAndSet(Range<float>& toSet,
                                  const float min,
                                  const float max)
{
  return compareAndSet(toSet.min, min) |
         compareAndSet(toSet.max, max);
}

bool ColorProvider::compareAndSet(ColorReference::HSVColorDefinition& toSet,
                                  const float minH, const float maxH,
                                  const float minS, const float maxS,
                                  const float minV, const float maxV)
{
  return compareAndSet(toSet.hue, minH, maxH) |
         compareAndSet(toSet.saturation, minS, maxS) |
         compareAndSet(toSet.value, minV, maxV);
}

void ColorProvider::update(ColorReference& colorReference)
{
  // calculate the colors and update the color table if necessary
  if(setupColorTable                  |
     calculateGreen(colorReference)   |
     calculateWhite(colorReference)   |
     calculateYellow(colorReference)  |
     calculateOrange(colorReference)  |
     calculateRed(colorReference)     |
     calculateBlue(colorReference)    |
     calculateBlack(colorReference))
  {
    colorReference.update();
    setupColorTable = false;
    colorReference.changed = true;
  }

  // stream the ColorReference if necessary
  streamColorReference(colorReference);

  // this is used to get the initial ColorReference
  DEBUG_RESPONSE_ONCE("representation:ColorReference",
  {
    OUTPUT(idColorReference, bin, colorReference);
  });
}

bool ColorProvider::calculateGreen(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdGreen,
                       minHGreen, maxHGreen,
                       minSGreen, maxSGreen,
                       minVGreen, maxVGreen);
}

bool ColorProvider::calculateWhite(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdWhite.first, minRWhite)  |
         compareAndSet(colorReference.thresholdWhite.second, minBWhite) |
         compareAndSet(colorReference.thresholdWhite.third, minRBWhite);
}

bool ColorProvider::calculateYellow(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdYellow,
                       minHYellow, maxHYellow,
                       minSYellow, maxSYellow,
                       minVYellow, maxVYellow);
}

bool ColorProvider::calculateOrange(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdOrange,
                       minHOrange, maxHOrange,
                       minSOrange, maxSOrange,
                       minVOrange, maxVOrange);
}

bool ColorProvider::calculateRed(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdRed,
                       minHRed, maxHRed,
                       minSRed, maxSRed,
                       minVRed, maxVRed);
}

bool ColorProvider::calculateBlue(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdBlue,
                       minHBlue, maxHBlue,
                       minSBlue, maxSBlue,
                       minVBlue, maxVBlue);
}

bool ColorProvider::calculateBlack(ColorReference& colorReference)
{
  return compareAndSet(colorReference.thresholdBlack.first, cbBlack)  |
         compareAndSet(colorReference.thresholdBlack.second, crBlack) |
         compareAndSet(colorReference.thresholdBlack.third, maxYBlack);
}

void ColorProvider::streamColorReference(ColorReference& colorReference)
{
  if(colorReference.changed)
  {
    Global::getDebugOut().bin << colorReference;
    if(Global::getDebugOut().finishMessage(idColorReference))
    {
      colorReference.changed = false;
    }
  }
}