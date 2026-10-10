/**
 * @file KickLengthConverter.h
 * This file contains functions to convert the kick length into the kick power
 *
 * @author Philip Reichenberg
 */
#include "Tools/Motion/KickLengthConverter.h"

namespace KickLengthConverter
{
  float interpolateKickStrength(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const std::vector<KickLengthPair::Pair>& kickPair)
  {
    const float useLength = theKickInfo[kickType].range.limit(length);
    const float lengthWidth = theKickInfo[kickType].range.getSize();
    for(std::size_t index = 1; index < kickPair.size(); index++)
    {
      if(lengthWidth * kickPair[index].rangeIncrease + theKickInfo[kickType].range.min > useLength)
      {
        const float minIndexLength = lengthWidth * kickPair[index - 1].rangeIncrease + theKickInfo[kickType].range.min;
        const float maxIndexLength = lengthWidth * kickPair[index].rangeIncrease + theKickInfo[kickType].range.min;
        const float ratio = (useLength - minIndexLength) / (maxIndexLength - minIndexLength);
        return kickPair[index - 1].powerIncrease * (1.f - ratio) + kickPair[index].powerIncrease * ratio;
      }
    }
    return 1.f;
  }

  float kickLengthToPower(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const KickLengthPair& theKickLengthPair)
  {
    Rangef useKickRange = theKickInfo[kickType].range;
    switch(kickType)
    {
      case KickInfo::rlKickDynamicOld:
        return interpolateKickStrength(kickType, length, theKickInfo, theKickLengthPair.kickLengthDynamicOldPair);
      default:
        return std::min(1.f, std::max(0.f, length - useKickRange.min) / (useKickRange.max - useKickRange.min));
    }
  }
}
