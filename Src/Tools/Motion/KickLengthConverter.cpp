/**
 * @file KickLengthConverter.h
 * This file contains functions to convert the kick length into the kick power
 *
 * @author Philip Reichenberg
 */
#include "Tools/Motion/KickLengthConverter.h"

namespace KickLengthConverter
{
  std::vector<KickLengthPair> getKickLengthPassPairList()
  {
    return { { 0.f, 0.15f }, { 0.08f, 0.15f }, { 0.6f, 0.25f }, { 0.95f, 0.8f }, { 1.f, 1.f } };
  }

  std::vector<KickLengthPair> getKickLengthPairList()
  {
    return { { 0.f, 0.2f }, { 0.5f, 0.5f }, { 1.f, 1.f } };
  }

  float kickLengthToPower(const KickInfo::KickType kickType, const float length, const Angle direction, const KickInfo& theKickInfo)
  {
    if(kickType == KickInfo::forwardFastRight || kickType == KickInfo::forwardFastLeft)
    {
      const float useLength = theKickInfo[kickType].range.limit(length);
      const float lengthWidth = theKickInfo[kickType].range.getSize();
      for(std::size_t index = 1; index < kickLengthPair.size(); index++)
      {
        if(lengthWidth * kickLengthPair[index].rangeIncrease + theKickInfo[kickType].range.min > useLength)
        {
          const float minIndexLength = lengthWidth * kickLengthPair[index - 1].rangeIncrease + theKickInfo[kickType].range.min;
          const float maxIndexLength = lengthWidth * kickLengthPair[index].rangeIncrease + theKickInfo[kickType].range.min;
          const float ratio = (useLength - minIndexLength) / (maxIndexLength - minIndexLength);
          return kickLengthPair[index - 1].powerIncrease * (1.f - ratio) + kickLengthPair[index].powerIncrease * ratio;
        }
      }
      return 1.f;
    }
    if(kickType == KickInfo::forwardFastRightPass || kickType == KickInfo::forwardFastLeftPass)
    {
      const float useLength = theKickInfo[kickType].range.limit(length);
      const float lengthWidth = theKickInfo[kickType].range.getSize();
      for(std::size_t index = 1; index < kickLengthPassPair.size(); index++)
      {
        if(lengthWidth * kickLengthPassPair[index].rangeIncrease + theKickInfo[kickType].range.min > useLength)
        {
          const float minIndexLength = lengthWidth * kickLengthPassPair[index - 1].rangeIncrease + theKickInfo[kickType].range.min;
          const float maxIndexLength = lengthWidth * kickLengthPassPair[index].rangeIncrease + theKickInfo[kickType].range.min;
          const float ratio = (useLength - minIndexLength) / (maxIndexLength - minIndexLength);
          return kickLengthPassPair[index - 1].powerIncrease * (1.f - ratio) + kickLengthPassPair[index].powerIncrease * ratio;
        }
      }
      return 1.f;
    }
    else
    {
      Rangef useKickRange = theKickInfo[kickType].range;
      if(kickType == KickInfo::walkForwardsLeft || kickType == KickInfo::walkForwardsRight)
      {
        const bool isLeft = kickType == KickInfo::KickType::walkForwardsLeft;
        const Angle useMaxKickAngle = -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset;
        const Rangea angleClip(!isLeft ? 0_deg : useMaxKickAngle, !isLeft ? useMaxKickAngle : 0_deg);
        const float interpolation = Rangef::ZeroOneRange().limit(angleClip.limit(direction) / -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset);

        useKickRange.min = (1 - interpolation) * theKickInfo[kickType].range.min + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.min;
        useKickRange.max = (1 - interpolation) * theKickInfo[kickType].range.max + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.max;
      }
      return std::min(1.f, std::max(0.f, length - useKickRange.min) / (useKickRange.max - useKickRange.min));
    }
  }
}
