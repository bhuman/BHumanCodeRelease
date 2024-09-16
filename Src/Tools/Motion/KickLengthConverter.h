/**
 * @file KickLengthConverter.h
 * This file contains functions to convert the kick length into the kick power
 *
 * @author Philip Reichenberg
 */
#include "Representations/Configuration/KickInfo.h"

namespace KickLengthConverter
{
  struct KickLengthPair
  {
    const float rangeIncrease; /**< The % range relative to the max range. */
    const float powerIncrease; /**< The power value. */
  };

  /**
   * Interpolation information for kicks
   */
  std::vector<KickLengthPair> getKickLengthPairList();
  /**
   * Interpolation information for kicks
   */
  std::vector<KickLengthPair> getKickLengthPassPairList();

  /**
   * Interpolation information for kicks
   */
  const std::vector<KickLengthPair> kickLengthPassPair = getKickLengthPassPairList();

  /**
   * Interpolation information for kicks
   */
  const std::vector<KickLengthPair> kickLengthPair = getKickLengthPairList();

  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float kickLengthToPower(const KickInfo::KickType kickType, const float length, const Angle direction, const KickInfo& theKickInfo);
}
