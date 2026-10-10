/**
 * @file KickLengthConverter.h
 * This file contains functions to convert the kick length into the kick power
 *
 * @author Philip Reichenberg
 */
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/KickLengthPair.h"

namespace KickLengthConverter
{
  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float interpolateKickStrength(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const std::vector<KickLengthPair::Pair>& kickPair);

  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float kickLengthToPower(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const KickLengthPair& theKickLengthPair);
}
