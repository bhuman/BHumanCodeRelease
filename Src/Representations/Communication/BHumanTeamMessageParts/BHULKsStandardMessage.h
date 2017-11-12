/**
 * @file BHULKsStandardMessage.h
 *
 * The file integrate the BHULKsStandardMessage into the B-Human system.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "Tools/Communication/BHULKsStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/Enum.h"

#pragma once

struct BHULKsStandardMessage : public B_HULKs::BHULKsStandardMessage, public Streamable
{
  static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "Please adjust this file to the newer version.");

protected:
  void serialize(In* in, Out* out);
};

namespace B_HULKs
{
  Out& operator<<(Out& stream, const Obstacle& bHSTMObstacle);
  In& operator >> (In& stream, Obstacle& bHSTMObstacle);

  Out& operator<<(Out& stream, const BNTPMessage& ntpMessage);
  In& operator >> (In& stream, BNTPMessage& ntpMessage);

  Out& operator<<(Out& stream, const OwnTeamInfo& ownTeamInfo);
  In& operator >> (In& stream, OwnTeamInfo& ownTeamInfo);

  inline static const char* getName(Role e);
  inline static const char* getName(HearingConfidence e);
  inline static const char* getName(ObstacleType e);
}
