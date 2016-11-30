/**
* @file HammingSoundToDataConverter.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/NoWirelessSound.h"
#include "Representations/Communication/NoWirelessReturnData.h"
#include "Representations/Infrastructure/FrameInfo.h"

MODULE(HammingSoundToDataConverter,
{,
  USES(BehaviorStatus),
  REQUIRES(FrameInfo),
  REQUIRES(NoWirelessSound),
  PROVIDES(NoWirelessReturnData),
  DEFINES_PARAMETERS(
  {,
    (int)(500) timeout,
  }),
});

class HammingSoundToDataConverter : public HammingSoundToDataConverterBase
{
public:
  void update(NoWirelessReturnData& returnData);

private:
  RingBuffer<unsigned char, 16> receivedSoundsForLocation;
  RingBuffer<unsigned char, 8> receivedSounds;

  unsigned lastTimeSoundReceived = 0;
  unsigned char lastReceivedSound = -1;
  BehaviorStatus::Activity lastActivity = BehaviorStatus::noWifi;

  void reset();
  void handleLocation(NoWirelessReturnData& returnData);
  void handleData(NoWirelessReturnData& returnData);

  bool decode(std::array<unsigned char, 8>& hamming, unsigned char& value) const;
};
