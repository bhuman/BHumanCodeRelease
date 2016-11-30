/**
* @file HammingSoundToDataConverter.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "HammingSoundToDataConverter.h"
#include "Tools/HammingEncodingBase4.h"
#include "Tools/Math/Random.h"

MAKE_MODULE(HammingSoundToDataConverter, communication)

void HammingSoundToDataConverter::reset()
{
  lastTimeSoundReceived = 0;
  receivedSounds.clear();
  receivedSoundsForLocation.clear();
  lastActivity = BehaviorStatus::noWifi;
}

void HammingSoundToDataConverter::update(NoWirelessReturnData& returnData)
{
  if(theBehaviorStatus.activity != lastActivity)
    reset();

  if(theFrameInfo.getTimeSince(lastTimeSoundReceived) > timeout)
  {
    reset();
    lastTimeSoundReceived = -1;
  }

  if(theBehaviorStatus.activity != BehaviorStatus::noWifiData)
    returnData.data.clear();

  lastActivity = theBehaviorStatus.activity;

  if(theBehaviorStatus.activity == BehaviorStatus::noWifiLocation)
    handleLocation(returnData);
  else if(theBehaviorStatus.activity == BehaviorStatus::noWifiData)
    handleData(returnData);
}


void HammingSoundToDataConverter::handleLocation(NoWirelessReturnData& returnData)
{
  if(theNoWirelessSound.lastTimeSoundDetected == theFrameInfo.time
     && lastReceivedSound != theNoWirelessSound.soundNumber)
  {
    lastReceivedSound = static_cast<unsigned char>(theNoWirelessSound.soundNumber);
    receivedSoundsForLocation.push_front(lastReceivedSound / 2);
    lastTimeSoundReceived = theFrameInfo.time;

    OUTPUT_TEXT("Detected Sound:" << theNoWirelessSound.soundNumber << " (" << (lastReceivedSound / 2) << ")");
  }

  if(lastTimeSoundReceived == theFrameInfo.time && receivedSoundsForLocation.full())
  {
    std::array<unsigned char, 8> hammingX;
    for(int i = 0; i <= 7; ++i)
      hammingX[i] = receivedSoundsForLocation[receivedSoundsForLocation.capacity() - 1 - i];

    std::array<unsigned char, 8> hammingY;
    for(int i = 8; i <= 15; ++i)
      hammingY[i - 8] = receivedSoundsForLocation[receivedSoundsForLocation.capacity() - 1 - i];

    unsigned char x, y;
    const bool xSuccess = decode(hammingX, x);
    const bool ySuccess = decode(hammingY, y);

    returnData.positionToPointAt.x() = static_cast<float>(x) * 9000.f / 256.f - 4500.f;
    returnData.positionToPointAt.y() = static_cast<float>(y) * 6000.f / 256.f - 3000.f;

    if(xSuccess)
      OUTPUT_TEXT("Received X: " << returnData.positionToPointAt.x());
    else
      OUTPUT_TEXT("Error receiving X: " << returnData.positionToPointAt.x());

    if(ySuccess)
      OUTPUT_TEXT("Received Y: " << returnData.positionToPointAt.y());
    else
      OUTPUT_TEXT("Error receiving Y: " << returnData.positionToPointAt.y());

    if(xSuccess && ySuccess)
    {
      returnData.timeOfLastLocationData = theFrameInfo.time;
      reset();
    }
  }
}


void HammingSoundToDataConverter::handleData(NoWirelessReturnData& returnData)
{
  if(theNoWirelessSound.lastTimeSoundDetected == theFrameInfo.time
     && lastReceivedSound != theNoWirelessSound.soundNumber)
  {
    lastReceivedSound = static_cast<unsigned char>(theNoWirelessSound.soundNumber);
    receivedSounds.push_front(lastReceivedSound / 2);
    lastTimeSoundReceived = theFrameInfo.time;

    OUTPUT_TEXT("Detected Sound:" << theNoWirelessSound.soundNumber << " (" << (lastReceivedSound / 2) << ")");
  }

  if(lastTimeSoundReceived == theFrameInfo.time && receivedSounds.full())
  {
    std::array<unsigned char, 8> hamming;
    for(int i = 0; i <= 7; ++i)
      hamming[i] = receivedSounds[receivedSounds.capacity() - 1 - i];

    unsigned char value;
    if(!decode(hamming, value))
      OUTPUT_WARNING("Received invalid hamming code.");
    returnData.data.push_back(value);
    reset();
  }
}

bool HammingSoundToDataConverter::decode(std::array<unsigned char, 8>& hamming, unsigned char& value) const
{
  const bool success = HammingEncodingBase4::correctHamming84(hamming);
  std::array<unsigned char, 4> quaternary;
  HammingEncodingBase4::decodeHamming84(hamming, quaternary);
  value = HammingEncodingBase4::fromBase4Inverse(quaternary);
  OUTPUT_TEXT("Received Value: " << value);
  return success;
}