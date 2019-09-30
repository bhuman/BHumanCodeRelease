/**
 * @file Tools/BNTP.cpp
 *
 * Representations and functions for time synchronization inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BNTP.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

void BNTP::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.ntpMessages.clear();
  // Process incoming NTP requests:
  while(!receivedNTPRequests.empty())
  {
    const BNTPRequest& request = receivedNTPRequests.back();
    BNTPMessage bntpMessage;
    bntpMessage.receiver = request.sender;
    bntpMessage.requestOrigination = request.origination;
    bntpMessage.requestReceipt = request.receipt;
    m.theBHumanStandardMessage.ntpMessages.emplace_back(bntpMessage);

    const_cast<RingBuffer<BNTPRequest, MAX_NUM_OF_NTP_PACKAGES>&>(receivedNTPRequests).pop_back();
  }

  // Send NTP requests to teammates?
  if((m.theBHumanStandardMessage.requestsNTPMessage = theFrameInfo.time - lastNTPRequestSent >= NTP_REQUEST_INTERVAL))
    const_cast<unsigned&>(lastNTPRequestSent) = theFrameInfo.time;
}

void BNTP::operator<<(const BHumanMessage& m)
{
  ASSERT(m.hasBHumanParts);

  const unsigned receiveTimestamp = Time::getCurrentSystemTime();

  if(m.theBHumanStandardMessage.requestsNTPMessage)
  {
    auto oldMessage = std::find_if(receivedNTPRequests.begin(), receivedNTPRequests.end(), [&m](const BNTPRequest& request){ return request.sender == m.theBSPLStandardMessage.playerNum; });
    BNTPRequest* ntpRequest = nullptr;
    if(oldMessage == receivedNTPRequests.end())
    {
      receivedNTPRequests.push_front(BNTPRequest());
      ntpRequest = &receivedNTPRequests.front();
    }
    else if(oldMessage->origination < m.theBHumanStandardMessage.timestamp)
      ntpRequest = &(*oldMessage);
    if(ntpRequest)
    {
      ntpRequest->sender = m.theBSPLStandardMessage.playerNum;
      ntpRequest->origination = m.theBHumanStandardMessage.timestamp;
      ntpRequest->receipt = receiveTimestamp;
    }
  }

  for(auto itr = m.theBHumanStandardMessage.ntpMessages.cbegin(); itr != m.theBHumanStandardMessage.ntpMessages.cend(); ++itr)
  {
    //Check, if this response belongs to an own request:
    if(itr->receiver == theRobotInfo.number)
    {
      SynchronizationMeasurementsBuffer::SynchronizationMeasurement t;
      t.roundTrip = int(receiveTimestamp - itr->requestOrigination) - int(m.theBHumanStandardMessage.timestamp - itr->requestReceipt);
      t.offset = int(itr->requestReceipt - itr->requestOrigination + m.theBHumanStandardMessage.timestamp - receiveTimestamp) / 2;
      timeSyncBuffers[m.theBSPLStandardMessage.playerNum].add(t);
    }
  }
}

void SynchronizationMeasurementsBuffer::add(const SynchronizationMeasurement& s)
{
  buffer.push_front(s);

  bestOffset = buffer[0].offset;
  int shortestRoundTrip = buffer[0].roundTrip;
  for(size_t i = 1; i < buffer.size(); ++i)
    if(buffer[i].roundTrip < shortestRoundTrip)
    {
      shortestRoundTrip = buffer[i].roundTrip;
      bestOffset = buffer[i].offset;
    }
}
