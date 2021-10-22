/**
 * @file Tools/Communication/BNTP.cpp
 *
 * Representations and functions for time synchronization inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BNTP.h"

void BNTP::operator>>(BHumanMessage& m) const
{
  // Respond to buffered requests.
  m.theBHumanStandardMessage.ntpMessages.clear();
  for(const auto& pair : receivedNTPRequests)
  {
    const BNTPRequest& request = pair.second;
    BNTPMessage ntpMessage;
    ntpMessage.receiver = pair.first;
    ntpMessage.requestOrigination = request.origination;
    ntpMessage.requestReceipt = request.receipt;
    m.theBHumanStandardMessage.ntpMessages.emplace_back(ntpMessage);
  }
  const_cast<std::unordered_map<std::uint8_t, BNTPRequest>&>(receivedNTPRequests).clear();

  // Send NTP requests to teammates?
  if((m.theBHumanStandardMessage.requestsNTPMessage = theFrameInfo.time - lastNTPRequestSent >= ntpRequestInterval))
    const_cast<unsigned&>(lastNTPRequestSent) = theFrameInfo.time;
}

void BNTP::operator<<(const BHumanMessage& m)
{
  const unsigned receiveTimestamp = m.timestamp;

  // Remember the teammate's request to respond to it later.
  if(m.theBHumanStandardMessage.requestsNTPMessage)
  {
    BNTPRequest& ntpRequest = receivedNTPRequests[m.theBSPLStandardMessage.playerNum];
    if(ntpRequest.origination < m.theBHumanStandardMessage.timestamp)
    {
      ntpRequest.origination = m.theBHumanStandardMessage.timestamp;
      ntpRequest.receipt = receiveTimestamp;
    }
  }

  if(m.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     m.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    return;

  // Invalidate the previous synchronization if it doesn't explain the received message.
  auto& remoteSMB = timeSyncBuffers[m.theBSPLStandardMessage.playerNum - Settings::lowestValidPlayerNumber];
  remoteSMB.validate(m.theBHumanStandardMessage.timestamp, receiveTimestamp);

  // Integrate the teammate's response (if the message contains one).
  for(const BNTPMessage& ntpMessage : m.theBHumanStandardMessage.ntpMessages)
  {
    // Only use responses for this player.
    if(ntpMessage.receiver != theRobotInfo.number)
      continue;
    // Ignore measurements that are normally impossible.
    // This catches cases in which a robot gets a response for a request but has been restarted in the meantime.
    if(receiveTimestamp < ntpMessage.requestOrigination || m.theBHumanStandardMessage.timestamp < ntpMessage.requestReceipt)
      continue;
    const unsigned totalRoundTrip = receiveTimestamp - ntpMessage.requestOrigination;
    const unsigned remoteProcessingTime = m.theBHumanStandardMessage.timestamp - ntpMessage.requestReceipt;
    if(remoteProcessingTime > totalRoundTrip)
      continue;
    const int offset = static_cast<int>(ntpMessage.requestReceipt - ntpMessage.requestOrigination + m.theBHumanStandardMessage.timestamp - receiveTimestamp) / 2;
    remoteSMB.update(offset, totalRoundTrip - remoteProcessingTime, receiveTimestamp);
  }

  // Upon receiving a message from a robot that we aren't synchronized with, force sending an NTP request at next occasion.
  if(!remoteSMB.isValid())
    lastNTPRequestSent = 0;
}

void SynchronizationMeasurementsBuffer::update(int newOffset, unsigned newRoundTrip, unsigned receiveTimestamp)
{
  const unsigned driftBound = 2 * (receiveTimestamp - timestamp) / clockDriftDivider;
  if(!timestamp ||
     newRoundTrip < roundTrip + 2 * driftBound ||
     2 * static_cast<unsigned>(std::abs(newOffset - offset)) > newRoundTrip + roundTrip + 2 * driftBound)
  {
    offset = newOffset;
    roundTrip = newRoundTrip;
    timestamp = receiveTimestamp;
  }
}

void SynchronizationMeasurementsBuffer::validate(unsigned sendTimestamp, unsigned receiveTimestamp)
{
  // Is the buffer already invalid?
  if(!timestamp)
    return;
  // Calculate an interval within which the send timestamp must have been.
  const int exactRemoteTimestamp = static_cast<int>(receiveTimestamp) + offset;
  const int errorBound = (roundTrip + 1u) / 2u + 2 * (receiveTimestamp - timestamp) / clockDriftDivider;
  unsigned lowestPossibleRemoteTimestamp = exactRemoteTimestamp - errorBound - maxPacketDelay;
  unsigned highestPossibleRemoteTimestamp = exactRemoteTimestamp + errorBound;
  if(sendTimestamp < lowestPossibleRemoteTimestamp || sendTimestamp > highestPossibleRemoteTimestamp)
    timestamp = 0;
}
