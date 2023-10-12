/**
 * @file Tools/Communication/GameControllerRBS.cpp
 *
 * Representations and functions for time synchronization inside
 * the team (reference broadcast synchronization using GameController
 * packets).
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 * @author Arne Hasselbring
 */

#include "GameControllerRBS.h"
#include "Debugging/Annotation.h"
#include "Platform/Time.h"

void GameControllerRBS::operator>>(BHumanMessage& m) const
{
  // If any GC packet has been buffered, always use the newest one, otherwise send "nothing".
  // See below for reasons why "the newest one" may not be the best choice.
  if(gameControllerPacketBuffer.empty() ||
     gameControllerPacketBuffer.front().timestamp > m.timestamp + timestampOffset ||
     gameControllerPacketBuffer.front().timestamp + std::numeric_limits<decltype(m.referenceGameControllerPacketTimestampOffset)>::max() <= m.timestamp + timestampOffset)
  {
    m.referenceGameControllerPacketNumber = 0;
    m.referenceGameControllerPacketTimestampOffset = std::numeric_limits<decltype(m.referenceGameControllerPacketTimestampOffset)>::max();
  }
  else
  {
    m.referenceGameControllerPacketNumber = gameControllerPacketBuffer.front().number;
    m.referenceGameControllerPacketTimestampOffset = static_cast<decltype(m.referenceGameControllerPacketTimestampOffset)>(m.timestamp + timestampOffset - gameControllerPacketBuffer.front().timestamp);
  }
}

void GameControllerRBS::operator<<(const BHumanMessage& m)
{
  if(m.playerNumber < Settings::lowestValidPlayerNumber ||
     m.playerNumber > Settings::highestValidPlayerNumber)
    return;

  auto& remoteSMB = timeSyncBuffers[m.playerNumber - Settings::lowestValidPlayerNumber];

  // Invalidate the previous synchronization if it doesn't explain the received message.
  // This is necessary to avoid using an old (and very wrong) offset if the other robot
  // has been restarted in the meantime (in case they cannot be synchronized immediately
  // based on this message).
  // We use the current system time as receive time because it is surely later than the
  // actual receive time and `maxTeamMessageSendReceiveDelay` contains some buffer for this.
  // (`theFrameInfo.time` is a camera timestamp and can thus be earlier than the actual
  // receive time.)
  remoteSMB.validate(m.timestamp, Time::getCurrentSystemTime());

  // The following is based on the following considerations:
  // - If the GameController sends a packet with number $i$, each robot will either not receive it or
  //     receive it at a timestamp which only differs due to the time spent on the receiver until the
  //     timestamp is taken (but not due to delay on the sender or channel, because they are the same
  //     for all receivers).
  // - Team messages contain a pair of (packet number, receive timestamp). Since we assume that all
  //     receive timestamps of a given packet are actually taken at the same instant (+- receiver delay),
  //     the difference between the remote and local receive timestamps of the same GC packet is
  //     the clock difference.
  // - This leaves the following questions:
  //   - If a robot receives a team message which references packet number $i$, how can it be sure that,
  //       given it has a packet with number $i$ in its buffer of received GC packets, this packet is
  //       *actually* the same?
  //     - reasons why this can occur:
  //       - GameController is restarted
  //       - multiple GameControllers (?)
  //         - This isn't actually as bad, as long as both robots receive both GameControllers.
  //     - The GameController sends two packets every second. Therefore, it takes more than two minutes
  //         until the packet number wraps around. Therefore, a maximum age for buffer entries prevents
  //         this.
  //   - If a robot receives a team message which references packet number $i$, what should it do if it
  //       doesn't have a packet with number $i$ in its buffer of received GC packets?
  //     - reasons why this can occur:
  //       - no GameController running (anymore)
  //       - receiver didn't get the GC packet (got lost)
  //         - In theory, the receiver could interpolate between adjacent packets, but this would be very
  //             imprecise and introduce exactly the kind of errors that we are avoiding by synchronizing
  //             on the *same* packet. Therefore, we have to accept this case.
  //       - receiver didn't get the GC packet *yet*
  //         - On the network level, the team message cannot "overtake" the GC packet.
  //             In each frame, this function is called *after* new GameController packets have been received.
  //             This means that, in order to get a team message that references a GC packet that
  //             has not been received yet (but will be received later), the following sequence of events
  //             must occur:
  //             1. GameControlDataProvider on this robot fetches latest GC packet
  //             2. GameControlDataProvider on other robot newer GC packet
  //             3. other robot sends team message referencing the new GC packet
  //             4. TeamMessageHandler on this robot received that team message (in the same frame as 1).
  //             Because (2) happens at the beginning of a frame on the other robot and (3) at its end,
  //             the frame must be significantly shorter on the other robot than on this robot. Therefore
  //             this case is *very* unlikely to happen :fingers_crossed:

  // The sender didn't receive any GameController packet recently. In that case, we cannot
  // calculate a new offset, i.e. we either keep the old offset if there is a valid one or
  // we stay unsynchronized.
  if(m.referenceGameControllerPacketTimestampOffset == std::numeric_limits<decltype(m.referenceGameControllerPacketTimestampOffset)>::max())
    return;

  const auto lookupGameControllerPacket = [this](unsigned number)
  {
    for(const auto& packet : gameControllerPacketBuffer)
      if(packet.number == number)
        return packet.timestamp;
    return 0u;
  };

  const unsigned ownReferenceGameControllerPacketTimestamp = lookupGameControllerPacket(m.referenceGameControllerPacketNumber);
  if(!ownReferenceGameControllerPacketTimestamp)
    return;

  // This offset is *subtracted* from the remote timestamp to get the local timestamp.
  const int offset = m.timestamp + timestampOffset - m.referenceGameControllerPacketTimestampOffset - ownReferenceGameControllerPacketTimestamp;
  remoteSMB.update(offset, ownReferenceGameControllerPacketTimestamp);
}

void GameControllerRBS::update()
{
  // Remove packets that are older than some timeout. This is necessary because despite the limited size
  // of the ring buffer, packet loss may cause old packets to remain in the buffer too long.
  while(!gameControllerPacketBuffer.empty() && theFrameInfo.getTimeSince(gameControllerPacketBuffer.back().timestamp) >= gameControllerTimeout)
    gameControllerPacketBuffer.pop_back();

  // In theory we could try to detect GameController restarts here (packet number unexpectedly jumping to 0).
  // However, at the current value of \c gameControllerTimeout, it is really unlikely to (manually) restart the
  // GameController within this time - especially since the first meaningful team messages are sent after
  // ready and set.

  // Add the most recent GameController packet to the buffer if it is sufficiently new and not already in the buffer.
  if(theFrameInfo.getTimeSince(theGameControllerData.timeLastPacketReceived) < gameControllerTimeout &&
     (gameControllerPacketBuffer.empty() ||
      theGameControllerData.packetNumber != gameControllerPacketBuffer.front().number ||
      theGameControllerData.timeLastPacketReceived != gameControllerPacketBuffer.front().timestamp))
    gameControllerPacketBuffer.push_front(GameControllerPacket(theGameControllerData.packetNumber, theGameControllerData.timeLastPacketReceived));
}

void SynchronizationMeasurementsBuffer::update(int newOffset, unsigned newTimestamp)
{
  offset = newOffset;
  timestamp = newTimestamp;
}

void SynchronizationMeasurementsBuffer::validate(unsigned sendTimestamp, unsigned receiveTimestamp)
{
  // Is the buffer already invalid?
  if(!timestamp)
    return;

  // Calculate an interval within which the send timestamp must be if the currently estimated offset is valid
  // and check whether the actual send timestamp (in the sender's frame) lies within it.

  // Given the previously known offset and the receive timestamp,
  // this is the expected send timestamp (in the sender's frame) if there was neither network delay nor clock drift:
  const int expectedSendTimestamp = static_cast<int>(receiveTimestamp) + offset;

  // maxGCPacketReceiveDelay basically bounds the difference between
  // \c GameController::timeLastPacketReceived of sender and receiver. It is the inherent error bound of
  // the method, even if both clocks don't drift.

  // ((receiveTimestamp - timestamp) / clockDriftDivier + 1) is an upper bound for how much each clock may have
  // drifted from the "real" clock since the synchronization point (which is the GC packet). We disregard
  // the fact that (receiveTimestamp - timestamp) itself is measured by a robot's clock and thus may be too
  // short/long.

  // The factor 2 is because both the clock of the sender and the receiver may drift in different directions
  // (i.e. one may be slower than real time and the other faster).
  const int errorBound = maxGCPacketReceiveDelay + 2 * ((receiveTimestamp - timestamp) / clockDriftDivider + 1);
  const Range<unsigned> possibleSendTimestampRange(expectedSendTimestamp - errorBound - maxTeamMessageSendReceiveDelay, expectedSendTimestamp + errorBound);
  if(!possibleSendTimestampRange.isInside(sendTimestamp))
  {
    // By default, an offset of 0 is set, so that robots that were started at the same time
    // will still be approximately correct (if \c isValid is ignored).
    update(0, 0);

    ANNOTATION("GameControllerRBS", "Invalidated synchronization; send timestamp expected to be in [" << possibleSendTimestampRange.min << ", " << possibleSendTimestampRange.max << "] but was " << sendTimestamp << ".");
  }
}
