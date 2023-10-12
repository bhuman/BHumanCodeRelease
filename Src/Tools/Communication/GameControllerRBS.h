/**
 * @file Tools/Communication/GameControllerRBS.h
 *
 * Representations and functions for time synchronization inside
 * the team (reference broadcast synchronization using GameController
 * packets).
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Communication/GameControllerData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Communication/BHumanMessageParticle.h"
#include "MathBase/RingBuffer.h"
#include "Framework/Settings.h"

/**
 * @class SynchronizationMeasurementsBuffer
 *
 * A class for keeping a valid synchronization measurement.
 */
class SynchronizationMeasurementsBuffer
{
private:
  int offset = 0; ///< Offset of the time of another robot relative to the own time.
  unsigned timestamp = 0; ///< The time when this measurement has been made (used to calculate how much the clock may have drifted at a later point).

  static constexpr unsigned int clockDriftDivider = 5000; ///< The time it takes at least to accumulate a clock error of 1ms on a robot clock (with respect to "real" time). NOT EMPIRICALLY VALIDATED YET!
  static constexpr unsigned int maxTeamMessageSendReceiveDelay = 2000; ///< The maximum time that can pass between the send timestamp of a team message is taken at the sender and the receive timestamp is taken at the receiver. NOT EMPIRICALLY VALIDATED YET!
  static constexpr unsigned int maxGCPacketReceiveDelay = 5; ///< The maximum time that can pass after the arrival of a GameController packet at the NIC until its timestamp (\c GameControllerData::timeLastPacketReceived) is taken.

public:
  /**
   * Adds a new measurement to the buffer.
   * @param newOffset The measured clock offset.
   * @param newTimestamp The timestamp when the measurement has been completed.
   */
  void update(int newOffset, unsigned newTimestamp);

  /**
   * Checks whether a message is compatible with the current synchronization data and otherwise invalidates the buffer.
   * @param sendTimestamp The (remote) time when the message has been sent.
   * @param receiveTimestamp The (local) time when the message has been received.
   */
  void validate(unsigned sendTimestamp, unsigned receiveTimestamp);

  /**
   * Checks whether the time offset is valid.
   * @return Whether ...
   */
  bool isValid() const
  {
#ifdef TARGET_ROBOT
    return timestamp != 0;
#else
    return true;
#endif
  }

  unsigned getRemoteTimeInLocalTime(unsigned remoteTime) const
  {
#ifdef TARGET_ROBOT
    return remoteTime ? static_cast<unsigned>(std::max(0, static_cast<int>(remoteTime) - offset)) : 0u;
#else
    return remoteTime;
#endif
  }
};

/**
 * @class GameControllerRBS
 *
 * Implementation of Reference Broadcast Synchronization using GameController packets.
 */
class GameControllerRBS : public BHumanMessageParticle
{
public:
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

  const SynchronizationMeasurementsBuffer* operator[](unsigned number) const
  {
    return number >= Settings::lowestValidPlayerNumber && number <= Settings::highestValidPlayerNumber ?
           &timeSyncBuffers[number - Settings::lowestValidPlayerNumber] : nullptr;
  }

private:
  struct GameControllerPacket
  {
    GameControllerPacket(std::uint8_t number, unsigned timestamp) : number(number), timestamp(timestamp) {}
    std::uint8_t number; /**< The number of the packet. */
    unsigned timestamp; /**< The receive timestamp of the packet. */
  };

  SynchronizationMeasurementsBuffer timeSyncBuffers[Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1]; /**< A buffer which contains synchronization data for all other robots. */

  /*
   * Buffer of the last received GameController packets.
   * If the buffer is too small (like 2 entries), it becomes more likely that the sender references
   * a packet that is too old to be buffered, especially if the sender lost the most recent packet and
   * there is some network delay.
   * 10 seems to be very generous.
   */
  RingBuffer<GameControllerPacket, 10> gameControllerPacketBuffer;

  /**
   * Older GameController packets are removed from \c gameControllerPacketBuffer. 5s roughly correspond
   * to 10 entries if all packets are received. It does not make sense to set this parameter smaller
   * than 500 times the ring buffer size (or make the ring buffer larger than 1/500 than this).
   */
  static constexpr int gameControllerTimeout = 5000;

  /**
   * This should be the upper bound (in milliseconds) of how much later a GameController packet can be
   * received than the team message's timestamp when it is still included in that team message. The
   * receive timestamp of a GameController packet is actually encoded relative to \c (timestamp + timestampOffset).
   */
  static constexpr unsigned timestampOffset = 200;

  const FrameInfo& theFrameInfo;
  const GameControllerData& theGameControllerData;

public:
  /**
   * Constructor.
   * @param theFrameInfo Contains the current time.
   * @param theGameControllerData Contains the packet number and receive timestamp.
   */
  GameControllerRBS(const FrameInfo& theFrameInfo, const GameControllerData& theGameControllerData) : theFrameInfo(theFrameInfo), theGameControllerData(theGameControllerData) {}

  /** Update the GameController packet buffer.  */
  void update();
};
