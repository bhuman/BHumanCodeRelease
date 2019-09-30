/**
 * @file Tools/BNTP.h
 *
 * Representations and functions for time synchronization inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"

/**
 * @class BNTPRequest
 *
 * A packet for starting a synchronization measurement
 */
STREAMABLE(BNTPRequest,
{
  /**
   * The timestamp of the generation of this message.
   * Should be set as late as possible before transmitting to WLAN
   */
  unsigned origination;

  /**
   * The timestamp of the receipt of a request by another robot.
   * Is not set (and transmitted) by the requesting robot. The receiving
   * robot sets this timestamp as early as possible after receiving from WLAN
   */
  unsigned receipt;

  BNTPRequest() = default;

  /**
   * @param sender The robot, which sent the request
   */
  BNTPRequest(unsigned char sender) : sender(sender) {},

  (unsigned char) sender, ///< The robot, which sent the request (the only attribute streamed)
});

/**
 * @class SynchronizationMeasurementsBuffer
 *
 * A class for buffering the last synchronization measurements.
 * The currently best time offset is the element with the smallest
 * round trip time.
 */
class BNTP;
class SynchronizationMeasurementsBuffer
{
public:
  /**
   * @class SynchronizationMeasurement
   *
   * A small datatype describing a measurement of
   * the time offset between two robots
   */
  struct SynchronizationMeasurement
  {
    int offset = 0;///< Offset of the time of another robot relative to the own time.
    int roundTrip = 0;///< The time, the two NTP messages have been in the WLAN.
  };

private:
  enum { MAX_NUMBER_OF_MEASUREMENTS = 6 }; ///< Constant for internal buffer size
  RingBuffer<SynchronizationMeasurement, MAX_NUMBER_OF_MEASUREMENTS> buffer; ///< A buffer for the last measurements

  int bestOffset = 0; ///< The time offset in this buffer with shortest round trip time

  friend BNTP;
  /**
   * Adds a new measurement to the ring buffer
   * @param s The measurement
   */
  void add(const SynchronizationMeasurement& s);

public:
  inline unsigned getRemoteTimeInLocalTime(unsigned remoteTime) const
  {
    return static_cast<unsigned>(std::max(0, static_cast<int>(remoteTime) - bestOffset));
  }
};

/**
 * @class NTP
 * Implementantion of the Network Time Protocol.
 */
class BNTP : public BHumanMessageParticle<MessageID::undefined>
{
public:
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

  const SynchronizationMeasurementsBuffer* operator[](unsigned number) const
  {
    return number <= MAX_NUM_OF_NTP_CLIENTS ? &timeSyncBuffers[number] : 0;
  }

private:
  enum { MAX_NUM_OF_NTP_CLIENTS = 12, MAX_NUM_OF_NTP_PACKAGES = 12, NTP_REQUEST_INTERVAL = 2000 }; /**< Some constants for NTP synchronisation. */

  unsigned lastNTPRequestSent = 0; /**< The point of time when the last NTP request has been sent to the team. */
  RingBuffer<BNTPRequest, MAX_NUM_OF_NTP_PACKAGES> receivedNTPRequests; /**< The requests received in the current frame. */

  SynchronizationMeasurementsBuffer timeSyncBuffers[MAX_NUM_OF_NTP_CLIENTS]; /**< A buffer which contains synchronization data for all other robots. */

  const FrameInfo& theFrameInfo;
  const RobotInfo& theRobotInfo;

public:
  /**
   * Constructor.
   * @param localId The number this robot is identified with.
   */
  BNTP(const FrameInfo& theFrameInfo, const RobotInfo& theRobotInfo) : theFrameInfo(theFrameInfo), theRobotInfo(theRobotInfo) {}
};
