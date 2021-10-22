/**
 * @file Tools/Communication/BNTP.h
 *
 * Representations and functions for time synchronization inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/Settings.h"

/**
 * @struct BNTPRequest
 *
 * A struct that represents received NTP requests.
 */
struct BNTPRequest
{
  unsigned origination = 0; ///< The remote timestamp of the generation of this message.
  unsigned receipt = 0; ///< The local timestamp of the receipt of this message.
};

/**
 * @class SynchronizationMeasurementsBuffer
 *
 * A class for keeping a valid synchronization measurement.
 */
class SynchronizationMeasurementsBuffer
{
private:
  int offset = 0; ///< Offset of the time of another robot relative to the own time.
  unsigned roundTrip = 0; ///< The time the two NTP messages have been in the WLAN.
  unsigned timestamp = 0; ///< The time when this measurement has been made.

  static constexpr unsigned int clockDriftDivider = 5000; ///< The time it takes at least to accumulate a clock error of 1ms on a robot clock.
  static constexpr unsigned int maxPacketDelay = 2000; ///< The maximum time a packet can spend in the WLAN.

public:
  /**
   * Adds a new measurement to the buffer.
   * @param newOffset The measured clock offset.
   * @param newRoundTrip The associated round trip time.
   * @param receiveTimestamp The timestamp when the measurement has been completed.
   */
  void update(int newOffset, unsigned newRoundTrip, unsigned receiveTimestamp);

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

  inline unsigned getRemoteTimeInLocalTime(unsigned remoteTime) const
  {
#ifdef TARGET_ROBOT
    return remoteTime ? static_cast<unsigned>(std::max(0, static_cast<int>(remoteTime) - offset)) : 0u;
#else
    return remoteTime;
#endif
  }
};

/**
 * @class BNTP
 *
 * Implementation of the Network Time Protocol.
 */
class BNTP : public BHumanMessageParticle<MessageID::undefined>
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
  static constexpr int ntpRequestInterval = 10000; /**< Request an NTP message every $ ms. */

  unsigned lastNTPRequestSent = 0; /**< The point of time when the last NTP request has been sent to the team. */
  std::unordered_map<std::uint8_t, BNTPRequest> receivedNTPRequests; /**< The most recently received NTP request per robot. */
  SynchronizationMeasurementsBuffer timeSyncBuffers[Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1]; /**< A buffer which contains synchronization data for all other robots. */

  const FrameInfo& theFrameInfo;
  const RobotInfo& theRobotInfo;

public:
  /**
   * Constructor.
   * @param theFrameInfo Contains the current time.
   * @param theRobotInfo Contains the player number.
   */
  BNTP(const FrameInfo& theFrameInfo, const RobotInfo& theRobotInfo) : theFrameInfo(theFrameInfo), theRobotInfo(theRobotInfo) {}
};
