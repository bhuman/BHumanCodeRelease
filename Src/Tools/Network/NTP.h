/**
 * @file Tools/NTP.h
 *
 * Representations anf functions for time synchronisation inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/AutoStreamable.h"

class TeammateDataProvider;

/**
 * @class SynchronizationMeasurement
 *
 * A small datatype describing a measurement of
 * the time offset between two robots
 */
struct SynchronizationMeasurement
{
  int offset = 0; ///< Offset of the time of another robot relative to the own time.
  int roundTrip = 0; ///< The time, the two NTP messages have been in the WLAN.
};

/**
 * @class NTPRequest
 *
 * A package for starting a synchronization measurement
 */
STREAMABLE(NTPRequest,
{
  unsigned char ipBasedSender; ///< The ip based id of the robot, which sent the request

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

  NTPRequest() = default;

  /**
   * @param sender The robot, which sent the request
   */
  NTPRequest(unsigned char sender) : sender(sender) {}
  ,
  (unsigned char) sender, ///< The robot, which sent the request (the only attribute streamed)
});

/**
 * @class NTPResponse
 *
 * A package for replying to an NTPRequest
 */
STREAMABLE(NTPResponse,
{
  unsigned char sender; ///< The robot, which sent the response

  /**
   * The timestamp of the generation of this message.
   * Should be set as late as possible before transmitting to WLAN
   */
  unsigned responseOrigination;

  /**
   * The timestamp of the receipt of the response by the robot which sent the request.
   * Is not set (and transmitted) by the responding robot. The receiving
   * robot sets this timestamp as early as possible after receiving from WLAN
   */
  unsigned responseReceipt;

  NTPResponse() = default;

  /**
   * @param request The request which will be responded
   */
  NTPResponse(const NTPRequest& request)
  {
    requestOrigination = request.origination;
    requestReceipt = request.receipt;
    receiver = request.sender;
  }

  /**
   * Computes time offset and round trip time from an NTPResponse
   * @return Time offset and tround trip
   */
  SynchronizationMeasurement computeSynchronizationMeasurement() const
  {
    SynchronizationMeasurement t;
    t.roundTrip = int(responseReceipt - requestOrigination) - int(responseOrigination - requestReceipt);
    t.offset = int(requestReceipt - requestOrigination + responseOrigination - responseReceipt) / 2;
    return t;
  }
  ,
  (unsigned char) receiver, ///< The robot, to which this message should be sent
  (unsigned) requestOrigination, ///< The timestamp of the generation of the request
  (unsigned) requestReceipt, ///< The timestamp of the receipt of the request
});

/**
 * @class SynchronizationMeasurementsBuffer
 *
 * A class for buffering the last synchronization measurements.
 * The currently best time offset is the element with the smallest
 * round trip time.
 */
class SynchronizationMeasurementsBuffer
{
private:
  enum { MAX_NUMBER_OF_MEASUREMENTS = 6 }; ///< Constant for internal buffer size
  RingBuffer<SynchronizationMeasurement, MAX_NUMBER_OF_MEASUREMENTS> buffer; ///< A buffer for the last measurements

public:
  int shortestRoundTrip = 0; ///< The shortest round trip time in this buffer
  int bestOffset = 0; ///< The time offset in this buffer with shortest round trip time

  /**
   * Adds a new measurement to the ring buffer
   * @param s The measurement
   */
  void add(const SynchronizationMeasurement& s)
  {
    buffer.push_front(s);

    bestOffset = buffer[0].offset;
    shortestRoundTrip = buffer[0].roundTrip;
    for(size_t i = 1; i < buffer.size(); ++i)
      if(buffer[i].roundTrip < shortestRoundTrip)
      {
        shortestRoundTrip = buffer[i].roundTrip;
        bestOffset = buffer[i].offset;
      }
  }
};

/**
 * @class NTP
 * Implementantion of the Network Time Protocol.
 */
class NTP : public MessageHandler
{
  friend class TeammateDataProvider;

  unsigned sendTimeStamp; /**< The send time stamp from the remote robot of the last handled team communication message. */
  unsigned receiveTimeStamp; /**< The receive time stamp of the last handled team communication message. */

private:
  enum { MAX_NUM_OF_NTP_CLIENTS = 255, MAX_NUM_OF_NTP_PACKAGES = 12, NTP_REQUEST_INTERVAL = 2000 }; /**< Some constants for NTP snychronization. */
  unsigned lastNTPRequestSent = 0; /**< The point of time when the last NTP request has been sent to the team. */
  int numberOfReceivedNTPRequests = 0; /**< The number of responses received in the current frame. */
  int numberOfReceivedNTPResponses = 0; /**< The number of requests received in the current frame. */
  NTPRequest receivedNTPRequests[MAX_NUM_OF_NTP_PACKAGES]; /**< The requests received in the current frame. */
  NTPResponse receivedNTPResponses[MAX_NUM_OF_NTP_PACKAGES]; /**< The responses receved in the current frame. */
  SynchronizationMeasurementsBuffer timeSyncBuffers[MAX_NUM_OF_NTP_CLIENTS]; /**< A buffer which contains synchronization data for all other robots. */
  unsigned char syncStarted[MAX_NUM_OF_NTP_CLIENTS]; /**< Synchronization was started for these robots. Only used in watchOnly mode. */
  unsigned char syncCompleted[MAX_NUM_OF_NTP_CLIENTS]; /**< Synchronization was completed for these robots. Only used in watchOnly mode. */
  unsigned timeOfLastRequest[MAX_NUM_OF_NTP_CLIENTS]; /**< Last time a packet was received from these robots. Only used in watchOnly mode. */
  int numberOfUnsyncedRobots = 0; /**< The number of robots known, but not synchronized yet. Only used in watchOnly mode. */
  unsigned timeWhenLastRobotDetected = 0; /**< The time when the last robot was detected. Only used in watchOnly mode. */

  unsigned char localId = 255; /**< The id of the local robot. */
  unsigned char currentRemoteId = 255; /**< The ip address based remote robot id of the last handled team communication message. */

public:
  NTP();

  /**
   * Function for handling all NTP stuff. Called by update().
   * @param now The current time.
   * @param out In case packages have to be sent, they will be streamed to this queue.
   * @param watchOnly Only send out NTP requests for new robots.
   * @return Send synchronization package in this frame?
   */
  bool doSynchronization(unsigned now, OutMessage& out, bool watchOnly = false);

  /**
   * Converts a time stamp from a team communciation participant to local time
   * @param remoteTime The time on the other robot
   * @param remoteId The number of that robot
   * @return The time in local time
   */
  unsigned getRemoteTimeInLocalTime(unsigned remoteTime, unsigned char remoteId) const;

  /**
   * Converts a time stamp from the team communciation participant who sent the last handled message to local time
   * @param remoteTime The time on the other robot
   * @return The time in local time
   */
  unsigned getRemoteTimeInLocalTime(unsigned remoteTime) const { return getRemoteTimeInLocalTime(remoteTime, currentRemoteId); }

  /**
   * Returns the shortest buffered round trip length of the given team communciation participant
   * @param remoteId The team communciation participant
   * @return The round trip length
   */
  unsigned getRoundTripLength(unsigned char remoteId) const;

  /**
   * Returns the shortest buffered round trip length of the team communciation participant who sent the last handled message
   * @return The round trip length
   */
  unsigned getRoundTripLength() const { return getRoundTripLength(currentRemoteId); }

  /**
   * Returns the id of last handled team communciation participant
   * @return The id
   */
  unsigned char getRemoteId() const { return currentRemoteId; }

  /**
   * The method is called for every incoming team communciation message.
   * @param message An interface to read the message from the queue.
   * @return true Was the message handled?
   */
  bool handleMessage(InMessage& message);
};
