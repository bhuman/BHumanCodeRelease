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

class TeamDataProvider;

/**
* @class SynchronizationMeasurement
*
* A small datatype describing a measurement of
* the time offset between two robots
*/
class SynchronizationMeasurement
{
public:
  /** Constructor */
  SynchronizationMeasurement() : offset(0), roundTrip(0) {}

  /** Offset of the time of another robot relative to the own time*/
  int offset;
  /** The time, the two NTP messages have been in the WLAN*/
  int roundTrip;
};


/**
* @class NTPRequest
*
* A package for starting a synchronization measurement
*/
STREAMABLE(NTPRequest,
{
public:
  /** The ip based id of the robot, which sent the request*/
  unsigned char ipBasedSender;
  /** The timestamp of the generation of this message.
   *  Should be set as late as possible before transmitting to WLAN*/
  unsigned origination;
  /** The timestamp of the receipt of a request by another robot.
   *  Is not set (and transmitted) by the requesting robot. The receiving
   *  robot sets this timestamp as early as possible after receiving from WLAN*/
  unsigned receipt;

  /** Constructor
  * @param sender The robot, which sent the request
  */
  NTPRequest(unsigned char sender) : sender(sender) {},

  /** The robot, which sent the request (the only attribute streamed) */
  (unsigned char) sender,
});

/**
* @class NTPResponse
*
* A package for replying to an NTPRequest
*/
STREAMABLE(NTPResponse,
{
public:
  /** The robot, which sent the response*/
  unsigned char sender;
  /** The timestamp of the generation of this message.
   *  Should be set as late as possible before transmitting to WLAN*/
  unsigned responseOrigination;
  /** The timestamp of the receipt of the response by the robot which sent the request.
   *  Is not set (and transmitted) by the responding robot. The receiving
   *  robot sets this timestamp as early as possible after receiving from WLAN*/
  unsigned responseReceipt;

  /** Constructor
  * @param request The request which will be responded
  */
  NTPResponse(const NTPRequest& request)
  {
    requestOrigination = request.origination;
    requestReceipt = request.receipt;
    receiver = request.sender;
  }

  /** Computes time offset and round trip time from an NTPResponse
  * @return Time offset and tround trip
  */
  SynchronizationMeasurement computeSynchronizationMeasurement() const
  {
    SynchronizationMeasurement t;
    t.roundTrip = int(responseReceipt - requestOrigination) -
                  int(responseOrigination - requestReceipt);
    t.offset = int(requestReceipt - requestOrigination +
                   responseOrigination - responseReceipt) / 2;
    return t;
  },

  /** The robot, to which this message should be sent*/
  (unsigned char) receiver,
  /** The timestamp of the generation of the request*/
  (unsigned) requestOrigination,
  /** The timestamp of the receipt of the request*/
  (unsigned) requestReceipt,
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
  /** Constant for internal buffer size*/
  enum {MAX_NUMBER_OF_MEASUREMENTS = 6};
  /** A buffer for the last measurements*/
  RingBuffer<SynchronizationMeasurement, MAX_NUMBER_OF_MEASUREMENTS> buffer;

public:
  int shortestRoundTrip; /**< The shortest round trip time in this buffer */
  int bestOffset; /**< The time offset in this buffer with shortest round trip time */

  /** Constructor */
  SynchronizationMeasurementsBuffer() : shortestRoundTrip(0), bestOffset(0) {}

  /** Adds a new measurement to the ring buffer
  * @param s The measurement
  */
  inline void add(const SynchronizationMeasurement& s)
  {
    buffer.add(s);

    bestOffset = buffer[0].offset;
    shortestRoundTrip = buffer[0].roundTrip;
    for(int i = 1; i < buffer.getNumberOfEntries(); ++i)
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
  friend class TeamDataProvider;
private:
  enum {MAX_NUM_OF_NTP_CLIENTS = 255, MAX_NUM_OF_NTP_PACKAGES = 12, NTP_REQUEST_INTERVAL = 2000};  /**< Some constants for NTP snychronization*/
  unsigned lastNTPRequestSent; /**< The point of time when the last NTP request has been sent to the team*/
  int numberOfReceivedNTPRequests; /**< The number of responses received in the current frame*/
  int numberOfReceivedNTPResponses; /**< The number of requests received in the current frame*/
  NTPRequest receivedNTPRequests[MAX_NUM_OF_NTP_PACKAGES]; /**< The requests received in the current frame*/
  NTPResponse receivedNTPResponses[MAX_NUM_OF_NTP_PACKAGES]; /**< The responses receved in the current frame*/
  SynchronizationMeasurementsBuffer timeSyncBuffers[MAX_NUM_OF_NTP_CLIENTS]; /**< A buffer which contains synchronization data for all other robots*/

  unsigned char localId; /**< The id of the local robot. */
  unsigned char currentRemoteId; /**< The ip address based remote robot id of the last handled team communication message. */

public:
  unsigned int sendTimeStamp; /**< The send time stamp from the remote robot of the last handled team communication message. */
  unsigned int receiveTimeStamp; /**< The receive time stamp of the last handled team communication message. */

  /**
  * Default constructor.
  */
  NTP();

  /**
  * Function for handling all NTP stuff. Called by update().
  * @param now The current time.
  * @param out In case packages have to be sent, they will be streamed to this queue.
  * @return Send synchronization package in this frame?
  */
  bool doSynchronization(unsigned now, OutMessage& out);

  /** Converts a time stamp from a team communciation participant to local time
  * @param remoteTime The time on the other robot
  * @param remoteId The number of that robot
  * @return The time in local time
  */
  unsigned getRemoteTimeInLocalTime(unsigned remoteTime, unsigned char remoteId) const;

  /** Converts a time stamp from the team communciation participant who sent the last handled message to local time
  * @param remoteTime The time on the other robot
  * @return The time in local time
  */
  inline unsigned getRemoteTimeInLocalTime(unsigned remoteTime) const {return getRemoteTimeInLocalTime(remoteTime, currentRemoteId);}

  /** Returns the shortest buffered round trip length of the given team communciation participant
  * @param remoteId The team communciation participant
  * @return The round trip length
  */
  unsigned getRoundTripLength(unsigned char remoteId) const;

  /** Returns the shortest buffered round trip length of the team communciation participant who sent the last handled message
  * @return The round trip length
  */
  inline unsigned getRoundTripLength() const {return getRoundTripLength(currentRemoteId);}

  /** Returns the id of last handled team communciation participant
  * @return The id
  */
  inline unsigned char getRemoteId() const {return currentRemoteId;}

  /**
  * The method is called for every incoming team communciation message.
  * @param message An interface to read the message from the queue.
  * @return true Was the message handled?
  */
  bool handleMessage(InMessage& message);
};
