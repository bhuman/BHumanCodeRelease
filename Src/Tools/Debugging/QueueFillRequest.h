/**
 * @file QueueFillRequest.h
 * Declaration of class QueueFillRequest.
 *
 * @author <a href=mailto:dueffert@informatik.hu-berlin.de>Uwe Düffert</a>
 * @author <a href=mailto:oberlies@sim.tu-darmstadt.de>Tobias Oberlies</a>
 */

#pragma once

#include "Tools/Streams/InOut.h"

/**
 * @class QueueFillRequest
 *
 * Settings for the outgoing queue in the Debug process.
 */
class QueueFillRequest
{
public:
  /**
   * Options for the actions and timing.
   */
  enum Behavior
  {
    sendImmediately = 0,  // Send messages as fast as possible
    sendAfter       = 1,  // Send messages after n milliseconds (and then discard new messages)
    sendEvery       = 2,  // Send messages every n milliseconds
    collect         = 4,  // Collect messages
    sendCollected   = 5,  // Send collected messages (and discard new messages)
    discardNew      = 6,  // Keep collected messages, but discard new ones
    discardAll      = 7   // Discard all messages
  };

  /**
   * Options for the filter applied to messages before they are sent/stored.
   */
  enum Filter
  {
    sendEverything = 0,  // Send all messages, no matter how long it takes
    latestOnly     = 1   // Only send latest message(s) per message type (more precisely, apply removeRepetitions)
  };

  /**
   * Options for the data sink of the queue.
   */
  enum Target
  {
    sendViaNetwork  = 0,  // Actually send messages via the network
    writeToStick    = 1   // Don't send messages but write them onto the stick
  };

public: // members

  /** Queue fill and send timing behaviour. */
  Behavior behavior = sendImmediately;

  /** Filter applied before sending/storing messages. */
  Filter filter = latestOnly;

  /** Target of the messages (network or stick). */
  Target target = sendViaNetwork;

  /** Timing parameter (used by some of the behaviours). */
  int timingMilliseconds = 0;

public:
  QueueFillRequest() = default;
  QueueFillRequest(Behavior behavior, Filter filter, Target target, int timingMilliseconds);
};

/**
 * Streaming operator that reads a QueueFillRequest from a stream. Note that for size compliance
 * with previous verions, the enums are stored as unsinged chars (with one extra char for padding).
 *
 * @param stream The stream from which is read.
 * @param queueFillRequest The QueueFillRequest taking the values.
 * @return The original stream.
 */
In& operator>>(In& stream, QueueFillRequest& queueFillRequest);

/**
 * Streaming operator that writes a QueueFillRequest to a stream. Note that for size compliance
 * with previous verions, the enums are stored as unsinged chars (with one extra char for padding).
 *
 * @param stream The stream to write on.
 * @param queueFillRequest The QueueFillRequest providing the values.
 * @return The original stream.
 */
Out& operator<<(Out& stream, const QueueFillRequest& queueFillRequest);
