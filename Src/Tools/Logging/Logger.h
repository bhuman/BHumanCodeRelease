/**
 * @file Logger.h
 *
 * This file declares a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Tools/Framework/Configuration.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"
#include <deque>
#include <stack>

STREAMABLE(Logger,
{
  /** Which representations will be logged for a certain thread? */
  STREAMABLE(RepresentationsPerThread,
  {,
    (std::string) thread,
    (std::vector<std::string>) representations,
  });

private:
  /** A team number and the corresponding team name. */
  STREAMABLE(Team,
  {,
    (uint8_t)(0) number,
    (std::string) name,
  });

  /** The map from team numbers to team names. */
  STREAMABLE(TeamList,
  {,
    (std::vector<Team>) teams,
  });

  DECLARE_SYNC;
  OutBinaryMemory typeInfo; /**< Streamed type information created in main thread and used in logger thread. */
  TeamList teamList; /**< The list of all teams for naming the log file after the opponent. */
  std::vector<MessageQueue> buffers; /**< All buffers to write log data to. */
  std::stack<MessageQueue*> buffersAvailable; /**< The buffers currently available to fill with log data. */
  std::deque<MessageQueue*> buffersToWrite; /**< The buffers already filled that need to be written. */
  char gameInfoThreadName[32]; /**< The thread that started logging and decides to stop it. */
  bool logging = false; /**< Are we currently logging? */
  bool hasLogged = false; /**< Have we logged before (reset when not logging and buffersToWrite is empty)? */
  std::string filename; /**< The base name of the log file. */
  Thread writerThread; /**< The thread that is writing the logged data to a file. */
  Semaphore framesToWrite; /**< How many frames the writer thread should write? */

  /** The method runs in a separate thread and writes the logged data to a file. */
  void writer();

public:
  /**
   * The constructor reads the configuration file and checks it against the module configuration.
   * @param config The initial module configuration.
   */
  Logger(const Configuration& config);

  /** The destructor signals the writer thread to stop. */
  ~Logger();

  /**
   * Execute the logger for this thread.
   * @param threadName The name of this thread.
   */
  void execute(const std::string& threadName);

private:,
  (bool) enabled, /**< Is logging enabled? */
  (std::string) path, /**< The directory that will contain the log file. */
  (unsigned) numOfBuffers, /**< The number of buffers allocated. */
  (unsigned) sizeOfBuffer, /**< The size of each buffer in bytes. */
  (int) writePriority, /**< The scheduling priority of the writer thread. */
  (unsigned) minFreeDriveSpace, /**< Logging will stop if less MB are available to the target device. */
  (std::vector<RepresentationsPerThread>) representationsPerThread, /**< Representations to log per thread. */
});
