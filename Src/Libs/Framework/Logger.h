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

#include "Framework/Configuration.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Streaming/MessageQueue.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/InStreams.h"
#include <atomic>
#include <deque>
#include <stack>
#include <unordered_map>

STREAMABLE(Logger,
{
  /** Which representations will be logged for a certain thread? */
  STREAMABLE(RepresentationsPerThread,
  {,
    (std::string) thread,
    (std::vector<std::string>) representations,
  });

private:
  DECLARE_SYNC;
  OutBinaryMemory typeInfo; /**< Streamed type information created in main thread and used in logger thread. */
  OutBinaryMemory settings; /**< Streamed settings created in main thread and used in logger thread. */
  std::vector<MessageQueue> buffers; /**< All buffers to write log data to. */
  std::stack<MessageQueue*> buffersAvailable; /**< The buffers currently available to fill with log data. */
  std::deque<MessageQueue*> buffersToWrite; /**< The buffers already filled that need to be written. */
  bool bufferWasAvailable = true; /**< Was a buffer previously available? */
  std::atomic<bool> logging = false; /**< Are we currently logging? */
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
   * Update whether the logger is logging.
   * @param shouldLog Whether the logger should log now.
   * @param getDescription A function that creates a description when a new log file is started.
   */
  void update(bool shouldLog, const std::function<std::string()>& getDescription);

  /**
   * Execute the logger for this thread.
   * @param threadName The name of this thread.
   * @return Whether the current frame was logged if it potentially could have been logged.
   */
  bool execute(const std::string& threadName);

private:,
  (bool) enabled, /**< Is logging enabled? */
  (std::string) path, /**< The directory that will contain the log file. */
  (unsigned) numOfBuffers, /**< The number of buffers allocated. */
  (unsigned) sizeOfBuffer, /**< The size of each buffer in bytes. */
  (int) writePriority, /**< The scheduling priority of the writer thread. */
  (unsigned) minFreeDriveSpace, /**< Logging will stop if less MB are available to the target device. */
  (std::vector<RepresentationsPerThread>) representationsPerThread, /**< Representations to log per thread. */
});
