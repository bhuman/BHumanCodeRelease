/*
 * @file Logger.h
 * The file declares a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * Logfile format:
 * magic byte | size of next compressed block | compressed block | size | compressed block | etc...
 * Each block is compressed using libsnappy
 *
 * Block format (after decompression):
 * | block length | number of messages | Frame | Frame | Frame | ... | Frame |
 *
 * Each frame looks like this:
 * | ProcessBegin | Log data 1 | Log data 2 | ... | Log data n | ProcessFinished |
 * Log data format:
 * | ID ( 1 byte) | Message size (3 byte) | Message |
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#pragma once

#include "Blackboard.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Settings.h"
#include "Tools/Cabsl.h"

class Logger : public Cabsl<Logger>
{
private:
  STREAMABLE(Parameters,
  {,
    (bool)(false) enabled, /**< Determines whether the logger is enabled or disabled. */
    (std::string) logFilePath, /**< Where to write the log file. */
    (int) maxBufferSize, /**< Max size of the buffer in bytes. */
    (int) blockSize, /**< Size per frame in bytes. */
    (std::vector<std::string>) representations, /**< Contains the representations that should be logged. */
    (int) writePriority,
    (bool) debugStatistics,
    (unsigned) minFreeSpace, /**< Minimum free space left on the device in MB. */
  });

  class Loggable
  {
  public:
    Streamable* representation;
    MessageID id;

    Loggable() = default;
    Loggable(Streamable* representation, MessageID id) : representation(representation), id(id) {}
  };

  Parameters parameters;
  int blackboardVersion; /**< The blackboard version the logger is currently configured for. */
  std::vector<Loggable> loggables; /**< The representations that should be logged. */
  std::vector<MessageQueue*> buffer; /**< Ring buffer of message queues. Shared with the writer thread. */
  std::string logFilename; /**< Path and name of the log file. Set in initial state. */
  volatile int readIndex; /**< The first index of the buffer that should be read by the writer thread. */
  volatile int writeIndex; /**< Index of the buffer that is currently used for writing. */
  int frameCounter; /**< Number of frames that are already in the current message queue. */
  Thread<Logger> writerThread;/**< Used to write the buffer to disk in the background */
  Semaphore framesToWrite; /**< How many frames the writer thread should write? */
  volatile bool writerIdle; /**< Is true if the writer thread has nothing to do. */
  volatile unsigned writerIdleStart; /**< The system time at which the writer thread went idle. */

  /**
   * Generate a filename containing the robot's player number, its name, and the current
   * date and time.
   * @return A log file name that starts with the path defined in the parameters.
   */
  std::string generateFilename() const;

  /** Write all loggable representations to a buffer. */
  void logFrame();

  /** Write contents of buffers to disk in the background. */
  void writeThread();

  /** Minimal behavior to handle logging. */
  option(Root)
  {
    ASSERT(Blackboard::getInstance().exists("GameInfo"));
    const GameInfo& gameInfo = (const GameInfo&) Blackboard::getInstance()["GameInfo"];

    initial_state(initial)
    {
      transition
      {
        goto start;
      }
    }

    state(start)
    {
      transition
      {
        goto idle;
      }
      action
      {
        writerThread.start(this, &Logger::writeThread);
      }
    }

    state(idle)
    {
      transition
      {
        if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
          goto running;
      }
    }

    state(running)
    {
      transition
      {
        if(gameInfo.state == STATE_INITIAL || gameInfo.state == STATE_FINISHED)
          goto delayPlaySound;
        else if(SystemCall::getFreeDiskSpace(logFilename.c_str()) >> 20 < parameters.minFreeSpace)
          goto error;
      }
      action
      {
        logFrame();
      }
    }

    state(delayPlaySound)
    {
      transition
      {
        if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
          goto running;
        else if(writerIdle && SystemCall::getTimeSince(writerIdleStart) > 5000 && state_time > 500)
          goto playSound;
      }
    }

    state(playSound)
    {
      transition
      {
        goto idle;
      }
      action
      {
        SystemCall::playSound("logWritten.wav");
      }
    }

    state(error)
    {
      transition
      {
        goto aborted;
      }
      action
      {
        writerThread.announceStop();
        OUTPUT_WARNING("Logger: Disk full, logging aborted.");
      }
    }

    aborted_state(aborted) {}
  }

public:
  Logger();
  ~Logger();

  /** Has to be called in each cycle. */
  void execute();
};
