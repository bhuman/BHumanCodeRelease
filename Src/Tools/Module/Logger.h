/*
 * @file Logger.h
 * The file declares a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * Logfile format:
 * logFileMessageIDs | number of message ids | streamed message id names |
 * logFileStreamSpecification | streamed StreamHandler |
 * idLogFileCompressed | size of next compressed block | compressed block | size | compressed block | etc...
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

  STREAMABLE(TeamList,
  {
    STREAMABLE(Team,
    {,
      (uint8_t) number,
      (std::string) name,
    }),

    (std::vector<Team>) teams,
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
  TeamList teamList; /**< The list of all teams for naming the log file after the opponent. */
  int blackboardVersion = 0; /**< The blackboard version the logger is currently configured for. */
  std::vector<Loggable> loggables; /**< The representations that should be logged. */
  std::vector<MessageQueue*> buffer; /**< Ring buffer of message queues. Shared with the writer thread. */
  std::string logFilename; /**< Path and name of the log file. Set in initial state. */
  bool receivedGameControllerPacket = false; /**< Ever received a packet from the GameController? */
  volatile int readIndex = 0; /**< The first index of the buffer that should be read by the writer thread. */
  volatile int writeIndex = 0; /**< Index of the buffer that is currently used for writing. */
  int frameCounter = 0; /**< Number of frames that are already in the current message queue. */
  Thread<Logger> writerThread;/**< Used to write the buffer to disk in the background */
  Semaphore framesToWrite; /**< How many frames the writer thread should write? */
  volatile bool writerIdle = true; /**< Is true if the writer thread has nothing to do. */
  volatile unsigned writerIdleStart = 0; /**< The system time at which the writer thread went idle. */
  OutBinaryFile* file = nullptr; /**< The stream that writes the log file. */
  std::vector<char> streamSpecification; /**< Streamed specification created in main thread and used in logger thread. */

  /**
   * Generate a filename containing the robot's player number, its name, and the current
   * date and time.
   * @return A log file name that starts with the path defined in the parameters.
   */
  std::string generateFilename() const;

  /** Create streamed data type specification to be used in the logger thread. */
  void createStreamSpecification();

  /** Write all loggable representations to a buffer. */
  void logFrame();

  /** Write contents of buffers to disk in the background. */
  void writeThread();

  /** Minimal behavior to handle logging. */
  option(Root)
  {
    ASSERT(Blackboard::getInstance().exists("GameInfo"));
    const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    receivedGameControllerPacket |= static_cast<const RoboCup::RoboCupGameControlData&>(gameInfo).packetNumber != 0 || gameInfo.secsRemaining != 0;

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
        goto waiting;
      }
      action
      {
        writerThread.start(this, &Logger::writeThread);
      }
    }

    state(waiting)
    {
      transition
      {
        if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
          goto prepareWriting;
      }
    }

    state(prepareWriting)
    {
      transition
      {
        goto running;
      }
      action
      {
        logFilename = generateFilename();
        logFrame(); // Stream all data once and theerby create stream specification for them
        createStreamSpecification(); // Serialize stream specification
      }
    }

    state(running)
    {
      transition
      {
        if(gameInfo.state == STATE_INITIAL || gameInfo.state == STATE_FINISHED)
          goto delayPlaySound;
        else if(file && SystemCall::getFreeDiskSpace(logFilename.c_str()) >> 20 < parameters.minFreeSpace)
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

    state(idle)
    {
      transition
      {
        if(gameInfo.state == STATE_READY || gameInfo.state == STATE_SET || gameInfo.state == STATE_PLAYING)
          goto running;
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
