/*
 * File:   CognitionLogger.h
 * Author: Arne Böckmann
 *
 * Created on May 23, 2013, 2:59 PM
 */

#pragma once
#include "Tools/Enum.h"
#include <functional>
#include <unordered_map>
#include <string>
#include <vector>
#include <atomic>
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/Streams/OutStreams.h"
#include "Platform/Thread.h"
#include "Platform/Semaphore.h"
class Streamable;
class MessageQueue;
class ModuleManager;

/**
 * This is a simple state machine that logs representations and writes them to disk.
 * It is configured using the logger.cfg which should be located somewhere inside
 * the Config directory.
 *
 * In order to be logable a representation needs a message id. If it does not have one
 * the code will not compile.
 *
 * A representation is only logable if it is registered.
 * To register a new representation add it to the initRepresentations() method
 * inside the .cpp of this class. Do NOT add an include to the representation as it is
 * not needed.
 *
 * As mentioned above this is a state machine. It consists of the following states:
 *  PRE_INITIAL: If the logger is enabled it will immediately switch the state to INITAL.
 *               If the code is running inside the simulator the logger will never leave this state.
 *  INITIAL    : General setup, representations are sorted, memory allocation etc. Switch to IDLE afterwards.
 *  IDLE       : Stays in this state until the game state is ready|set|playing then switches to RUNNING.
 *  RUNNING    : Logs every frame until the game state switches to initial|finished then switches back to IDLE
 *
 * Everything that the logger logs is written into a ring buffer. The buffer size can be configured (see logger.cfg).
 * A non-real-time thread slowly writes the data from the buffer to the disk. If the bhuman process is stopped while
 * there is still data inside the buffer that data is lost!
 *
 * Usage of the logger:
 * run() should be called once in the end of every frame.
 */
class CognitionLogger {
public:
  /**Run one iteration of the logger state machine*/
  void run();
  ~CognitionLogger();

private:
    //State of the logger
  ENUM(State,
    PRE_INITIAL, // logger does nothing, is not even enabled.
    INITAL, //logger allocates memory, waits for game state to switch to ready
    IDLE, //logger does nothing, game state is finished or initial. Waits until the state changes
    RUNNING, //logger logs every frame until the game state switches to finished or initial
    ERROR_STATE //logger does nothing, can never recover from this state  (has to be named ERROR_STATE instead of ERROR because ERROR is a global variable in windows...)
  );

  /**This method is called every frame in state PRE_INITIAL*/
  void preInitial();
  /**This method is called every frame in state INITIAL*/
  void initial();
  /**This method is called every frame in state IDLE*/
  void idle();
  /**This method is called every frame in state RUNNING*/
  void running();
  /**This method is called every frame in state ERROR*/
  void error();
  /**Initializes the state functor array*/
  void initStateFunctors();
  /**Initializes the representations hash map */
  void initRepresentations();
  /**Sorts params.representations in order of execution*/
  void sortRepresentations();
  /**log the current frame*/
  void logFrame();
  /**calculate size of a streamble*/
  unsigned int getStreamableSize(const Streamable& s);
  /**implements a  mod b in a mathematically correct way*/
  int mod(int a, int b);
  /**A thread that writes the logged data from the buffer to the disk*/
  void writeThread();
  /**generates the filename for the log file. The mane contains the robot name, player number and the date*/
  std::string generateFilename();
  /**returns the free space left on the device in KB */
  unsigned getFreeSpace() const;

  CognitionLogger(const ModuleManager& moduleManager); //only the Cognition process may create the logger
  friend class Cognition;

private:
  struct Parameters;
  Parameters& params; /**< parameters loaded from logger.cfg */
  State state; /**< the current state the state machine is in */
  bool initialized; /**< has the logger already been initialized* */
  std::function<void ()> states[numOfStates]; /**< This array holds one functor per state. Whenever we enter a state the functor is called */
  std::unordered_map<std::string, std::pair<MessageID, Streamable*>> representations; /**< Map from representation names to actual representations and their message ids */

  std::vector<MessageQueue*> buffer; /**<  Ring buffer of message queues. Shared with the writer thread. */
  std::atomic<int> readIndex; /**< the first index of the buffer that should be read by the writer thread*/
  int writeIndex; /**< index of the buffer that is currently used for writing */
  unsigned int frameCounter; /**< number of frames that are already in the current messageQueue. */

  OutBinarySize sizeCounter; /**< used by getStreamableSize() to calculate the size */

  Thread<CognitionLogger> writerThread;/**< used to write the buffer to disk in the background */
  Semaphore framesToWrite; /**< How many frames the writer thread should write */
  std::string logFilename; /**< path and name of the log file. Set by initial state */
  std::atomic<bool> writerIdle; /**< Is true if the writer thread has nothing to do */
  std::atomic<unsigned> writerIdleStart; /**< The system time at which the writer thread went idle */
  bool shouldPlayLogSound; /**< what the name says */
  const ModuleManager& moduleManager; /**< Reference to the module manager. Used to gather information about execution order of modules */
  std::vector<std::string> representationNames; /**< contains all representations that should be logged in execution order */
};
