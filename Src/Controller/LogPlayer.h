/**
 * @file Controller/LogPlayer.h
 *
 * Definition of class LogPlayer
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/TypeInfo.h"

#include <array>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <string>
#include <vector>

/**
 * @class LogPlayer
 *
 * A message queue that can record and play logfiles.
 * The messages are played in the same time sequence as they were recorded.
 *
 * @author Martin Lötzsch
 */
class LogPlayer : public MessageQueue
{
public:
  /** different states of the logplayer */
  ENUM(LogPlayerState,
  {,
    initial,
    recording,
    paused,
    playing,
  });

  LogPlayerState state = initial; /**< The state of the log player. */
  int currentFrameNumber; /**< The number of the current frame. */
  int numberOfFrames; /**< The overall number of frames available. */
  bool typeInfoReplayed; /**< The type information has to be replayed once. Already done? */
  int lastImageFrameNumber; /**< The number of the last frame that contained an image. */
  std::string logfilePath;

private:
  friend class LogExtractor; /**< The LogExtractor use queue and logfilePath. */
  MessageQueue& targetQueue; /**< The queue into that messages from played logfiles shall be stored. */
  int currentMessageNumber; /**< The current message number in the message queue. */
  int numberOfMessagesWithinCompleteFrames; /**< The number of messages within complete frames. Messages behind that number will be skipped. */
  bool loop;
  int replayOffset;
  std::vector<int> frameIndex; /**< The message numbers the frames start at. */
  std::array<int, 601> gcTimeIndex; /**< The frames correspending to Game Controller times. */
  std::unique_ptr<TypeInfo> typeInfo; /**< The type information of the log file entries. */

public:
  /**
   * @param targetQueue The queue into that messages from played logfiles shall be stored.
   */
  LogPlayer(MessageQueue& targetQueue);

  /** Deletes all messages from the queue */
  void init();

  /**
   * Opens a log file and reads all messages into the queue.
   * @param fileName the name of the file to open
   * @return if the reading was successful
   */
  bool open(const std::string& fileName);

  /**
   * Plays the queue.
   * Note that you have to call replay() regularly if you want to use that function
   */
  void play();

  /** Pauses playing the queue. */
  void pause();

  /** Stops playing the queue, resets the position in the queue to the first message */
  void stop();

  /** Stops recording, resets the position to the first message */
  void recordStop();

  /**
   * Starts recording.
   * Note that you have to notify the queue on new messages with handleMessage().
   */
  void recordStart();

  /** Plays the next message in the queue */
  void stepForward();

  /** Plays messages in the queue until a frame that contains an image got copied. */
  void stepImageForward();

  /** Plays the previous message in the queue */
  void stepBackward();

  /** Plays previous messages in the queue until a frame that contains an image got copied. */
  void stepImageBackward();

  /** repeats the current message in the queue */
  void stepRepeat();

  /** jumps to given message-number in the queue */
  void gotoFrame(int frame);

  /** Set loop mode. If disabled the log file is played only once. */
  void setLoop(bool);

  bool getLoop() const { return loop; };

  /** Returns the message number of the first frame in the queue for which the remaining GameController time is as given or -1 if no such frame exists. */
  int getFrameForRemainingGCTime(int time);

  /**
   * Adds the message to the queue depending on isRecording.
   * That function should be called for every message in the queue that the
   * log player shall work on.
   */
  void handleMessage(InMessage& message);

  /**
   * If playing a log file, that function checks if it is time to release the next
   * message dependend on the timestamp. Call that function whenever there is some
   * processing time left.
   * @return Was log data replayed?
   */
  bool replay();

  /**
   * The function filters the message queue.
   * @param filter Returns whether a message should be kept.
   */
  void keep(const std::function<bool(InMessage&)>& filter);

  /**
   * The function filters the message queue frame-wise.
   * @param filter Returns whether a frame should be kept because
   *               of this message.
   */
  void keepFrames(const std::function<bool(InMessage&)>& filter);

  void trim(int startFrame, int endFrame);

  /**
   * The function filters the message queue by message numbers.
   * @param savedMessages Vector of message numbers that should be kept.
   */
  void keep(const std::vector<int>& savedMessages);

  /**
   * The function creates a histogram on the message ids contained in the log file.
   * @param frequency An array that is filled with the frequency of message ids.
   * @param sizes The accumulated message sizes per id. Ignored if nullptr.
   * @param threadIdentifier If set, only consider messages from this thread.
   * @param init Initialize counters before counting.
   */
  void statistics(int frequencies[numOfDataMessageIDs], unsigned* sizes = nullptr,
                  const std::string& threadIdentifier = "", bool init = true);

  /**
   * Loads labels for the current log from a file and adds them to the log.
   */
  void loadLabels();

  /**
   * Insert the type information into the target queue if one is available
   * and it has not been replayed yet.
   */
  void replayTypeInfo();

  /**
   * Returns the thread identifier of the next frame.
   * @return The thread identifier or an empty string if the next message is not
   *         an idFrameBegin.
   */
  std::string getThreadIdentifierOfNextFrame();

private:
  /**
   * The method counts the number of frames.
   */
  void countFrames();

  /**
   * Creates the index of the first message numbers of all frames as well as the
   * index of frames corresponding to Game Controller times.
   */
  void createIndices();

  /** Renames all frames called "Upper" that contain lower camera data to "Lower". */
  void upgradeFrames();
};
