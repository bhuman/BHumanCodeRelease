/**
* @file Controller/LogPlayer.h
*
* Definition of class LogPlayer
*
* @author Martin Lötzsch
*/

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"

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
  /**
  * Constructor
  * @param targetQueue The queue into that messages from played logfiles shall be stored.
  */
  LogPlayer(MessageQueue& targetQueue);

  /** Destructor. */
  ~LogPlayer() {if(streamHandler) delete streamHandler;}

  /** Deletes all messages from the queue */
  void init();

  /**
  * Opens a log file and reads all messages into the queue.
  * @param fileName the name of the file to open
  * @return if the reading was successful
  */
  bool open(const char* fileName);

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

  /** Starts recording.
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

  /** Set loop mode. If disabled the logfile is played only once. */
  void setLoop(bool);

  /**
  * Writes all messages in the log player queue to a log file.
  * @param fileName the name of the file to write
  * @param streamHandler Specification of logged data types. Will be ignored if
  *                      it is a nullptr or logger already has a specification.
  * @return Whether the writing was successful
  */
  bool save(const char* fileName, const StreamHandler* streamHandler);

  /**
  * Writes all audio data in the log player queue to a single wav file.
  * @param fileName the name of the file to write
  * @return if the writing was successful
  */
  bool saveAudioFile(const char* fileName);

  /**
  * Writes all images in the log player queue to a bunch of image files (.png).
  * @param raw Save color unconverted
  * @param fileName The name of one file to write, all files will be enumerated by appending a 3 digit number to the filename.
  * @return if writing all files was successful
  */
  bool saveImages(const bool raw, const char* fileName);

  /**
   * Writes a csv with all module timings
   * @return true if writing was successful
   */
  bool writeTimingData(const std::string& fileName);

  /**
  * Save an image to a file.
  * @param image The image to save.
  * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
  * @param imageNumber A number that will be integrated into the file name. -1: ignore.
  * @param YUV2RGB Convert from YUV to RGB?
  * @return Was writing successful?
  */
  static bool saveImage(const Image& image, const char* fileName, int imageNumber = -1, bool YUV2RGB = true);

  /**
  * Adds the message to the queue depending on isRecording.
  * That function should be called for every message in the queue that the
  * log player shall work on.
  */
  void handleMessage(InMessage& message);

  /**
  * If playing a log file, that function checks if it is time to release the next
  * message dependend on the time stamp. Call that function whenever there is some
  * processing time left.
  * @return Was log data replayed?
  */
  bool replay();

  /**
  * The functions filters the message queue.
  * @param messageIDs An null-terminated array of message ids that should be kept.
  */
  void keep(MessageID* messageIDs);

  /**
  * The functions filters the message queue.
  * @param messageIDs An null-terminated array of message ids that should be removed.
  */
  void remove(MessageID* messageIDs);

  /**
  * The function creates a histogram on the message ids contained in the log file.
  * @param frequency An array that is filled with the frequency of message ids.
  */
  void statistics(int frequencies[numOfDataMessageIDs], unsigned* sizes = nullptr, char processIdentifier = 0);

  /** different states of the logplayer */
  ENUM(LogPlayerState,
  {,
    initial,
    recording,
    paused,
    playing,
  });

  LogPlayerState state; /**< The state of the log player. */
  int currentFrameNumber; /**< The number of the current frame. */
  int numberOfFrames; /**< The overall number of frames available. */
  bool streamSpecificationReplayed; /**< The stream specification has to be replayed once. Already done? */

private:
  MessageQueue& targetQueue; /**< The queue into that messages from played logfiles shall be stored. */
  int currentMessageNumber; /**< The current message number in the message queue. */
  int numberOfMessagesWithinCompleteFrames; /**< The number of messages within complete frames. Messages behind that number will be skipped. */
  bool loop;
  int lastImageFrameNumber; /**< The number of the last frame that contained an image. */
  int replayOffset;
  std::vector<int> frameIndex; /**< The message numbers the frames start at. */
  StreamHandler* streamHandler; /**< The stream specification of the log file entries. */

  /**
  * The method counts the number of frames.
  */
  void countFrames();

  /**
   * Creates the index of the first message numbers of all frames.
   */
  void createFrameIndex();

  /**
  * The method expands image file name to its full path and integrates
  * an image number if desired.
  * @param fileName The short file name of the image.
  * @param imageNumber A number that will be integrated into the file name. -1: ignore.
  * @return The full path of the image file.
  */
  static std::string expandImageFileName(const char* fileName, int imageNumber);

  /**
  * Insert the stream specifiation into the target queue if one is available
  * and it has not been replayed yet.
  */
  void replayStreamSpecification();

  template<class T>
  std::string represetation2csv(Streamable *stream);
};


template<class T>
std::string LogPlayer::represetation2csv(Streamable *stream)
{
  return ((T*)stream)->csv();
}
