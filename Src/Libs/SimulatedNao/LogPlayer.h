/**
 * @file LogPlayer.h
 *
 * This file declares a log player that records or plays back log files. It is
 * based on a message queue that stores the actual data. While the message
 * queue allows to access messages through iterators, this class manages the
 * messages as frames, which can be accessed through an index. The log player
 * is connected to a target message queue to which it copies messages to play
 * them back.
 * Most of the functionality is available if a log was loaded from a file. A
 * more limited set of methods can be used if the log player is recording a log
 * by itself. The reason is that several indices are computed for the logged
 * data, and since this is a costly operation, they are only created after
 * opening a log file and after a few selected operations that can usually be
 * applied to logs that were loaded from disk.
 * When loading a log file from disk, the log player tries to directly map the
 * file into the message queue, which means that the log file does need to be
 * loaded completely. When a log file is opened for the first time, indices are
 * computed and appended to the file. Further uses can directly load these
 * indices to avoid recreating them and thereby going through the whole log
 * file.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/MemoryMappedFile.h"
#include "Representations/AnnotationInfo.h"
#include "Streaming/MessageQueue.h"
#include "Streaming/TypeInfo.h"
#include <unordered_map>

class LogPlayer : public MessageQueue
{
  static const unsigned char indexVersion = 2; /**< The version of the index chunk. */
  MessageQueue& target; /**< The queue played back messages are copied to. */
  std::string path; /**< The file system path to the log file. */
  std::unique_ptr<MemoryMappedFile> file; /**< The memory mapped file if an uncompressed log was loaded from disk. */
  std::vector<MessageID> mapLogToID; /**< Maps message ids from the log to their current values. */
  std::vector<MessageID> mapIDToLog; /**< Maps message ids from their current values to the ones found in the log. */
  std::vector<std::string> logIDNames; /**< The message ids from the log as strings. */
  std::unique_ptr<TypeInfo> typeInfo; /**< The type information of the log file entries. */
  bool typeInfoRequested = false; /**< Should the type information be played back during the next call to \c playBack ? */
  std::vector<size_t> frameIndex; /**< The byte offsets of all frames relative to the beginning of the message queue. */
  std::vector<bool> framesHaveImage; /**< Determines for each frame whether it contains an image. */
  bool anyFrameHasImage = false; /**< Are there any frames with images in the log? */
  std::unordered_map<std::string, std::vector<std::pair<size_t, size_t>>> statsPerThread; /**< How often is each message id present in each thread and how much space is used? */
  std::unordered_map<std::string, std::vector<AnnotationInfo::AnnotationData>> annotationsPerThread; /**< Annotations per thread. */
  size_t sizeWhenIndexWasComputed = 0; /**< Remembers the size of the message queue when the indices were computed. */
  size_t currentFrame = -1; /**< The current frame, i.e. the one that was last played back. */

  /**
   * Reads the names of the message ids from a stream and fills the fields
   * \c mapLogToID , \c mapIDToLog , and \c logIDNames .
   * @param stream The stream to read from.
   */
  void readMessageIDs(In& stream);

  /**
   * Writes the names of the message ids to a stream.
   * @param stream The stream to write to.
   */
  void writeMessageIDs(Out& stream);

  /**
   * Update the indices, i.e. \c frameIndex , \c framesHaveImage ,
   * \c statsPerThread , and \c annotationsPerThread. In addition,
   * \c anyFrameHasImage and \c sizeWhenIndexWasComputed are updated as well.
   * If the last frame in the log is not complete, the queue is resized to
   * discard that frame.
   */
  void updateIndices();

  /**
   * Load the indices from a stream, i.e. \c frameIndex , \c framesHaveImage ,
   * \c statsPerThread , and \c annotationsPerThread. In addition,
   * \c anyFrameHasImage and \c sizeWhenIndexWasComputed are updated as well.
   * @param stream The stream to read from.
   * @param usedSize The size of that queue that contains complete frames. It is expected
   *                 that the queue will be resized to this value.
   * @return Could the indices be read? If not, they had the wrong format.
   */
  bool readIndices(In& stream, size_t& usedSize);

  /**
   * Write the indices to a stream.
   * @param stream The stream to write to.
   */
  void writeIndices(Out& stream) const;

  /**
   * Determines the statistics for a certain message id and optionally a thread.
   * @param id The message id as defined in the enumeration \c MessageID .
   * @param threadName The name of the thread the statistics should be limited
   *                   to. If empty, the statistics is for all threads.
   * @return A pair of number of messages and the overall size of these
   *         messages.
   */
  std::pair<size_t, size_t> statOf(MessageID id, const std::string& threadName = "") const;

public:
  /**
   * The current mode in which the log player is used. This field is only used
   * externally, i.e. the log player does not access it.
   */
  enum State {stopped, playing, recording} state = stopped;
  bool cycle = false; /**< Will playback continue at the beginning after reaching the end? */

  /**
   * Constructor.
   * @param target The queue played back messages are copied to.
   */
  LogPlayer(MessageQueue& target) : target(target) {clear();}

  /** Clear the queue and the indices. */
  void clear();

  /**
   * Opens a log file and might append indices to it.
   * @param fileName The name of the log file.
   * @return Could the file be opened? If not, the previous contents are still
   *         present.
   */
  bool open(const std::string& fileName);

  /**
   * Save log to a file.
   * @param fileName The name of the file. If empty, the name under which the
   *                 log was opened is used instead (if available).
   * @param typeInfo The type information is written to the log file if the log
   *                 player does not already store such information (from a
   *                 read log file).
   * @return Could the log file be saved? If not, it was not possible to create
   *         the file, either because the name is invalid or it was tried to
   *         overwrite the original file of this log while it was still mapped
   *         into memory.
   */
  bool save(std::string fileName, const TypeInfo& typeInfo);

  /**
   * Returns the message type translated to the current value in the
   * enumeration type. Any id that stems from a log file must be translated
   * using this method, in particular if the method \c filter is used.
   * @param message A message from the log file currently stored by this log
   *                player.
   * @return The corresponding constant in \c MessageID.
   */
  MessageID id(Message message) const;

  /**
   * Filters messages based on a used-defined criterion.
   * @param keep A function that returns whether a message should be kept. The
   *             parameter passed is the iterator to to the message in
   *             question. Note that \c id() defined above must be used to
   *             determine the id of the message the iterator points to rather
   *             than the member function of the message itself.
   */
  void filter(const std::function<bool(const_iterator)>& keep);

  /**
   * Determines the frequency of a certain message id and optionally for a
   * certain thread.
   * @param id The message id as defined in the enumeration \c MessageID .
   * @param threadName The name of the thread the frequency should be limited
   *                   to. If empty, the frequency is for all threads.
   * @return The frequency.
   */
  size_t frequencyOf(MessageID id, const std::string& threadName = "") const {return statOf(id, threadName).first;}

  /**
   * Determines the overall size for a certain message id and optionally a thread.
   * @param id The message id as defined in the enumeration \c MessageID .
   * @param threadName The name of the thread the overall size should be
   *                   limited to. If empty, the overall size is for all
   *                   threads.
   * @return The overall size.
   */
  size_t sizeOf(MessageID id, const std::string& threadName = "") const {return statOf(id, threadName).second;}

  /**
   * Returns all annotations in the log per thread.
   * @return The annotations per thread.
   */
  const std::unordered_map<std::string, std::vector<AnnotationInfo::AnnotationData>>& annotations() const;

  // The remaining methods are only available when a log file was loaded from
  // disk (and not cleared afterwards).

  /**
   * Returns the number of frames in the log.
   * @return The number of frames.
   */
  size_t frames() const {return frameIndex.size();}

  /**
   * Returns the number of the frame that was last played back.
   * @return A number between 0 and \c frames()-1 or \c size_t(-1) if no frame
   *         was played back yet.
   */
  size_t frame() const {return currentFrame;}

  /**
   * Plays back a frame.
   * @param frame The number of the frame to play back. If \c cycle is true,
   *              values larger than \c frames()-1 will be wrapped. Otherwise,
   *              they will be ignored. "Negative" numbers (if \c frame is
   *              interpreted as signed type) set the current frame to
   *              \c size_t(-1) and do not play back any frame.
   */
  void playBack(size_t frame);

  /**
   * Returns the next frame containing an image after a certain frame.
   * If \c cycle is true, this might wrap around the end of the log.
   * @param frame The number of the frame at which the search starts.
   * @return Either a frame containing an image or \c frame if the log does not
   *         contain any images. If \c cycle is false and there are no frames
   *         with images after \c frame, \c frames()-1 is returned.
   */
  size_t nextImageFrame(size_t frame) const;

  /**
   * Returns the previous frame containing an image before a certain frame.
   * @param frame The number of the frame at which the search starts.
   * @return Either a frame containing an image or \c frame if the log does not
   *         contain any images. If there are no frames with images before
   *         \c frame, 0 is returned.
   */
  size_t prevImageFrame(size_t frame) const;

  /**
   * Returns the name of the thread a frame belongs to.
   * @param frame The number of the frame.
   * @return The name of the thread.
   */
  std::string threadOf(size_t frame) const;

  /** Request that the type information will be inserted into the target queue. */
  void requestTypeInfo() {typeInfoRequested = true;}
};
