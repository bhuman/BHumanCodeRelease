/**
 * @file Log.h
 *
 * This file declares the class representing a log file.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#pragma once

#include "Frame.h"
#include "Platform/MemoryMappedFile.h"
#include "Streaming/TypeInfo.h"
#include <string>
#include <vector>

class Log : public MessageQueue
{
public:
  Log(const std::string& path, bool keepGoing = false);

  Frame iter();

  std::string headName;
  std::string bodyName;
  std::string scenario;
  std::string location;
  std::string identifier;
  int playerNumber = -1;
  std::string suffix;
  int numberOfFrames = 0;

private:
  void readMessageIDs(In& stream);

  /**
   * Returns the message type translated to the current value in the
   * enumeration type. Any id that stems from a log file must be translated
   * using this method.
   * @param message A message from the log file currently stored by this log
   *                player.
   * @return The corresponding constant in \c MessageID.
   */
  MessageID id(Message message) const;

  friend class Frame;
  std::unique_ptr<MemoryMappedFile> file; /**< The memory mapped file if an uncompressed log was loaded from disk. */
  TypeInfo typeInfo;
  bool keepGoing = false;
  const std::vector<std::string>* messageIDNames = nullptr;
  std::vector<MessageID> mapLogToID; /**< Maps message ids from the log to their current values. */
  std::vector<MessageID> mapIDToLog; /**< Maps message ids from their current values to the ones found in the log. */
};
