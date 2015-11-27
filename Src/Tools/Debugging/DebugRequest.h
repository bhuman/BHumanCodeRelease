/**
 * @file DebugRequest.h
 * Declaration of class DebugRequest
 *
 * @author Matthias Jüngel
 */

#pragma once

#include "Tools/Streams/InOut.h"
#include <unordered_map>

class Framework;

struct DebugRequest
{
  std::string description = "empty";
  bool enable = true;

  /** Constructor, resets the table */
  DebugRequest() = default;
  DebugRequest(const std::string& description, bool enable = true);
};

/**
 * Streaming operator that reads a DebugRequest from a stream.
 * @param stream The stream from which is read.
 * @param debugRequest The DebugRequest object.
 * @return The stream.
 */
In& operator>>(In& stream, DebugRequest& debugRequest);

/**
 * Streaming operator that writes a DebugRequest to a stream.
 * @param stream The stream to write on.
 * @param debugRequest The DebugRequest object.
 * @return The stream.
 */
Out& operator<<(Out& stream, const DebugRequest& debugRequest);

class RobotConsole;

/**
 * @class DebugRequestTable
 *
 * A singleton class that maintains the table of currently active debug requests.
 */
class DebugRequestTable
{
private:
  enum { maxNumberOfDebugRequests = 1000 };

public:
  /** The Debug Key Table */
  DebugRequest debugRequests[maxNumberOfDebugRequests];
  int currentNumberOfDebugRequests = 0;

  bool poll = false;
  int pollCounter = 0;

  const char* alreadyPolledDebugRequests[maxNumberOfDebugRequests];
  int alreadyPolledDebugRequestCounter = 0;
  mutable const char* lastName = nullptr;

  mutable int lastIndex = 0;
  mutable std::unordered_map<const char*, int> nameToIndex;

  friend class Framework;

private:
  friend class Process;
  friend class RobotConsole;

  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via getDebugRequestTable
   * therefore the constructor is private.
   */
  DebugRequestTable() = default;
  // only a process is allowed to create the instance.

public:
  DebugRequestTable(const DebugRequestTable&) = delete;

  /**
   * Prints a message to stderr
   * @param message The error to print
   */
  static void print(const char* message);
  void addRequest(const DebugRequest& debugRequest, bool force = false);
  void removeRequest(const char* description);
  bool isActive(const char* name) const;
  void disable(const char* name);
  bool notYetPolled(const char* name);
  void removeAllRequests() { currentNumberOfDebugRequests = 0; }
};

inline bool DebugRequestTable::isActive(const char* name) const
{
  if(name == lastName)
    return lastIndex < currentNumberOfDebugRequests && debugRequests[lastIndex].enable;
  else if(currentNumberOfDebugRequests == 0)
    return false;
  else
  {
    lastName = name;
    std::unordered_map<const char*, int>::const_iterator iter = nameToIndex.find(name);
    if(iter != nameToIndex.end())
    {
      lastIndex = iter->second;
      return lastIndex < currentNumberOfDebugRequests && debugRequests[lastIndex].enable;
    }
    for(lastIndex = 0; lastIndex < currentNumberOfDebugRequests; ++lastIndex)
      if(debugRequests[lastIndex].description == name)
      {
        nameToIndex[name] = lastIndex;
        return debugRequests[lastIndex].enable;
      }
    nameToIndex[name] = lastIndex;
    return false;
  }
}
