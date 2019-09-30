/**
 * @file DebugRequest.h
 *
 * Declaration of a class that manages debug requests.
 * Debug requests are switches that (de)activate blocks of code at runtime.
 *
 * @author Matthias Jüngel
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @class DebugRequest
 *
 * The debug requests that are sent from the GUI to the robot code.
 */
STREAMABLE(DebugRequest,
{
  DebugRequest() = default;
  DebugRequest(const std::string& name, bool enable = true),

  (std::string) name,
  (bool)(true) enable,
});

inline DebugRequest::DebugRequest(const std::string& name, bool enable)
  : name(name), enable(enable) {}

/**
 * @class DebugRequestTable
 *
 * A singleton class that maintains the table of currently active debug requests.
 * It provides a fast access based on character pointers and a slower one based
 * on strings.
 */
class DebugRequestTable
{
private:
  std::vector<char> enabled; /**< Are requests enabled or disabled? */
  std::unordered_map<const char*, size_t> fastIndex; /**< Maps char pointers to entries of vector "enabled". */
  std::unordered_map<std::string, size_t> slowIndex; /**< Maps strings to entries of vector "enabled". */
  std::unordered_set<const char*> polled; /**< Which requests were already published during this polling phase? */

  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via Global::getDebugRequestTable.
   * Therefore the constructor is private.
   */
  DebugRequestTable();

  /**
   * Uses the slow index to find request and updates the fast index.
   * @param name The name of the debug request.
   * @return Is it active?
   */
  bool isActiveSlow(const char* name);

public:
  int pollCounter = 0; /**< How many frames is polling still active? */

  /** No copy constructor. */
  DebugRequestTable(const DebugRequestTable&) = delete;

  /**
   * Adds or updates a certain debug request.
   * @param debugRequest The debug request that is updated in the table.
   */
  void addRequest(const DebugRequest& debugRequest);

  /**
   * Is a debug request active?
   * @param name The name of the request.
   * @return Is it active?
   */
  bool isActive(const char* name);

  /**
   * Disable a debug request.
   * Note: isActive must have been called before for this request.
   * @param name The name of the request to disable.
   */
  void disable(const char* name);

  /**
   * Has this request still to be published during this polling phase?
   * This also marks the request as polled.
   * @param name The name of the request.
   * @return Was the request not yet polled?
   */
  bool notYetPolled(const char* name);

  /** Clear the table. */
  void clear();

  /**
   * Prints a message to stderr.
   * @param message The error to print.
   */
  static void print(const char* message);

  friend class ConsoleRoboCupCtrl;
  friend class RobotConsole;
  friend class ThreadFrame; /**< A thread is allowed to create the instance. */
};

inline bool DebugRequestTable::isActive(const char* name)
{
  std::unordered_map<const char*, size_t>::const_iterator i = fastIndex.find(name);
  return i != fastIndex.end() ? enabled[i->second] != 0 : isActiveSlow(name);
}
