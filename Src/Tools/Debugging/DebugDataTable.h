/*
 * @file Tools/Debugging/DebugDataTable.h
 * This file declares a table for generic handling of streamable debug data.
 * Representations mentioned in the table will be overwritten with the table
 * entry.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Debugging.h"
#include <string>

class Framework;
class InMessage;

/**
* @class DebugDataTable
*
* A singleton class that maintains the debug data table.
*/
class DebugDataTable
{
private:
  std::unordered_map<std::string, char*> table;

  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via getDebugDataTable
   * therefore the constructor is private.
   */
  DebugDataTable() {}

  /**
   * Copy constructor.
   * Copying instances of this class is not allowed
   * therefore the copy constructor is private.
   */
  DebugDataTable(const DebugDataTable&) {}

  /*
   * Only a process is allowed to create the instance.
   */
  friend class Process;

public:
  ~DebugDataTable();

  /**
   * Registers the object with the debug data table and updates the object if the
   * respective entry in the table has been modified through RobotControl.
   */
  template<class T> void updateObject(const char* name, T& t)
  {
    // Find entry in debug data table
    std::unordered_map<std::string, char*>::iterator iter = table.find(name);
    if(iter != table.end())
    {
      InBinaryMemory stream(iter->second);
      stream >> t;
    }
  }

  void processChangeRequest(InMessage& in);

  /**
   * Functions for ensuring that object is streamable at compile time.
   */
  static inline void testForStreamable(const ImplicitlyStreamable& object) {}
  static inline void testForStreamable(const bool& object) {}
  static inline void testForStreamable(const int& object) {}
  static inline void testForStreamable(const unsigned int& object) {}
  static inline void testForStreamable(const long& object) {}
  static inline void testForStreamable(const unsigned long& object) {}
  static inline void testForStreamable(const float& object) {}
  static inline void testForStreamable(const double& object) {}
  static inline void testForStreamable(const std::string& object) {}

  friend class Framework;
};
