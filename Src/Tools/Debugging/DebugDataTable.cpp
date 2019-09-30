/**
 * @file Tools/Debugging/DebugDataTable.cpp
 * This file implements a table for generic handling of streamable debug data.
 * Representations mentioned in the table will be overwritten with the table
 * entry.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#include "Tools/Debugging/DebugDataTable.h"
#include "Tools/MessageQueue/InMessage.h"

DebugDataTable::~DebugDataTable()
{
  for(std::unordered_map< std::string, char*>::iterator iter = table.begin(); iter != table.end(); ++iter)
    delete[] iter->second;
}

void DebugDataTable::threadChangeRequest(InMessage& in)
{
  std::string name;
  char change;
  in.bin >> name >> change;
  std::unordered_map<std::string, char*>::iterator iter = table.find(name);
  if(change)
  {
    int size = in.getBytesLeft();
    char* buffer = new char[size];
    in.bin.read(buffer, size);
    if(iter == table.end())
      table[name] = buffer;
    else
    {
      delete[] iter->second;
      iter->second = buffer;
    }
  }
  else if(iter != table.end())
  {
    delete[] iter->second;
    table.erase(iter);
  }
}
