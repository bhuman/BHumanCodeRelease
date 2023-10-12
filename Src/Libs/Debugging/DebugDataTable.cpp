/**
 * @file DebugDataTable.cpp
 * This file implements a table for generic handling of streamable debug data.
 * Representations mentioned in the table will be overwritten with the table
 * entry.
 *
 * @author Michael Spranger
 * @author Tobias Oberlies
 * @author Thomas Röfer
 */

#include "DebugDataTable.h"

DebugDataTable::~DebugDataTable()
{
  for(std::unordered_map<std::string, char*>::iterator iter = table.begin(); iter != table.end(); ++iter)
    delete[] iter->second;
}

void DebugDataTable::processChangeRequest(MessageQueue::Message message)
{
  std::string name;
  char change;
  auto stream = message.bin();
  stream >> name >> change;
  std::unordered_map<std::string, char*>::iterator iter = table.find(name);
  if(change)
  {
    size_t size = stream.getSize() - stream.getPosition();
    char* buffer = new char[size];
    stream.read(buffer, size);
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
