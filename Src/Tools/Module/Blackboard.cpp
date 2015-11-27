/**
 * The file implements a class that represents the blackboard containing all
 * representations used in a process.
 * @author Thomas Röfer
 */

#include "Blackboard.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <unordered_map>

/** The instance of the blackboard of the current process. */
static PROCESS_LOCAL Blackboard* theInstance = nullptr;

/** The actual type of the map for all entries. */
class Blackboard::Entries : public std::unordered_map<std::string, Blackboard::Entry> {};

Blackboard::Blackboard() :
  entries(*new Entries)
{
  theInstance = this;
}

Blackboard::~Blackboard()
{
  ASSERT(theInstance == this);
  theInstance = 0;
  ASSERT(entries.size() == 0);
  delete &entries;
}

Blackboard::Entry& Blackboard::get(const char* representation)
{
  return entries[representation];
}

const Blackboard::Entry& Blackboard::get(const char* representation) const
{
  return entries.find(representation)->second;
}

bool Blackboard::exists(const char* representation) const
{
  return entries.find(representation) != entries.end();
}

Streamable& Blackboard::operator[](const char* representation)
{
  Entry& entry = get(representation);
  ASSERT(entry.data);
  return *entry.data;
}

const Streamable& Blackboard::operator[](const char* representation) const
{
  const Entry& entry = get(representation);
  ASSERT(entry.data);
  return *entry.data;
}

void Blackboard::free(const char* representation)
{
  Entry& entry = get(representation);
  ASSERT(entry.counter > 0);
  if(--entry.counter == 0)
  {
    delete entry.data;
    entries.erase(representation);
    ++version;
  }
}

Blackboard& Blackboard::getInstance()
{
  return *theInstance;
}

void Blackboard::setInstance(Blackboard& blackboard)
{
  theInstance = &blackboard;
}
