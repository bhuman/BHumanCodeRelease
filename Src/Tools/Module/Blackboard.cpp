/**
 * The file implements a class that represents the blackboard containing all
 * representations used in a thread.
 * @author Thomas Röfer
 */

#include "Blackboard.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <unordered_map>

/** The instance of the blackboard of the current thread. */
static thread_local Blackboard* theInstance = nullptr;

/** The actual type of the map for all entries. */
class Blackboard::Entries : public std::unordered_map<std::string, Blackboard::Entry> {};

Blackboard::Blackboard() :
  entries(new Entries)
{
  theInstance = this;
}

Blackboard::~Blackboard()
{
  ASSERT(theInstance == this);
  theInstance = nullptr;
  ASSERT(entries->size() == 0);
}

Blackboard::Entry& Blackboard::get(const char* representation)
{
  return (*entries)[representation];
}

const Blackboard::Entry& Blackboard::get(const char* representation) const
{
  return entries->find(representation)->second;
}

bool Blackboard::exists(const char* representation) const
{
  return entries->find(representation) != entries->end();
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
    entries->erase(representation);
    ++version;
  }
}

void Blackboard::reset(const char* representation)
{
  Entry& entry = get(representation);
  entry.reset(&*entry.data);
}

Blackboard& Blackboard::getInstance()
{
  return *theInstance;
}

void Blackboard::setInstance(Blackboard& blackboard)
{
  theInstance = &blackboard;
}
