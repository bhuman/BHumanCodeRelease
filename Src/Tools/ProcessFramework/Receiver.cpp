/**
 * @file Tools/ProcessFramework/Receiver.cpp
 *
 * This file implements classes related to receivers.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Receiver.h"
#include "PlatformProcess.h"
#include "ProcessFramework.h"

ReceiverList::ReceiverList(PlatformProcess* p, const std::string& receiverName) :
  name(receiverName), // copy the receiver's name. The name of the process is still missing.
  process(p)
{
  if(getFirst())
  {
    ReceiverList* p = getFirst();
    while(p->next)
      p = p->next;
    p->next = this;
  }
  else
    getFirst() = this;
  for(int i = 0; i < 3; ++i)
    package[i] = 0;
}

ReceiverList::~ReceiverList()
{
  for(int i = 0; i < 3; ++i)
    if(package[i])
      delete[] (char*)package[i];
}

ReceiverList*& ReceiverList::getFirst()
{
  return process->getFirstReceiver();
}

void ReceiverList::checkAllForPackages()
{
  for(ReceiverList* p = getFirst(); p; p = p->getNext())
    p->checkForPackage();
}

ReceiverList* ReceiverList::lookup(const std::string& processName, const std::string& receiverName)
{
  for(ReceiverList* p = getFirst(); p; p = p->getNext())
    if(processName + "." + p->name == receiverName)
      return p;
  return 0;
}

void ReceiverList::setPackage(void* p)
{
  int writing = 0;
  if(writing == actual)
    ++writing;
  if(writing == reading)
    if(++writing == actual)
      ++writing;
  ASSERT(writing != actual);
  ASSERT(writing != reading);
  if(package[writing])
    delete[] (char*)package[writing];
  package[writing] = p;
  actual = writing;
  process->trigger();
}
