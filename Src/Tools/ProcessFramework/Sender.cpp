/**
 * @file Tools/ProcessFramework/Sender.cpp
 *
 * This file implements classes related to senders.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Sender.h"
#include "ProcessFramework.h"
#include "PlatformProcess.h"

SenderList::SenderList(PlatformProcess* p, const std::string& senderName) :
  name(senderName), // copy the sender's name. The name of the process is still missing.
  process(p)
{
  if(getFirst())
  {
    SenderList* p = getFirst();
    while(p->next)
      p = p->next;
    p->next = this;
  }
  else
    getFirst() = this;
}

SenderList*& SenderList::getFirst()
{
  return process->getFirstSender();
}

void SenderList::sendAllUnsentPackages()
{
  for(SenderList* p = getFirst(); p; p = p->getNext())
    p->sendPackage();
}

SenderList* SenderList::lookup(const std::string& processName, const std::string& senderName)
{
  for(SenderList* p = getFirst(); p; p = p->getNext())
    if(processName + "." + p->name == senderName)
      return p;
  return 0;
}
