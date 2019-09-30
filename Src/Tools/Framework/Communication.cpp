/**
 * @file Tools/Framework/Communication.cpp
 *
 * This file implements classes related to the communication between threads.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#include "Communication.h"
#include "Tools/Framework/ThreadFrame.h"

bool DebugSenderBase::terminating = false;

void ReceiverBase::setPacket(void* p)
{
  int writing = 0;
  if(writing == actual)
    ++writing;
  if(writing == reading)
    if(++writing == actual)
      ++writing;
  ASSERT(writing != actual);
  ASSERT(writing != reading);
  if(packet[writing])
    std::free(packet[writing]);
  packet[writing] = p;
  actual = writing;
  thread->trigger();
}
