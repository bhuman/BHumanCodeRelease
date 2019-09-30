#include "Platform/BHAssert.h"
#include "Platform/Thread.h"

void Thread::nameCurrentThread(const std::string& name)
{
  char cname[16] = "";
  name.copy(cname, 15);
  cname [15] = '\0';
  VERIFY(!pthread_setname_np(pthread_self(), cname));
}
