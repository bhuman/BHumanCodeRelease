#include "Platform/File.h"
#include "Platform/SystemCall.h"

#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/mount.h>

void SystemCall::getLoad(float& mem, float load[3])
{
  mem = -1.f;
  load[0] = load[1] = load[2] = -1.f;
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/Config/" + path;
  struct statfs data;
  if(!statfs(fullPath.c_str(), &data))
    return data.f_bavail * data.f_bsize;
  else
    return 0;
}

bool SystemCall::usbIsMounted()
{
  return false;
}
