#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"

#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>

SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

void SystemCall::getLoad(float& mem, float load[3])
{
  struct sysinfo info;
  if(sysinfo(&info) == -1)
    load[0] = load[1] = load[2] = mem = -1.f;
  else
  {
    load[0] = float(info.loads[0]) / 65536.f;
    load[1] = float(info.loads[1]) / 65536.f;
    load[2] = float(info.loads[2]) / 65536.f;
    mem = float(info.totalram - info.freeram) / float(info.totalram);
  }
}

unsigned long long SystemCall::getFreeDiskSpace(const char* path)
{
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/" + path;
  struct statvfs data;
  if(!statvfs(fullPath.c_str(), &data))
    return static_cast<unsigned long long>(data.f_bfree)
           * static_cast<unsigned long long>(data.f_bsize);
  else
    return 0;
}

int SystemCall::playSound(const char* name)
{
  fprintf(stderr, "Playing %s\n", name);
  return SoundPlayer::play(name);
}

bool SystemCall::soundIsPlaying()
{
  return SoundPlayer::isPlaying();
}

bool SystemCall::usbIsMounted()
{
  return system("mount | grep \"media/usb\" >/dev/null") == 0;
}
