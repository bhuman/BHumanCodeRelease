#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif

#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>

SystemCall::Mode SystemCall::getMode()
{
#ifdef TARGET_SIM
  if(RoboCupCtrl::controller)
  {
    static thread_local SystemCall::Mode mode = static_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller)->getMode();
    return mode;
  }
  else
#endif
    return simulatedRobot;
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
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/Config/" + path;
  struct statvfs data;
  if(!statvfs(fullPath.c_str(), &data))
    return data.f_bavail * data.f_bsize;
  else
    return 0;
}

int SystemCall::playSound(const char* name)
{
  return SoundPlayer::play(name);
}

int SystemCall::say(const char* text)
{
  return SoundPlayer::say(text);
}

bool SystemCall::soundIsPlaying()
{
  return SoundPlayer::isPlaying();
}

bool SystemCall::usbIsMounted()
{
  return false;
}
