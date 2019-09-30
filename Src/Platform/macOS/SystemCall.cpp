#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif

#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/mount.h>

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
