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
  std::string fullPath = File::isAbsolute(path) ? path : std::string(File::getBHDir()) + "/Config/" + path;
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

int SystemCall::say(const char* text)
{
  fprintf(stderr, "Saying %s\n", text);
  if(!strcmp(text, "Upright"))
    return SoundPlayer::play("abseits.wav");
  else if(!strcmp(text, "Falling"))
    return SoundPlayer::play("falling.wav");
  else if(!strcmp(text, "Fallen"))
    return SoundPlayer::play("fallen.wav");
  else if(!strcmp(text, "Fire"))
    return SoundPlayer::play("fire.wav");
  else if(!strcmp(text, "Fire exclamation mark"))
    return SoundPlayer::play("fireExclamationMark.wav");
  else if(!strcmp(text, "forward left"))
    return SoundPlayer::play("forwardLeft.wav");
  else if(!strcmp(text, "forward right"))
    return SoundPlayer::play("forwardRight.wav");
  else if(!strcmp(text, "Ground"))
    return SoundPlayer::play("ground.wav");
  else if(!strcmp(text, "Heat"))
    return SoundPlayer::play("heat.wav");
  else if(!strcmp(text, "High"))
    return SoundPlayer::play("high.wav");
  else if(!strcmp(text, "Jump"))
    return SoundPlayer::play("jump.wav");
  else if(!strcmp(text, "log file written"))
    return SoundPlayer::play("logWritten.wav");
  else if(!strcmp(text, "Low battery"))
    return SoundPlayer::play("lowBattery.wav");
  else if(!strcmp(text, "Penalized"))
    return SoundPlayer::play("penalized.wav");
  else if(!strcmp(text, "Not penalized"))
    return SoundPlayer::play("notPenalized.wav");
  else if(!strcmp(text, "Picked up"))
    return SoundPlayer::play("pickedUp.wav");
  else if(!strcmp(text, "squatting"))
    return SoundPlayer::play("squatting.wav");
  else if(!strcmp(text, "USB mounted"))
    return SoundPlayer::play("usb_mounted.wav");
  else
    return SoundPlayer::say(text);
}

bool SystemCall::soundIsPlaying()
{
  return SoundPlayer::isPlaying();
}

bool SystemCall::usbIsMounted()
{
  return system("mount | grep \"media/usb\" >/dev/null") == 0;
}
