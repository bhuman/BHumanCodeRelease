/**
* @file  Platform/OSX/SoundPlayer.cpp
* Implementation of class SoundPlayer.
* @attention This is the MacOS implementation.
* @author Colin Graf
* @author Thomas Röfer
*/

#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstdlib>
#include <cstdio>
#include <AppKit/NSSound.h>
#include "SoundPlayer.h"
#include "Platform/File.h"

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::~SoundPlayer()
{
  if(isRunning())
  {
    announceStop();
    sem.post();
  }
}

void SoundPlayer::start()
{
  Thread<SoundPlayer>::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  while(isRunning())
  {
    flush();
    sem.wait();
  }
}

void SoundPlayer::playDirect(const std::string& basename)
{
  std::string fileName(filePrefix);
  fileName += basename;
  @autoreleasepool
  {
    NSSound* sound = [[NSSound alloc] initWithContentsOfFile:[NSString stringWithUTF8String:fileName.c_str()] byReference:NO];
    [sound play];
  }
}

void SoundPlayer::flush()
{
  for(;;)
  {
    std::string first;
    {
      SYNC;
      if(0 == queue.size())
        break;
      first = queue.front();
      queue.pop_front();
    }

    playDirect(first);
  }
}

int SoundPlayer::play(const std::string& name)
{
  SYNC_WITH(soundPlayer);
  soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
  int queuelen = (int) soundPlayer.queue.size();
  if(!soundPlayer.isRunning())
  {
    soundPlayer.filePrefix = File::getBHDir();
    soundPlayer.filePrefix += "/Config/Sounds/";
    soundPlayer.start();
  }
  else
    soundPlayer.sem.post();

  return queuelen;
}
