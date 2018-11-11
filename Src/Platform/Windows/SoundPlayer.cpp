/**
 * @file Platform/Windows/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention this is the Windows implementation
 * @author Colin Graf
 * @author Lukas Post
 */

#include <Windows.h>

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer() :
  started(false), closing(false)
{}

SoundPlayer::~SoundPlayer()
{
  if(started)
  {
    closing = true;
    sem.post();
    stop();
  }
}

void SoundPlayer::start()
{
  Thread::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  Thread::nameThread("SoundPlayer");
  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
}

void SoundPlayer::playDirect(const std::string& basename)
{
  playing = true;
  std::string filePath(filePrefix);
  filePath += basename;
  PlaySound(filePath.c_str(), nullptr, SND_SYNC | SND_FILENAME);
  playing = false;
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
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
    queuelen = static_cast<int>(soundPlayer.queue.size());
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.filePrefix = File::getBHDir();
      soundPlayer.filePrefix += "/Config/Sounds/";
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queuelen;
}

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing;
}
