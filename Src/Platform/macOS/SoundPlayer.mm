/**
 * @file  Platform/macOS/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention This is the macOS implementation.
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

@interface SoundPlayerDelegate : NSObject<NSSoundDelegate>
{
  bool* isPlaying;
  Semaphore* sem;
}
-(id)initWithFlag:(bool*)pIsPlaying andSemaphore:(Semaphore*) pSem;
@end

@implementation SoundPlayerDelegate
-(id)initWithFlag:(bool*)pIsPlaying andSemaphore:(Semaphore*) pSem
{
  isPlaying = pIsPlaying;
  sem = pSem;
  return self;
}

-(void)sound:(NSSound*)sound didFinishPlaying:(BOOL)finishedOk
{
  *isPlaying = false;
  sem->post();
}
@end

SoundPlayer SoundPlayer::soundPlayer;
static SoundPlayerDelegate* delegate;

SoundPlayer::SoundPlayer()
{
  delegate = [[SoundPlayerDelegate alloc] initWithFlag:&playing andSemaphore:&sem];
}

SoundPlayer::~SoundPlayer()
{
  if(isRunning())
  {
    announceStop();
    sem.post();
    stop();
  }
  [delegate release];
}

void SoundPlayer::start()
{
  Thread::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  Thread::nameThread("SoundPlayer");
  while(isRunning())
  {
    if(!playing)
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
    [sound setDelegate:delegate];
    playing = (bool) [sound play];
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

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing;
}
