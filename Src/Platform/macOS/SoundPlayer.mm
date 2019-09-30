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
#include <AppKit/NSSpeechSynthesizer.h>
#include "SoundPlayer.h"
#include "Platform/File.h"

@interface SoundPlayerDelegate : NSObject<NSSoundDelegate, NSSpeechSynthesizerDelegate>
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

-(void)speechSynthesizer:(NSSpeechSynthesizer*) sender
       didFinishSpeaking:(BOOL) finishedSpeaking
{
  if(finishedSpeaking)
  {
    *isPlaying = false;
    sem->post();
  }
}
@end

SoundPlayer SoundPlayer::soundPlayer;
static SoundPlayerDelegate* delegate;
static NSSpeechSynthesizer* speechSynthesizer;

SoundPlayer::SoundPlayer()
{
  delegate = [[SoundPlayerDelegate alloc] initWithFlag:&playing andSemaphore:&sem];
  speechSynthesizer = [[NSSpeechSynthesizer alloc] initWithVoice:@"com.apple.speech.synthesis.voice.Alex"];
  [speechSynthesizer setDelegate:delegate];
}

SoundPlayer::~SoundPlayer()
{
  if(isRunning())
  {
    announceStop();
    sem.post();
    stop();
  }
  [speechSynthesizer release];
  [delegate release];
}

void SoundPlayer::start()
{
  Thread::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  Thread::nameCurrentThread("SoundPlayer");
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
  @autoreleasepool
  {
    if(!basename.empty() && basename[0] == ':')
      playing = [speechSynthesizer startSpeakingString:[NSString stringWithUTF8String:basename.c_str() + 1]];
    else
    {
      std::string fileName(filePrefix);
      fileName += basename;
      NSSound* sound = [[NSSound alloc] initWithContentsOfFile:[NSString stringWithUTF8String:fileName.c_str()] byReference:NO];
      [sound setDelegate:delegate];
      playing = static_cast<bool>([sound play]);
    }
  }
}

int SoundPlayer::play(const std::string& name)
{
  SYNC_WITH(soundPlayer);
  soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
  int queuelen = static_cast<int>(soundPlayer.queue.size());
  if(!soundPlayer.isRunning())
  {
    soundPlayer.filePrefix = File::getBHDir();
    soundPlayer.filePrefix += "/Config/Sounds/";
    soundPlayer.start();
  }
  else if(!soundPlayer.playing)
    soundPlayer.sem.post();

  return queuelen;
}

int SoundPlayer::say(const std::string& text)
{
  return play(":" + text);
}

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing;
}
