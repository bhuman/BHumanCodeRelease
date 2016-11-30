/**
 * @file  Platform/Linux/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention this is the Linux implementation
 * @author Colin Graf
 * @author Lukas Post
 */

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <unistd.h>
#include <sys/wait.h>
//#include <math.h>

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
  }
}

void SoundPlayer::start()
{
  Thread::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
}

void SoundPlayer::playDirect(const std::string& basename)
{
  std::string fileName(filePrefix);
  fileName += basename;

  int r = vfork();
  if(r == -1)
    perror("SoundPlayer: fork() failed");
  else if(r != 0)  //parent
  {
    int status;
    waitpid(r, &status, 0);
  }
  else //child
  {
    if(execlp("aplay", "aplay", "-q", fileName.c_str(), (char*)0) == -1)
      perror("SoundPlayer: exec failed");
  }
}

void SoundPlayer::playDirect(std::vector<short>& samples)
{
#ifdef TARGET_ROBOT
  ////////////////SETUP//////////////////
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(snd_pcm_open(&handle, "hw:0", SND_PCM_STREAM_PLAYBACK, 0) >= 0)
      break;
    Thread::sleep(retryDelay);
  }
  ASSERT(i < retries);
  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(!snd_pcm_hw_params_any(handle, params));
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, 0));;
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, channels));
  VERIFY(!snd_pcm_hw_params(handle, params));
  VERIFY(!snd_pcm_hw_params_get_period_size(params, &periodSize, 0));
  snd_pcm_hw_params_free(params);
  VERIFY(!snd_pcm_prepare(handle));
  ASSERT(channels <= 2);

  int err = 0;
  short* buf = new short[0];
  err = snd_pcm_writei(handle, buf, 0);
  VERIFY(err >= 0);

  delete[] buf;
  ///////////////END SETUP///////////////
  long length = samples.size();
  if(length % 2 != 0)
  {
    return;
  }
  short* dataptr = samples.data();
  while(length > 0)
  {
    err = snd_pcm_writei(handle, dataptr, std::min((unsigned long)length / 2, periodSize));
    if(err < 0)
    {
      err = snd_pcm_recover(handle, err, 0);
      snd_strerror(err);
    }
    length = length - (periodSize * 2);
    dataptr = dataptr + (periodSize * 2);
  }
  snd_pcm_drain(handle);
  snd_pcm_close(handle);
#endif
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
  for(;;)
  {
    std::vector<short> first;
    {
      SYNC;
      if(0 == sampleQueue.size())
        break;
      first = sampleQueue.front();
      sampleQueue.pop_front();
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

int SoundPlayer::playSamples(std::vector<short>& samples)
{
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.sampleQueue.push_back(samples); // avoid copy-on-write
    queuelen = static_cast<int>(soundPlayer.sampleQueue.size());
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
