/**
 * @file  Platform/Linux/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention This is the Linux implementation for the NAO
 * @author Colin Graf
 * @author Lukas Post
 * @author Thomas Röfer
 */

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <unistd.h>
#include <sys/wait.h>

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer() :
  started(false), closing(false)
{
}

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
  Thread::nameThread("SoundPlayer");
  BH_TRACE_INIT("SoundPlayer");
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
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, 2));
  VERIFY(!snd_pcm_hw_params(handle, params));
  VERIFY(!snd_pcm_hw_params_get_period_size(params, &periodSize, 0));
  snd_pcm_hw_params_free(params);

  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
  VERIFY(!snd_pcm_close(handle));
}

void SoundPlayer::playDirect(const std::string& basename)
{
  std::string fileName(filePrefix);
  fileName += basename;

  File file(fileName, "rb");
  if(file.exists())
  {
    playing = true;
    size_t size = file.getSize();
    char* buf = new char[size];
    file.read(buf, size);
    int channels = *reinterpret_cast<unsigned short*>(buf + 22);
    char* data = buf + *reinterpret_cast<unsigned*>(buf + 16) + 28;
    snd_pcm_uframes_t frames = static_cast<snd_pcm_uframes_t>(*reinterpret_cast<unsigned*>(data - 4) / sizeof(short) / channels);

    const unsigned* p;
    if(channels == 1)
    {
      char* buf2 = new char[frames * 4];
      for(short* pSrc = reinterpret_cast<short*>(data), *pEnd = pSrc + frames, *pDst = reinterpret_cast<short*>(buf2); pSrc < pEnd; ++pSrc)
      {
        *pDst++ = *pSrc;
        *pDst++ = *pSrc;
      }
      delete [] buf;
      buf = buf2;
      p = reinterpret_cast<unsigned*>(buf);
    }
    else
      p = reinterpret_cast<unsigned*>(data);

    VERIFY(!snd_pcm_prepare(handle));
    while(frames > 0)
    {
      snd_pcm_uframes_t periodFrames = std::min(frames, periodSize);
      int err = snd_pcm_writei(handle, p, periodFrames);
      if(err < 0)
      {
        err = snd_pcm_recover(handle, err, 0);
        snd_strerror(err);
      }
      p += periodFrames;
      frames -= periodFrames;
    }
    VERIFY(!snd_pcm_drain(handle));
    delete [] buf;
    playing = false;
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
