/**
 * @file  Platform/Linux/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention this is the Linux implementation
 * @author Colin Graf
 * @author Lukas Post
 * @author Thomas Röfer
 * @author Jan Blumenkamp
 */

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <unistd.h>
#include <sys/wait.h>

extern "C" cst_voice* register_cmu_us_slt(const char*);

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer() :
  started(false), closing(false)
{
  flite_init();
  voice = register_cmu_us_slt(nullptr);

  filePrefix = File::getBHDir();
  filePrefix += "/Config/Sounds/";
}

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
  Thread::nameCurrentThread("SoundPlayer");
  BH_TRACE_INIT("SoundPlayer");
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
    if(snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0) >= 0)
      break;
    Thread::sleep(retryDelay);
  }
  ASSERT(i < retries);
  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(snd_pcm_hw_params_any(handle, params) >= 0);
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE));
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, nullptr));
  VERIFY(!snd_pcm_hw_params_set_channels(handle, params, 2));
  VERIFY(!snd_pcm_hw_params(handle, params));
  VERIFY(!snd_pcm_hw_params_get_period_size(params, &periodSize, nullptr));
  snd_pcm_hw_params_free(params);

  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
  VERIFY(!snd_pcm_close(handle));
}

void SoundPlayer::playWave(const Wave& wave)
{
  auto frames = static_cast<snd_pcm_uframes_t>(wave.numFrames);
  auto* p = reinterpret_cast<const unsigned*>(wave.data.data());

  VERIFY(!snd_pcm_prepare(soundPlayer.handle));
  while(frames > 0)
  {
    snd_pcm_uframes_t periodFrames = std::min(frames, soundPlayer.periodSize);
    int err = static_cast<int>(snd_pcm_writei(soundPlayer.handle, p, periodFrames));
    if(err < 0)
    {
      err = snd_pcm_recover(soundPlayer.handle, err, 0);
      snd_strerror(err);
    }
    p += periodFrames;
    frames -= periodFrames;
  }
  VERIFY(!snd_pcm_drain(soundPlayer.handle));
}

void SoundPlayer::flush()
{
  for(;;)
  {
    SoundRequest first;
    {
      SYNC;
      if(queue.empty())
        break;
      first = queue.front();
      queue.pop_front();
    }

    playing = true;
    if(first.isTextToSpeech && !first.fileOrText.empty())
    {
      bool isStretched = first.ttsDurationStretchFactor != 1.f;
      const std::string& text = first.fileOrText;
      flite_feat_set_float(soundPlayer.voice->features, "duration_stretch", first.ttsDurationStretchFactor);

      if(isStretched)
      {
        cst_wave* rawWave = flite_text_to_wave(text.c_str(), soundPlayer.voice);
        cst_wave_rescale(rawWave, static_cast<int>(textToSpeechVolumeFactor * 65536));
        soundPlayer.playWave(Wave(rawWave));
        delete_wave(rawWave);
      }
      else
      {
        auto soundInMap = soundPlayer.synthesizedSounds.find(text);
        if(soundInMap == soundPlayer.synthesizedSounds.end())
        {
          // Sound not yet synthesized => Synthesize and insert into map
          cst_wave* rawWave = flite_text_to_wave(text.c_str(), soundPlayer.voice);
          cst_wave_rescale(rawWave, static_cast<int>(textToSpeechVolumeFactor * 65536));
          soundInMap = soundPlayer.synthesizedSounds.emplace(std::make_pair(text, Wave(rawWave))).first;
          delete_wave(rawWave);
        }
        soundPlayer.playWave(soundInMap->second);
      }
    }
    else
    {
      File file(soundPlayer.filePrefix + first.fileOrText, "rb");
      if(file.exists())
      {
        Wave wave(file);
        soundPlayer.playWave(wave);
      }
    }
    playing = false;
  }
}

int SoundPlayer::enqueue(const SoundRequest& soundRequest)
{
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(soundRequest);
    queuelen = static_cast<int>(soundPlayer.queue.size());
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queuelen;
}

int SoundPlayer::play(const std::string& name)
{
  return enqueue(SoundRequest(name));
}

int SoundPlayer::say(const std::string& text, float speed)
{
  float stretchFactor = speed > 0.f ? 1.f/speed : 0.f;
  return enqueue(SoundRequest(text, stretchFactor));
}

bool SoundPlayer::isPlaying()
{
  return soundPlayer.playing;
}
