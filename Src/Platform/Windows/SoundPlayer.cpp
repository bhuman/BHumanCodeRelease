/**
 * @file Platform/Windows/SoundPlayer.cpp
 * Implementation of class SoundPlayer.
 * @attention this is the Windows implementation
 * @author Colin Graf
 * @author Lukas Post
 * @author Jan Fiedler
 */

#include <Windows.h>
#include <sapi.h>
#pragma warning( push )
// Disable the following warning for the include:
// C:\Program Files(x86)\Windows Kits\10\Include\10.0.17763.0\um\sphelper.h(1319, 1) : warning C4996 : 'GetVersionExA' : was declared deprecated
#pragma warning( disable : 4996 )
#include <sphelper.h>
#pragma warning( pop )

#include "SoundPlayer.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer() :
  started(false), closing(false)
{
  soundPlayer.filePrefix = File::getBHDir();
  soundPlayer.filePrefix += "/Config/Sounds/";
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

  // Initialize TTS for this thread.
  VERIFY(SUCCEEDED(CoInitialize(NULL)));
  const HRESULT& hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, reinterpret_cast<void**>(& pVoice));
  ASSERT(SUCCEEDED(hr));

  // Set language to (Microsoft Zira Desktop - English (United States)) from HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens
  ISpObjectToken* cpToken(NULL);
  VERIFY(SUCCEEDED(SpFindBestToken(SPCAT_VOICES, L"Language=409", L"", &cpToken)));
  VERIFY(SUCCEEDED(pVoice->SetVoice(cpToken)));
  cpToken->Release();

  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }

  // Release TTS for this thread.
  pVoice->Release();
  pVoice = nullptr;
  CoUninitialize();
}

void SoundPlayer::playDirect(const std::string& basename)
{
  playing = true;
  std::string filePath(filePrefix);
  filePath += basename;
  PlaySound(filePath.c_str(), nullptr, SND_SYNC | SND_FILENAME);
  playing = false;
}

void SoundPlayer::sayDirect(const std::string& text)
{
  playing = true;

  // Convert string to PCWSTR
  const int wchars_num = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, NULL, 0);
  wchar_t* pcwstr = new wchar_t[wchars_num];
  MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, pcwstr, wchars_num);

  // Speak async
  const HRESULT& hr = pVoice->Speak(pcwstr, SPF_ASYNC, NULL);
  ASSERT(SUCCEEDED(hr));
  delete[] pcwstr;

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

    if(!first.empty() && first[0] == ':')
      sayDirect(std::string(first.c_str() + 1));
    else
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
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
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
