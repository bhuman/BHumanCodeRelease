/**
* @file Platform/Windows/SoundPlayer.cpp
* Implementation of class SoundPlayer.
* @attention this is the Windows implementation
* @author Colin Graf
*/

#define NOMINMAX
#include <windows.h>

#include "SoundPlayer.h"
#include "Platform/File.h"

int SoundPlayer::play(const std::string& name)
{
  std::string filePath(File::getBHDir());
  filePath += "/Config/Sounds/";
  filePath += name;
  PlaySound(filePath.c_str(), nullptr, SND_ASYNC | SND_FILENAME);
  return 1;
}
