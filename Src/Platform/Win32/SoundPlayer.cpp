/**
* @file Platform/Win32/SoundPlayer.cpp
* Implementation of class SoundPlayer.
* @attention this is the Win32 implementation
* @author Colin Graf
*/

#include <windows.h>
#include <mmsystem.h>

#include "SoundPlayer.h"
#include "Platform/File.h"

int SoundPlayer::play(const std::string& name)
{
  std::string filePath(File::getBHDir());
  filePath += "/Config/Sounds/";
  filePath += name;
  PlaySound(filePath.c_str(), NULL, SND_ASYNC | SND_FILENAME);
  return 1;
}

