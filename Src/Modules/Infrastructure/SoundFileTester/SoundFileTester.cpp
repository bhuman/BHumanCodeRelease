/**
 * @file SoundFileTester.cpp
 *
 * This file implements a module that plays sound files.
 *
 * @author Tim Laue
 */

#include "SoundFileTester.h"
#include "Platform/SystemCall.h"
#include "Debugging/Modify.h"

MAKE_MODULE(SoundFileTester);

void SoundFileTester::update(DummyRepresentation& dummyRepresentation)
{
  std::string fileName = "";
  MODIFY("module:SoundFileTester:fileName", fileName);
  DEBUG_RESPONSE_ONCE("module:SoundFileTester:play")
  {
    if(fileName != "")
      SystemCall::playSound(fileName.c_str());
  }
  std::string text = "";
  MODIFY("module:SoundFileTester:text", text);
  DEBUG_RESPONSE_ONCE("module:SoundFileTester:say")
  {
    if(text != "")
      SystemCall::say(text.c_str());
  }
  dummyRepresentation.dummy = 1;
}
