/**
 * @file SoundFileTester.cpp
 *
 * This file implements a module that plays sound files.
 *
 * @author Tim Laue
 */

#include "SoundFileTester.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Modify.h"

MAKE_MODULE(SoundFileTester, infrastructure)

void SoundFileTester::update(DummyRepresentation& dummyRepresentation)
{
  std::string fileName = "";
  MODIFY("module:SoundFileTester:fileName", fileName);
  DEBUG_RESPONSE_ONCE("module:SoundFileTester:play")
  {
    if(fileName != "")
      SystemCall::playSound(fileName.c_str());
  }
  dummyRepresentation.dummy = 1;
}
