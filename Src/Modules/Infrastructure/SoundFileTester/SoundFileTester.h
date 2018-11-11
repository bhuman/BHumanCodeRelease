/**
 * @file SoundFileTester.h
 *
 * This file declares a module that plays sound files
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/DummyRepresentation.h"

MODULE(SoundFileTester,
{,
  PROVIDES(DummyRepresentation),
});

class SoundFileTester : public SoundFileTesterBase
{
private:
  void update(DummyRepresentation& dummyRepresentation) override;
};
