/*
 * @file SoundSignalRecognizerDummy.h
 *
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/SoundSignal.h"

MODULE(SoundSignalRecognizerDummy,
{,
REQUIRES(FrameInfo),
PROVIDES_WITH_MODIFY(SoundSignal),
});

/*
* @class SoundSignalRecognizer
*
* Module that identifies signals in audio data
*/
class SoundSignalRecognizerDummy : public SoundSignalRecognizerDummyBase
{
  /**
  * The method that detects the signals
  *
  * @param SoundSignal The identified signal
  */
  void update(SoundSignal& soundSignal);
};
