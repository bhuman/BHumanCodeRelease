/*
 * FootContactModelProvider.h
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/GameInfo.h"

/** Number of contacts to buffer. 100 complies to 1 second */
#define BUFFER_SIZE 100

MODULE(FootContactModelProvider)
  REQUIRES(MotionInfo)
  REQUIRES(FallDownState)
  REQUIRES(FrameInfo)
  REQUIRES(KeyStates)
  REQUIRES(GameInfo)
  REQUIRES(DamageConfiguration)
  PROVIDES_WITH_MODIFY(FootContactModel)
  DEFINES_PARAMETER(bool, debug, false)             /**< enables debug mode (debug sound) */
  DEFINES_PARAMETER(int, contactThreshold, 15)      /**< threshold in contacts per second to determine foot contact */
  DEFINES_PARAMETER(int, malfunctionThreshold, 250) /**< threshold in Motion frames of contact after a malfunction of a bumper is detected (2.5 seconds) */
  DEFINES_PARAMETER(unsigned, soundDelay, 1000)     /**< Delay between debug sounds. */
END_MODULE

/**
 * Provides information about foot contacts :)
 */
class FootContactModelProvider : public FootContactModelProviderBase
{
public:
  FootContactModelProvider();

private:
  void update(FootContactModel& model);

  /**
   * Buffers the contacts over the last frames.
   * Each frame one value is added to this buffer.
   * A 1 if there was a foot contact in the frame, 0 otherwise.
   */
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferLeft;
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferRight;

  int contactDurationLeft;                             /**< the duration of the last contact. Will be reset to 0 if contact is lost */
  int contactDurationRight;                            /**< the duration of the last contact. Will be reset to 0 if contact is lost */

  int leftFootLeftDuration;                            /**< duration of consecutive contact detections for each bumper */
  int leftFootRightDuration;
  int rightFootLeftDuration;
  int rightFootRightDuration;


  /**
  * Checks for contact of a bumper. Additionally, if a bumper is pressed longer than the
  * malfunction threshold, it is ignored (i.e. this method returns false).
  *
  * @param key The key to check.
  * @param duration The duration how long this key has been pressed. This value will be incremented by
  *     this method (if bumper has contact) or reset to 0 (if bumper has no contact)
  * @returns true if the key is pressed and its pressed duration is not longer than the malfunctionThreshold.
  */
  bool checkContact(KeyStates::Key key, int& duration);
};
