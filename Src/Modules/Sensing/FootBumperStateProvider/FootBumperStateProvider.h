/**
 * FootBumperStateProvider.h
 *
 *  Created on: Mar 14, 2012
 *      Author: arne
 *              simont@tzi.de
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Math/Constants.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

/** Number of contacts to buffer. 83 complies to 1 second */
#define BUFFER_SIZE static_cast<int>(1.f / Constants::motionCycleTime)

MODULE(FootBumperStateProvider,
{,
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(KeyStates),
  REQUIRES(MotionInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(FootBumperState),
  DEFINES_PARAMETERS(
  {,
    (float)(6.6666f) contactThreshold,           /**< Factor to get 12.5 detections on the V6 */
    (int)(250) malfunctionThreshold,             /**< threshold in Motion frames of contact after a malfunction of a bumper is detected (2.5 seconds) */
    (float)(11.f) distanceBetweenFootBumpers,    /**< Inner foot bumper edges must have at least this distance, otherwise a contact is ignored. */
    (Vector2f)(Vector2f(-10.f, 0)) lowerShift,   /**< Shift inner foot bumper edges this much for the lower edge. */
    (float)(-25.f) additionalLowerShift,         /**< Shift inner foot bumper edges this much additional for the lower edge. */
    (Vector2f)(Vector2f(5.f, -8.f)) upperShift,  /**< Shift inner foot bumper edges this much for the upper edge. */
  }),
});

/**
 * Provides information about foot contacts :)
 */
class FootBumperStateProvider : public FootBumperStateProviderBase
{
  /**
   * Buffers the contacts over the last frames.
   * Each frame one value is added to this buffer.
   * A 1 if there was a foot contact in the frame, 0 otherwise.
   */
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferLeft;
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferRight;

  RingBufferWithSum<int, BUFFER_SIZE> contactBufferLeftLeft;
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferLeftRight;
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferRightLeft;
  RingBufferWithSum<int, BUFFER_SIZE> contactBufferRightRight;

  int contactDurationLeft = 0; /**< the duration of the last contact. Will be reset to 0 if contact is lost */
  int contactDurationRight = 0; /**< the duration of the last contact. Will be reset to 0 if contact is lost */

  int leftFootLeftDuration = 0; /**< duration of consecutive contact detections for each bumper */
  int leftFootRightDuration = 0;
  int rightFootLeftDuration = 0;
  int rightFootRightDuration = 0;

  void update(FootBumperState& footBumperState) override;

  /**
   * Checks for contact of a bumper. Additionally, if a bumper is pressed longer than the
   * malfunction threshold, it is ignored (i.e. this method returns false).
   *
   * @param key The key to check.
   * @param duration The duration how long this key has been pressed. This value will be incremented by
   *     this method (if bumper has contact) or reset to 0 (if bumper has no contact)
   * @return true if the key is pressed and its pressed duration is not longer than the malfunctionThreshold.
   */
  bool checkContact(KeyStates::Key key, int& duration);

  /**
   * Checks if the feet are too close together. In such a case, we assume the inner bumpers are touching each other and should be ignored.
   */
  bool ignoreContact();
};
