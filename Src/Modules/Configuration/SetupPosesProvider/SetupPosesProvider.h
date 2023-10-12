/**
 * @file SetupPosesProvider.h
 *
 * This file declares a module that provides the poses of the robots before
 * entering the field. Poses are based on setupPoses.cfg configuration file.
 * However, if any robot with a number > 7 is in the starting team, the poses
 * are rearranged.
 * Default lineup is clockwise with ascending numbers, starting in the own left corner with the number 1:
 *
 * //  4        5  //  (edge of center circle)
 * //              //
 * //  3        6  //  (middle between 2/4 and 5/7)
 * //              //
 * //  2        7  //  (edge of penalty area)
 * //              //
 * //  1           //  (edge of goal area [not larger penalty area])
 * //   Own Goal   //
 *
 * If any robot is missing as it is a substitute (not because of a penalty!),
 * the other robots step up to fill the gap to to preserve the constraint of ascending numbers.
 * Example (robot 2 is a substitute and robot 9 is playing instead):
 *
 * //  5        6  //  (edge of center circle)
 * //              //
 * //  4        7  //  (middle between 2/4 and 5/7)
 * //              //
 * //  3        9  //  (edge of penalty area)
 * //              //
 * //  1           //  (edge of goal area [not larger penalty area])
 * //   Own Goal   //
 *
 * OK?
 *
 * @author Tim Laue
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/GameState.h"

MODULE(SetupPosesProvider,
{,
  REQUIRES(GameState),
  PROVIDES(SetupPoses),
  LOADS_PARAMETERS(
  {,
    (std::vector<SetupPoses::SetupPose>) poses, /**< A list of all available robot poses, not ordered by number */
  }),
});

class SetupPosesProvider : public SetupPosesProviderBase
{
private:
  std::vector<int> robotOrderFromGC;       /**< The numbers of the robots of which the GC claims that they are not substitutes. */
  bool representationHasBeenInitialized;   /**< Data has been written to the representation at least once, i.e. it is valid. */

  /** Updates the setup poses */
  void update(SetupPoses& setupPoses);

  /** Checks, if there are other substitutes that before.
   * @return true, if the active and substituted robots have changed.
   */
  bool updateRobotOrder();

public:
  /** Constructor */
  SetupPosesProvider();
};
