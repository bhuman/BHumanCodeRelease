/**
 * @file SideInformationProvider.h
 *
 * Declaration of the module SideInformationProvider.
 * The SPL field is symmetric and perceiving the field elements alone does not
 * allow to infer the correct playing direction.
 * This module computes some information
 * that might help to solve this problem in some situations.
 * One part is to compute, if a robot MUST be inside its ow half given the walked distance and the current rules of the game.
 * The other part (a flip detection based on observations) is currently removed.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/SideInformation.h"
#include "Framework/Module.h"

MODULE(SideInformationProvider,
{,
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(Odometer),
  REQUIRES(SetupPoses),
  REQUIRES(StaticInitialPose),
  PROVIDES(SideInformation),
  LOADS_PARAMETERS(
  {,
    (float) distanceUncertaintyOffset,     /**< Estimated odometry and localization base error (mm). */
    (float) distanceUncertaintyFactor,     /**< Estimated odometry error as a factor of the distance walked. */
    (float) awayFromLineDistance,          /**< The distance the robot has to be before or behind a certain line (mm). */
    (bool)  forceOwnHalf,                  /**< Robot is always in the own half (due do some special circumstances, like a demo or a technical challenge) */
    (bool)  forceOpponentHalf,             /**< Robot is always in the opponent half (due do some special circumstances, like a demo or a technical challenge) */
  }),
});

/**
* @class SideInformationProvider
*
* Computation of information that might help to overcome some field symmetry problems.
*/
class SideInformationProvider : public SideInformationProviderBase
{
public:
  /** Constructor */
  SideInformationProvider();

private:
  float distanceWalkedAtKnownPosition;                  /**< The robot walked this far at its last known position. */
  float largestXPossibleAtKnownPosition;                /**< The largest x coordinate possible at its last known position. */

  /** Checks, how far into the opponent half the robot could have walked after it entered the game.
   *  Furthermore, it is checked, if the robot is probably still inside the own half of the field.
   *  @param SideInformation The representation of which some fields are filled by this function
   */
  void update(SideInformation& sideInformation) override;
};
