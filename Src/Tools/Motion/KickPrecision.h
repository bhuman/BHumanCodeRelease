/**
 * @file KickPrecision.h
 * This file contains information about the precision types of the kicks
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Streaming/Enum.h"

ENUM(KickPrecision,
{,
  precise, /**< As precise as the kick can get. */
  notPrecise, /**< Some inaccuracy is allowed. */
  justHitTheBall, /**< Throw all precision out of the window and just touch the ball. */
});
