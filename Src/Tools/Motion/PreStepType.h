/**
 * @file PreStepType.h
 * This file contains information about whether a pre step is allowed or not for an in walk kick
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Streaming/Enum.h"

ENUM(PreStepType,
{,
  allowed, /**< A pre step is allowed, but can also be skipped. */
  notAllowed, /**< A pre step is not allowed. */
  forced, /**< A pre step is forced (if the kick defined one). */
});
