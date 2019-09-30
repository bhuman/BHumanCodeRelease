/**
 * @file BehaviorContext.h
 *
 * This file declares a struct that stores common context of a behavior (card or skill).
 *
 * @author Arne Hasselbring
 */

#pragma once

struct BehaviorContext
{
  unsigned lastFrame = 1; /**< The timestamp of the last frame in which this option was executed. */
  unsigned behaviorStart = 0; /**< The time when the behavior (card or skill) started to run. */
  unsigned stateStart = 0; /**< The time when the current state started to run. */
  const char* stateName = nullptr; /**< The name of the current state. */
};
