/**
 * @file Plot.h
 */

#pragma once

#include "Debugging/Debugging.h"
#include "Streaming/Output.h"

#if !defined TARGET_ROBOT || !defined NDEBUG

/**
 * A macro that declares a pollable plot.
 * @param id The name of the plot.
 */
#define DECLARE_PLOT(id) \
  DECLARE_DEBUG_RESPONSE("plot:" id)

/**
 * A macro that plots a value.
 * These values are collected and plotted over time.
 * @param id The name of the plot.
 * @param value The value to be plotted.
 */
#define PLOT(id, value) \
  do \
    DEBUG_RESPONSE("plot:" id) OUTPUT(idPlot, bin, id << static_cast<float>(value)); \
  while(false)

#else
//Ignore everything
#define DECLARE_PLOT(id) static_cast<void>(0)
#define PLOT(id, value) static_cast<void>(0)
#endif
