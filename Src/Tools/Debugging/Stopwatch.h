/**
* @file Stopwatch.h
* The file declares the stopwatch macros.
* @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
*/

#pragma once

#include "TimingManager.h"
#include "DebugDrawings.h"

/*
 * Allows the measurement the execution time of an expression.
 * @param eventID The id of the stop watch.
 * @param ... The expression of which the execution time is measured.
 */
#define STOP_TIME_ON_REQUEST(eventID, ...) \
  _STOP_TIME_ON_REQUEST(eventID, (void) time, __VA_ARGS__)

/*
 * Allows the measurement the execution time of an expression and plot the measurements.
 * @param eventID The id of the stop watch.
 * @param ... The expression of which the execution time is measured.
 */
#define STOP_TIME_ON_REQUEST_WITH_PLOT(eventID, ...) \
  _STOP_TIME_ON_REQUEST(eventID, PLOT("stopwatch:" eventID, time * 0.001f), __VA_ARGS__)

/*
 * Private macro for measuring the execution time of an expression.
 * @param eventID The id of the stop watch.
 * @param plot Extra code that might output plot data.
 * @param ... The expression of which the execution time is measured.
 */
#define _STOP_TIME_ON_REQUEST(eventID, plot, ...) \
  do \
  { \
    TimingManager& _tm = Global::getTimingManager(); \
    _tm.startTiming(eventID); \
    { __VA_ARGS__ } \
    const unsigned time = _tm.stopTiming(eventID); \
    plot; \
  } \
  while(false)
