/**
 * @file Stopwatch.h
 * The file declares the stopwatch macros.
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 */

#pragma once

#include "TimingManager.h"
#include "Debugging.h"

/**
 * Helper function to declare a plot as a single statement.
 * @param id The id of the plot.
 * @param value The time to plot in ms.
 */
inline void _plot(const char* id, unsigned time)
{
  DEBUG_RESPONSE(id) OUTPUT(idPlot, bin, (id + 5) << static_cast<float>(time) * 0.001f);
}

/**
 * Allows the measurement the execution time of the following block.
 * @param eventID The id of the stop watch.
 */
#define STOPWATCH(eventID) \
  for(bool _start = true; (_start ? Global::getTimingManager().startTiming(eventID) : (void) Global::getTimingManager().stopTiming(eventID)), _start; _start ^= true)

/**
 * Allows the measurement the execution time of the following block and plot the measurements.
 * @param eventID The id of the stop watch.
 */
#define STOPWATCH_WITH_PLOT(eventID) \
  for(bool _start = true; (_start ? Global::getTimingManager().startTiming(eventID) : _plot("plot:stopwatch:" eventID, Global::getTimingManager().stopTiming(eventID))), _start; _start ^= true)
