/**
 * @file Stopwatch.h
 * The file declares the stopwatch macros.
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author Thomas Röfer
 * @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 */

#pragma once

#include "TimingManager.h"
#include "Debugging.h"

/** A stopwatch that measures the time an instance of it lives and plots it. */
class Stopwatch
{
  const char* const name; /**< The name of the plot. */
  bool running = true; /**< Should the stopwatch still be running? */

public:
  /**
   * Start the stopwatch.
   * @param name The name of the plot.
   */
  Stopwatch(const char* name) : name(name) {Global::getTimingManager().startTiming(name + 15);}

  /** Stop the stopwatch.*/
  ~Stopwatch()
  {
    const unsigned time = Global::getTimingManager().stopTiming(name + 15);
    static_cast<void>(time);
    DEBUG_RESPONSE(name)
      OUTPUT(idPlot, bin, (name + 5) << static_cast<float>(time) * 0.001f);
  }

  /**< Should the stopwatch still be running? */
  bool isRunning() {return !(running ^= true);}
};

/**
 * Allows the measurement the execution time of the following block and plot the measurements.
 * @param name The name of the stopwatch.
 */
#define STOPWATCH(name) \
  for(Stopwatch _stopwatch("plot:stopwatch:" name); _stopwatch.isRunning();)
