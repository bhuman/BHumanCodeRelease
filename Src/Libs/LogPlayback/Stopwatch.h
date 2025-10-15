/**
 * @file Stopwatch.h
 *
 * This file declares a struct that wraps \c idStopwatch messages.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/Streamable.h"

/**
 * Please note that \c namesToIndices usually only contains three
 * entries, i.e. multiple messages have to be read to be able to build a
 * complete map.
 */
struct Stopwatch : public Streamable
{
  std::unordered_map<std::string, unsigned short> namesToIndices; /**< Map stopwatch names to indices into \c durations . */
  std::vector<unsigned> durations; /**< The durations in µs. */
  unsigned frameStartTime; /**< The time when the frame started in ms. */
  unsigned frameNumber; /**< The number of the frame. */

protected:
  /**
   * Read object from a stream.
   * @param stream The stream to read from.
   */
  void read(In&) override;

  /**
   * Write object to a stream.
   * @param stream The stream to write to.
   */
  void write(Out&) const override;

private:
  /** Registers an incomplete type for this struct, because streaming is non-standard. */
  static void reg();
};
