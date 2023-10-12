/**
 * @file LowPassFilterPR.h
 *
 * This is a filter, which uses context of the last measurements to filter less or stronger.
 * This allows to "believe" the measurement more, if the changes of the measurements support the current measurement.
 * But filters stronger, if the changes do not support the current measurement.
 *
 * @author Philip Reichenberg
 */

#include "Math/RingBuffer.h"

#pragma once

struct LowPassFilterPR
{
  /**
   * Buffered measurements
   */
  RingBuffer<float, 4> buffer;

  /**
   * The current filtered value
   */
  float currentValue = 0.f;

  /**
   * The normal low pass filter parameter
   */
  float lowPassFactor = 0.f;

  /**
   * The "more believe" low pass filter parameter
   */
  float fastFactor = 0.f;

  LowPassFilterPR(const float lowPassFactor, const float fastFactor) :
    lowPassFactor(lowPassFactor),
    fastFactor(fastFactor) {};

  LowPassFilterPR() = default;

  void update(const float value);

  void clear();
};
