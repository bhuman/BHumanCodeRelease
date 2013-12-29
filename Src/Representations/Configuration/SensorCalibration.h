/**
* @file SensorCalibration.h
* Declaration of a class for representing the calibration values of sensors.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SensorCalibration,
{,
  (float)(0) accXOffset, /**< The correction offset in g. */
  (float)(1.f) accXGain, /**< The factor between sensor units and g. */
  (float)(0) accYOffset, /**< The correction offset in g. */
  (float)(1.f) accYGain, /**< The factor between sensor units and g. */
  (float)(0) accZOffset, /**< The correction offset in g. */
  (float)(1.f) accZGain, /**< The factor between sensor units and g. */
  (float)(1.f) gyroXGain, /**< The factor between sensor units and g. */
  (float)(1.f) gyroYGain, /**< The factor between sensor units and g. */
  (float)(0) fsrLFLOffset,
  (float)(1.f) fsrLFLGain,
  (float)(0) fsrLFROffset,
  (float)(1.f) fsrLFRGain,
  (float)(0) fsrLBLOffset,
  (float)(1.f) fsrLBLGain,
  (float)(0) fsrLBROffset,
  (float)(1.f) fsrLBRGain,
  (float)(0) fsrRFLOffset,
  (float)(1.f) fsrRFLGain,
  (float)(0) fsrRFROffset,
  (float)(1.f) fsrRFRGain,
  (float)(0) fsrRBLOffset,
  (float)(1.f) fsrRBLGain,
  (float)(0) fsrRBROffset,
  (float)(1.f) fsrRBRGain,
});
