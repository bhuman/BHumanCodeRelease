/**
 * @file CameraCalibrationStatus.h
 *
 * This file defines a representation that represent the current state of the camera calibration.
 *
 * @author Lukas Plecher
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

/** This enumeration describes the states that this module can be in. */
ENUM(State,
{,
  idle, /**< Nothing special is done. */
  recordSamples, /**< Samples are constructed from observations. */
  optimize, /**< The optimization is running (one iteration per frame). */
});

ENUM(SampleConfigurationStatus,
{,
  none, /**< There is no active SampleConfigurationRequest. */
  recording, /**< The calibrator is currently recording samples. */
  visible, /**< All required samples are visible, but the calibrator is not configured to record them. */
  notVisible, /**< Some required SampleTypes are not visible. */
  finished, /**< The request SampleConfiguration has finished recording. */
});

STREAMABLE(CameraCalibrationStatus,
{,
  (State)(State::idle) state,
  (int)(0) inStateSince,
  (SampleConfigurationStatus)(SampleConfigurationStatus::none) sampleConfigurationStatus,
});
