/**
 * @file CalibrationRequest.h
 *
 * This file defines a representation that represent a request to start or abort the camera calibration.
 *
 * @author Lukas Plecher
 */

#pragma once

#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

/** This enumeration lists the possible types of samples. */
ENUM(SampleType,
{,
  cornerAngle, /**< An angle in the penalty area that must be 90 degrees. */
  parallelAngle, /**< An angle in the penalty area that must be 180 degrees. */
  parallelLinesDistance, /**< The (known) distance between the goal line and the front goal area line. */
  goalAreaDistance, /**< The (known) distance between the penalty mark and the front goal area line. */
  goalLineDistance, /**< The (known) distance between the penalty mark and the goal line. */
});

STREAMABLE(SampleConfigurationRequest,
{,
  (bool)(false) doRecord, /**< Actually record seen samples? */
  (unsigned)(0) index,
  (CameraInfo::Camera)(CameraInfo::Camera::upper) camera,
  (Angle)(0) headPan,
  (Angle)(0) headTilt,
  (unsigned)(0) sampleTypes,
});

STREAMABLE(CalibrationRequest,
{,
  (CameraCalibrationStatus::State)(CameraCalibrationStatus::State::idle) targetState,
  (unsigned)(0) totalNumOfSamples,
  (std::optional<SampleConfigurationRequest>)() sampleConfigurationRequest,
  (unsigned)(0) serialNumberIMUCalibration, /**< Current iteration, how often the IMU calibration was requested to be calibrated. */
  (bool)(false) preciseJointPositions, /**< Instead of an energy saving mode, the robot should reach the exact target joint positions for the legs. */
  (unsigned)(0) numOfFootSoleCalibration, /**< Average over this number of calibrations tries the foot sole rotation offset. */
});
