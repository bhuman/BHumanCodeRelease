/**
 * @file Calibration.h
 *
 * This file declares complex skills related to robot calibration.
 *
 * @author Thomas Röfer
 */

/** Lets the robot start the manual camera calibration. */
option(ManualCameraCalibration);

/** This skill can be used to calibrate the robot. */
option(CalibrateRobot, args((const CalibrationRequest&) request));

/** Lets the robot start the calibration process. */
option(CalibrationControl);
