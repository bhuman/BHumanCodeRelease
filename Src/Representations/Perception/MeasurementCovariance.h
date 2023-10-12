/**
 * @file MeasurementCovariance.h
 *
 * This file defines a representation that allows to compute a covariance matrix
 * given some information about a measurement.
 *
 * This functionality is implemented via a representation to allow for / experiment with different
 * ways of computing proper covariance matrices for percepts.
 *
 * @author Tim Laue
 * @author Yannik Meinken
 */

#pragma once

#include "Streaming/Function.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"


STREAMABLE(MeasurementCovariance,
           {
             /**
              * A function that computes a 2D covariance matrix for a robot-relative position on the floor.
              * @param p A 2D position in robot-relative coordinates.
              * @return A 2D covariance Matrix.
              */
             FUNCTION(Matrix2f(
             const Vector2f& p)) computeForRelativePosition;

             /**
              * A function that transforms a point in the image to robot-relative coordinates and computes a covariance matrix for the converted point
              * @param inImage A 2D position in image coordinates.
              * @param z The height of the horizontal plane (relative to the ground)
              * @param inRobot The resulting transformed point in 2D robot-relative coordinates
              * @param covariance The resulting 2D covariance matrix for the robot-relative point
              * @return true if the result is valid
              */
             FUNCTION(bool(
             const Vector2f& inImage,
             float z, Vector2f
             &inRobot, Matrix2f & covariance)) transformWithCov;

             /**
              * A function that transforms a point in the image to robot-relative coordinates and computes a covariance matrix for the converted point
              * Legacy support for setting the rotationDeviation manually, remove with legacyMeasurementCovarianceProvider
              * @param inImage A 2D position in image coordinates.
              * @param z The height of the horizontal plane (relative to the ground)
              * @param rotationDeviation The robot's current rotation deviation
              * @param inRobot The resulting transformed point in 2D robot-relative coordinates
              * @param covariance The resulting 2D covariance matrix for the robot-relative point
              * @return true if the result is valid
              */
             FUNCTION(bool(
             const Vector2f& inImage,
             float z,
             const Vector2f& rotationDeviation, Vector2f
             &inRobot, Matrix2f & covariance)) transformWithCovLegacy,
           });
