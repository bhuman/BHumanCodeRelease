/**
 * @file Modules/Infrastructure/JointCalibrator.h
 * This file declares a module with tools for calibrating leg joints.
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Eigen.h"

MODULE(JointCalibrator,
{,
  USES(JointAngles),
  USES(JointRequest),
  REQUIRES(RobotDimensions),
  REQUIRES(MassCalibration),
  PROVIDES(JointCalibration),
});

class JointCalibrator : public JointCalibratorBase
{
public:
  /**
   * Sets the angles of the bodyRotationCorrection of the CameraCalibration into the offsets.
   * @param x angle correction, y angle correction
   */
  static void setOffsets(Angle x, Angle y);

private:
  /**
   * An offset which is added to the target of the inverse kinematics to detemermine the calibration offset for each joint angle.
   */
  STREAMABLE(Offset,
  {
    bool operator!=(const Offset& other) const
    {
      return translation != other.translation || rotation != other.rotation;
    }

    Offset& operator-=(const Offset& other)
    {
      translation -= other.translation;
      rotation -= other.rotation;
      return *this;
    }

    void clear()
    {
      translation = Vector3f::Zero();
      rotation = Vector3a::Zero();
    },

    (Vector3f)(Vector3f::Zero()) translation,
    (Vector3a)(Vector3a::Zero()) rotation,
  });

  /**
   * Calibration offsets for each foot.
   */
  STREAMABLE(Offsets,
  {
    bool operator!=(const Offsets& other) const
    {
      return bodyRotation != other.bodyRotation || leftFoot != other.leftFoot || rightFoot != other.rightFoot;
    }

    Offsets& operator-=(const Offsets& other)
    {
      bodyRotation -= other.bodyRotation;
      leftFoot -= other.leftFoot;
      rightFoot -= other.rightFoot;
      return *this;
    }

    void clear()
    {
      bodyRotation = Vector2a::Zero();
      leftFoot.clear();
      rightFoot.clear();
    },

    (Vector2a)(Vector2a::Zero()) bodyRotation,
    (Offset) leftFoot,
    (Offset) rightFoot,
  });

  Offsets offsets;
  Offsets lastOffsets;
  JointCalibration original;

  /**
   * The central update method to provide the calibration
   * @param jointCalibration The jointCalibration
   */
  void update(JointCalibration& jointCalibration) override;

  static void streamOffsets(const std::string representationName, const Streamable& representation);
};
