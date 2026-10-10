#include "InertialDataProvider.h"
#include "Debugging/Plot.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"
#include "Math/RotationMatrix.h"

#include <cmath>

MAKE_MODULE(InertialDataProvider);

void InertialDataProvider::update(InertialData& inertialData)
{
  const Quaternionf rot = Rotation::Euler::fromAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), theInertialSensorData.angle.z());

  inertialData.acc = theInertialSensorData.acc;
  inertialData.gyro = theInertialSensorData.gyro;

  inertialData.orientation2D = Rotation::removeZRotation(rot);
  inertialData.orientation3D = rot;
  inertialData.angleChange = inertialData.angle.head<2>();
  if(theMotionInfo.executedPhase == MotionPhase::stand && theCalibrationRequest.targetState != CameraCalibrationStatus::State::idle)
    inertialData.angle << theInertialSensorData.angle;
  else
    inertialData.angle << Rotation::AngleAxis::pack(AngleAxisf(inertialData.orientation2D)).head<2>().cast<Angle>(), 0_deg;
  inertialData.angleChange = inertialData.angle.head<2>() - inertialData.angleChange;

  const Vector3a angleAxisVec = Rotation::AngleAxis::pack(AngleAxisf(rot)).cast<Angle>();
  PLOT("module:InertialDataProvider:internalOrientation:x", angleAxisVec.x().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:y", angleAxisVec.y().toDegrees());
  PLOT("module:InertialDataProvider:internalOrientation:z", angleAxisVec.z().toDegrees());

  const RotationMatrix diff = rot.inverse() * RotationMatrix(Rotation::Euler::fromAngles(theInertialSensorData.angle));
  PLOT("module:InertialDataProvider:diffToAngle", diff.getAngleAxis().angle());
}
