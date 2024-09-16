/**
 * @file InertialSensorDataProvider.h
 *
 * This file implements a module that applies the calibration to the measured IMU data and annotates it with covariance matrices
 *
 * @author Yannik Meinken
 */

#include "Debugging/Annotation.h"
#include "InertialSensorDataProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(InertialSensorDataProvider);

void InertialSensorDataProvider::update(InertialSensorData& inertialSensorData)
{
  inertialSensorData.angle = theRawInertialSensorData.angle;

  if(theMotionRobotHealth.motionFramesDropped)
  {
    inertialSensorData.newAccData = false;
    inertialSensorData.newGyroData = false;

    lastNewRawAccData = Vector3f::Zero();
    lastNewRawGyroData = Vector3a::Zero();

    accChange = Vector3f::Ones() * initialAccChange;
    gyroChange = Vector3a::Ones() * initialGyroChange;
  }

  bool orderKnown = true;
  //We don't now the order yet
  if(!inertialSensorData.newAccData && !inertialSensorData.newGyroData)
  {
    const bool notFirstFrame = !(lastNewRawAccData == Vector3f::Zero() && lastNewRawGyroData == Vector3a::Zero());
    //True if not the first frame and the data changed
    inertialSensorData.newAccData = notFirstFrame && lastNewRawAccData != theRawInertialSensorData.acc;
    inertialSensorData.newGyroData = notFirstFrame && lastNewRawGyroData != theRawInertialSensorData.gyro;
    if(notFirstFrame && (inertialSensorData.newAccData || inertialSensorData.newGyroData) && SystemCall::getMode() != SystemCall::Mode::simulatedRobot)
      ASSERT(inertialSensorData.newAccData != inertialSensorData.newGyroData);

    orderKnown = inertialSensorData.newAccData || inertialSensorData.newGyroData;
  }
  else
  {
    // if we already know which data was new last frame -> this frame it is the other one
    inertialSensorData.newAccData = !inertialSensorData.newAccData;
    inertialSensorData.newGyroData = !inertialSensorData.newGyroData;
  }

  if(orderKnown)
  {
    // Compute change
    accChange = (lastNewRawAccData - theRawInertialSensorData.acc).cwiseAbs();
    ASSERT(accChange(0) >= 0);
    ASSERT(accChange(1) >= 0);
    ASSERT(accChange(2) >= 0);

    gyroChange = (lastNewRawGyroData - theRawInertialSensorData.gyro).cast<float>().cwiseAbs().cast<Angle>();
    ASSERT(gyroChange(0) >= 0);
    ASSERT(gyroChange(1) >= 0);
    ASSERT(gyroChange(2) >= 0);

    // Unexpected order (but not a skipped motion frame -> the new value is actually new)
    if(SystemCall::getMode() != SystemCall::Mode::logFileReplay &&
       ((inertialSensorData.newAccData && lastNewRawGyroData != theRawInertialSensorData.gyro) ||
        (inertialSensorData.newGyroData && lastNewRawAccData != theRawInertialSensorData.acc) ||
        inertialSensorData.newGyroData == inertialSensorData.newAccData)) //Simulation
    {
      // Determine which data is new
      inertialSensorData.newAccData = lastNewRawAccData != theRawInertialSensorData.acc;
      inertialSensorData.newGyroData = lastNewRawGyroData != theRawInertialSensorData.gyro;

      //the order of motion frames was wrong so set the change to a big value
      if(!inertialSensorData.newAccData)
        accChange = Vector3f::Ones() * initialAccChange;
      if(!inertialSensorData.newGyroData)
        gyroChange = Vector3a::Ones() * initialGyroChange;
    }
  }
  lastNewRawAccData = theRawInertialSensorData.acc;
  lastNewRawGyroData = theRawInertialSensorData.gyro;

  // Calibration
  Matrix3f rotation(theIMUCalibration.isCalibrated ? Matrix3f(theIMUCalibration.rotation) : Matrix3f::Identity()); //Rotation matrix
  inertialSensorData.acc = rotation * theRawInertialSensorData.acc;
  inertialSensorData.gyro = theRawInertialSensorData.gyro.cast<float>().cwiseProduct(theIMUCalibration.gyroFactor).cast<Angle>();
  inertialSensorData.gyro = (rotation * inertialSensorData.gyro.cast<float>()).cast<Angle>();

  // Covariance
  inertialSensorData.accCovariance = (Vector3f::Ones() * accVariance).asDiagonal();
  if(!inertialSensorData.newAccData)
    inertialSensorData.accCovariance += overestimateVarianceOfExtrapolationFactor *
                                        (baseAccExtrapolationVariance + factorAccExtrapolationVariance.cwiseProduct(accChange)).asDiagonal();
  inertialSensorData.accCovariance = rotation * inertialSensorData.accCovariance * rotation.transpose(); // Rotate Matrix (RMR⁻¹)

  inertialSensorData.gyroCovariance = (Vector3a::Ones() * gyroVariance).asDiagonal();
  if(!inertialSensorData.newGyroData)
    inertialSensorData.gyroCovariance += overestimateVarianceOfExtrapolationFactor *
                                         (baseGyroExtrapolationVariance + factorGyroExtrapolationVariance.cwiseProduct(gyroChange)).asDiagonal();
  inertialSensorData.gyroCovariance = (rotation * inertialSensorData.gyroCovariance.cast<float>() * rotation.transpose()).cast<Angle>();

  for(size_t i = 0; i < 3; i++)
  {
    ASSERT(inertialSensorData.gyroCovariance(i, i) >= 0);
    ASSERT(inertialSensorData.accCovariance(i, i) >= 0);
  }
}
