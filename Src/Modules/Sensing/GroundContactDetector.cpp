/**
* @file GroundContactDetector.cpp
* Implementation of module GroundContactDetector.
* @author Colin Graf
*/

#include "GroundContactDetector.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT
#include "Tools/Streams/InStreams.h"

MAKE_MODULE(GroundContactDetector, Sensing)

GroundContactDetector::GroundContactDetector() : contact(false), contactStartTime(0), useAngle(false)
{
}

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  DECLARE_PLOT("module:GroundContactDetector:angleNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:angleNoiseY");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseY");
  DECLARE_PLOT("module:GroundContactDetector:accNoiseZ");
  DECLARE_PLOT("module:GroundContactDetector:gyroNoiseX");
  DECLARE_PLOT("module:GroundContactDetector:gyroNoiseY");

  MODIFY("module:GroundContactDetector:contact", contact);
  bool ignoreSensors = (theMotionInfo.motion != MotionRequest::walk && theMotionInfo.motion != MotionRequest::stand &&
                        theMotionInfo.motion != MotionRequest::specialAction && theMotionInfo.motion != MotionRequest::getUp) ||
                       (theMotionRequest.motion != MotionRequest::walk && theMotionRequest.motion != MotionRequest::stand &&
                        theMotionRequest.motion != MotionRequest::specialAction && theMotionRequest.motion != MotionRequest::getUp) ||
                       (theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.kickType != WalkRequest::none) ||
                       (theMotionRequest.motion == MotionRequest::walk && theMotionRequest.walkRequest.kickType != WalkRequest::none);
  if(!ignoreSensors)
  {
    if(contact)
    {
      calibratedAccZValues.add(theSensorData.data[SensorData::accZ]);

      Vector3<> angleDiff = ((const RotationMatrix&)(theTorsoMatrix.rotation * expectedRotationInv)).getAngleAxis();
      angleNoises.add(Vector2<>(sqr(angleDiff.x), sqr(angleDiff.y)));
      Vector2<> angleNoise = angleNoises.getAverage();
      PLOT("module:GroundContactDetector:angleNoiseX", angleNoise.x);
      PLOT("module:GroundContactDetector:angleNoiseY", angleNoise.y);

      if(!useAngle && angleNoises.isFull() && angleNoise.x < contactAngleActivationNoise && angleNoise.y < contactAngleActivationNoise)
        useAngle = true;

      if((useAngle && (angleNoise.x > contactMaxAngleNoise || angleNoise.y > contactMaxAngleNoise)) ||
         (calibratedAccZValues.isFull() && calibratedAccZValues.getAverage() > contactMaxAccZ))
      {
        /*
        if((useAngle && (angleNoise.x > p.contactMaxAngleNoise || angleNoise.y > p.contactMaxAngleNoise)))
          OUTPUT_ERROR("lost ground contact via angle");
        if((calibratedAccZValues.isFull() && calibratedAccZValues.getAverage() > p.contactMaxAccZ))
          OUTPUT_ERROR("lost ground contact via acc");
        */

        contact = false;
        accNoises.clear();
        gyroNoises.clear();
        accValues.clear();
        gyroValues.clear();
        angleNoises.clear();
#ifndef TARGET_SIM
        if(contactStartTime != 0)
          SystemCall::playSound("high.wav");
#endif
      }
    }
    else
    {
      const Vector3<> accAverage = accValues.getAverage();
      const Vector2<> gyroAverage = gyroValues.getAverage();
      const Vector2<> gyro = Vector2<>(theSensorData.data[SensorData::gyroX], theSensorData.data[SensorData::gyroY]);
      const Vector3<> acc = Vector3<>(theSensorData.data[SensorData::accX], theSensorData.data[SensorData::accY], theSensorData.data[SensorData::accZ]);
      accValues.add(acc);
      gyroValues.add(gyro);
      if(accValues.isFull())
      {
        accNoises.add(Vector3<>(sqr(acc.x - accAverage.x), sqr(acc.y - accAverage.y), sqr(acc.z - accAverage.z)));
        gyroNoises.add(Vector2<>(sqr(gyro.x - gyroAverage.x), sqr(gyro.y - gyroAverage.y)));
      }
      Vector3<> accNoise = accNoises.getAverage();
      Vector2<> gyroNoise = gyroNoises.getAverage();
      PLOT("module:GroundContactDetector:accNoiseX", accNoise.x);
      PLOT("module:GroundContactDetector:accNoiseY", accNoise.y);
      PLOT("module:GroundContactDetector:accNoiseZ", accNoise.z);
      PLOT("module:GroundContactDetector:gyroNoiseX", gyroNoise.x);
      PLOT("module:GroundContactDetector:gyroNoiseY", gyroNoise.y);

      if(accNoises.isFull() &&
         accAverage.z < -5.f && std::abs(accAverage.x) < 5.f && std::abs(accAverage.y) < 5.f &&
         accNoise.x < noContactMinAccNoise && accNoise.y < noContactMinAccNoise && accNoise.z < noContactMinAccNoise &&
         gyroNoise.x < noContactMinGyroNoise && gyroNoise.y < noContactMinGyroNoise)
      {
        contact = true;
        useAngle = false;
        contactStartTime = theFrameInfo.time;
        angleNoises.clear();
        calibratedAccZValues.clear();
      }
    }
  }

  groundContactState.contact = contact;

  expectedRotationInv = theRobotModel.limbs[MassCalibration::footLeft].translation.z > theRobotModel.limbs[MassCalibration::footRight].translation.z ? theRobotModel.limbs[MassCalibration::footLeft].rotation : theRobotModel.limbs[MassCalibration::footRight].rotation;
}
