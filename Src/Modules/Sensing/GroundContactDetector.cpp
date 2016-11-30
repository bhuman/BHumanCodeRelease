/**
 * @file GroundContactDetector.cpp
 * Implementation of module GroundContactDetector.
 * @author Colin Graf
 */

#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(GroundContactDetector, sensing)

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

  if(theMotionInfo.motion == MotionRequest::getUp) //hack to avoid long pause after get up
  {
    contact = true;
    useAngle = false;
    groundContactState.contact = contact;
    contactStartTime = theFrameInfo.time;
  }

  bool ignoreSensors = (theMotionInfo.motion != MotionRequest::walk && theMotionInfo.motion != MotionRequest::stand &&
                        theMotionInfo.motion != MotionRequest::specialAction && theMotionInfo.motion != MotionRequest::getUp) ||
                       (theMotionRequest.motion != MotionRequest::walk && theMotionRequest.motion != MotionRequest::stand &&
                        theMotionRequest.motion != MotionRequest::specialAction && theMotionRequest.motion != MotionRequest::getUp) ||
                       (theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.walkKickRequest.kickType != WalkKicks::none) ||
                       (theMotionRequest.motion == MotionRequest::walk && theMotionRequest.walkRequest.walkKickRequest.kickType != WalkKicks::none) ||
                       (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction != SpecialActionRequest::standHigh) ||
                       (theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::standHigh);
  if(!ignoreSensors)
  {
    if(contact)
    {
      calibratedAccZValues.push_front(theInertialData.acc.z());

      Vector3f angleDiff = (theTorsoMatrix.rotation * expectedRotationInv).getPackedAngleAxis();
      angleNoises.push_front(Vector2f(sqr(angleDiff.x()), sqr(angleDiff.y())));
      Vector2f angleNoise = angleNoises.average();
      PLOT("module:GroundContactDetector:angleNoiseX", angleNoise.x());
      PLOT("module:GroundContactDetector:angleNoiseY", angleNoise.y());

      if(!useAngle && angleNoises.full() && angleNoise.x() < contactAngleActivationNoise && angleNoise.y() < contactAngleActivationNoise)
        useAngle = true;

      if((useAngle && (angleNoise.x() > contactMaxAngleNoise || angleNoise.y() > contactMaxAngleNoise)) ||
         (calibratedAccZValues.full() && calibratedAccZValues.average() > contactMaxAccZ))
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
        if(SystemCall::getMode() == SystemCall::physicalRobot && contactStartTime != 0)
          SystemCall::playSound("high.wav");
      }
    }
    else
    {
      const Vector3f accAverage = accValues.average();
      const Vector2f gyroAverage = gyroValues.average();
      const Vector2f gyro = theInertialData.gyro.head<2>().cast<float>();
      const Vector3f acc = theInertialData.acc.cast<float>();
      accValues.push_front(acc);
      gyroValues.push_front(gyro);
      if(accValues.full())
      {
        accNoises.push_front((acc - accAverage).array().square());
        gyroNoises.push_front((gyro - gyroAverage).array().square());
      }
      Vector3f accNoise = accNoises.average();
      Vector2f gyroNoise = gyroNoises.average();
      PLOT("module:GroundContactDetector:accNoiseX", accNoise.x());
      PLOT("module:GroundContactDetector:accNoiseY", accNoise.y());
      PLOT("module:GroundContactDetector:accNoiseZ", accNoise.z());
      PLOT("module:GroundContactDetector:gyroNoiseX", gyroNoise.x());
      PLOT("module:GroundContactDetector:gyroNoiseY", gyroNoise.y());

      if(accNoises.full() &&
         std::abs(accAverage.z() - Constants::g_1000) < 5.f && std::abs(accAverage.x()) < 5.f && std::abs(accAverage.y()) < 5.f &&
         accNoise.x() < noContactMinAccNoise && accNoise.y() < noContactMinAccNoise && accNoise.z() < noContactMinAccNoise &&
         gyroNoise.x() < noContactMinGyroNoise && gyroNoise.y() < noContactMinGyroNoise)
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

  expectedRotationInv = theRobotModel.limbs[Limbs::footLeft].translation.z() > theRobotModel.limbs[Limbs::footRight].translation.z() ? theRobotModel.limbs[Limbs::footLeft].rotation : theRobotModel.limbs[Limbs::footRight].rotation;
}
