#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/JointDataPrediction.h"
#include "Representations/Sensing/Zmp.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Tools/RingBuffer.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"

MODULE(ZmpProvider,
{,
  USES(JointRequest),//note this has to be the old request, otherwise things break
  REQUIRES(JointDataPrediction),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(MotionSettings),
  REQUIRES(MassCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(InertialSensorData),
  PROVIDES(Zmp),
  LOADS_PARAMETERS(
  {,
    (float) zFactor,
    (bool) useBodyRotAcc,
    (bool) useRealCom,
    (Vector2f) accOffset, //calibration offset of the acceleration vector
    (Vector2f) comP,
  }),
});

class ZmpProvider : public ZmpProviderBase
{
private:
  RobotModel requestModel;
  RingBuffer<Vector3f, 2> leftComs;
  RingBuffer<Vector3f, 2> rightComs;

public:
  void update(Zmp& jv);

private:
  /**
   * Calculates the currently measured zmp
   */
  Vector2f calcZmp(const Vector3f& comInSole, const Pose3f& torsoInSole) const;
  Vector2f calcJointZmp(const Vector3f& comInSole, const Pose3f& torsoInSole, RingBuffer<Vector3f, 2>&);
  Vector2f calcRotZmp(const Vector3f& comInSole, const Pose3f& torsoInSole) const;//in left sole only
};
