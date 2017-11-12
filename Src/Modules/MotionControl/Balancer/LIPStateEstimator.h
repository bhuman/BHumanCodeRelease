#pragma once

#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Motion/LIP3D.h"

STREAMABLE(LIPStateEstimatorParameters,
{
  void onRead(),

  (Vector2f)(20, 20) positionProcessDeviation, // Standard deviation of the position in the prediction step (in mm per second).
  (Vector2f)(20, 20) velocityProcessDeviation, // Standard deviation of the velocity in the prediction step (in mm/s per second).
  (Vector2f)(20, 20) zmpProcessDeviation, // Standard deviation of the zmp in the prediction step (in mm per second).
  (Vector2f)(20, 20) positionMeasurementDeviation, // Standard deviation of the zmp in the measurement step (in mm per second).
  (Vector2f)(20, 20) zmpMeasurementDeviation, // Standard deviation of the zmp in the measurement step (in mm per second).
});

class LIPStateEstimator
#ifdef TARGET_ROBOT
  : public AlignedMemory
#endif
{
public:
  enum class SupportFoot
  {
    left = 1,
    right = -1
  };

  struct EstimatedState
  {
    LIP3D com; // relative to origin
    Vector2f zmp = Vector2f::Zero(); // relative to origin
    SupportFoot supportFoot = SupportFoot::right;
    Vector2f origin = Vector2f::Zero();

    EstimatedState(const Array2f& comHeights) : com(comHeights) {};

    EstimatedState& update(float time) { com.update(time, zmp); return *this; }
    EstimatedState predict(float time) const { return EstimatedState(*this).update(time); }
  };

  LIPStateEstimator(const InertialData& theInertialData, const RobotModel& theRobotModel);

  void init(const Array2f& LIPHeights, const Vector2f& leftOrigin, LIPStateEstimatorParameters params);
  void update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin, const SupportFoot newSupportFoot);
  void update(float timePassed, const Array2f& LIPHeights, const Vector2f& leftOrigin);

  EstimatedState getEstimate() const;
  EstimatedState convertToOtherFoot(const EstimatedState& state) const;
  SupportFoot guessSupportFoot(const Vector2f& leftOrigin) const;
  void draw(float footOffsetY, float forwardingTime) const;
  void plot() const;

private:
  Array2f LIPHeights = Array2f::Zero();
  SupportFoot supportFoot = SupportFoot::right;
  Vector2f origin = Vector2f::Zero();

  UKF<6> ukf = UKF<6>(Vector6f::Zero()); // The statevector of the ukf is composed of: position, velocity, zmp;

  const InertialData& theInertialData;
  const RobotModel& theRobotModel;
  LIPStateEstimatorParameters params;

  Vector4f measure(SupportFoot supportFoot, const Vector2f& LIPOrigin) const;
};
