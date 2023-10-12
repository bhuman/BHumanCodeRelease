/**
 * @file Tools/Motion/Transformation.cpp
 *
 * @author Philip Reichenberg
 */

#include "Transformation.h"

Pose2f Motion::Transformation::getTransformationToZeroStep(const TorsoMatrix& torsoMatrix, const RobotModel& robotModel,
                                                           const RobotDimensions& robotDimensions, const Pose2f& startOdometry,
                                                           const Pose2f& targetOdometry, const bool isLeftSwing)
{
  const Pose3f supportInTorso3D = torsoMatrix * (isLeftSwing ? robotModel.soleRight : robotModel.soleLeft);
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f hipOffset(0.f, 0.f, isLeftSwing ? -robotDimensions.yHipOffset : robotDimensions.yHipOffset);
  return hipOffset * supportInTorso.inverse() * targetOdometry.inverse() * startOdometry;
}
