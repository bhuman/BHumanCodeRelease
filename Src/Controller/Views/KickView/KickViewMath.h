/**
 * Declaration of class KickViewMath
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"

namespace KickViewMath
{
  bool intersectRayAndPlane(const Vector3f& point, const Vector3f& v,
                            const Vector3f& plane, const Vector3f& n,
                            Vector3f& intersection);
  bool intersectRayAndBox(const Vector3f& rayPosition, const Vector3f& ray,
                          const Vector3f& boxPosition, const RotationMatrix& boxRotation,
                          float length, float width, float height, Vector3f& intersection);
  void transformRayToObjectAtOrigin(const Vector3f& rayPosition, const Vector3f& ray,
                                    const Vector3f& objPosition, const RotationMatrix& objRotation,
                                    Vector3f& transformedPosition, Vector3f& transformedRay);
  bool intersectRayWithAxisAlignedPlane(const Vector3f& p, const Vector3f& v,
                                        float distance, int thirdAxis,
                                        float maxAbsAxis1, float maxAbsAxis2,
                                        Vector3f& intersectionPos);

  Pose3f calculateFootPos(const JointAngles& jointAngles, const Joints::Joint& joint, const RobotDimensions& robotDimensions);

  Pose3f calculateHandPos(const JointAngles& jointAngles, const Joints::Joint& joint, const RobotDimensions& robotDimensions);

  Vector3f calcFootPos(const float& leg0, const float& leg1, const float& leg2, const float& leg3, const float& leg4, const float& leg5,
                       const Joints::Joint& joint, const RobotDimensions& robotDimensions);

  bool calcLegJoints(const Vector3f& footPos, const Vector3f& footRotAng, bool left, const RobotDimensions& robotDimensions,
                     float& leg0, float& leg1, float& leg2, float& leg3, float& leg4, float& leg5);
};
