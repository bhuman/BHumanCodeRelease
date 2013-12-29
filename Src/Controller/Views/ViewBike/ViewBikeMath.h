/**
*
*
* Declaration of class ViewBikeMath
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#pragma once
#include "Tools/Math/Pose3D.h"
#include "Tools/Math/RotationMatrix.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"

class ViewBikeMath
{
public:
  ViewBikeMath() {};
  ~ViewBikeMath() {};

  static bool intersectRayAndPlane(const Vector3<>& point, const Vector3<>& v,
                                   const Vector3<>& plane, const Vector3<>& n,
                                   Vector3<>& intersection);
  static bool intersectRayAndBox(const Vector3<>& rayPosition, const Vector3<>& ray,
                                 const Vector3<>& boxPosition, const RotationMatrix& boxRotation,
                                 float length, float width, float height, Vector3<>& intersection);
  static void transformRayToObjectAtOrigin(const Vector3<>& rayPosition, const Vector3<>& ray,
      const Vector3<>& objPosition, const RotationMatrix& objRotation,
      Vector3<>& transformedPosition, Vector3<>& transformedRay);
  static bool intersectRayWithAxisAlignedPlane(const Vector3<>& p, const Vector3<>& v,
      float distance, int thirdAxis,
      float maxAbsAxis1, float maxAbsAxis2,
      Vector3<>& intersectionPos);

  static Pose3D calculateFootPos(const SensorData& sensorData, const JointData& jointData, const JointData::Joint& joint, const RobotDimensions& robotDimensions);

  static Pose3D calculateHandPos(const JointData& jointData, const JointData::Joint& joint, const RobotDimensions& robotDimensions);


  static Vector3<> calcFootPos(const float& leg0, const float& leg1, const float& leg2, const float& leg3, const float& leg4, const float& leg5, const JointData::Joint& joint, const RobotDimensions& robotDimensions);

  static bool calcLegJoints(const Vector3<>& footPos, const Vector3<>& footRotAng, const bool& left, const RobotDimensions& robotDimensions, float& leg0, float& leg1, float& leg2, float& leg3, float& leg4, float& leg5);


};
