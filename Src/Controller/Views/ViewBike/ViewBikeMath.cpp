/**
*
* Implementation of class ViewBikeMath
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/


#include "ViewBikeMath.h"
#include "Tools/Range.h"
#include "Tools/Math/Vector2.h"

bool ViewBikeMath::intersectRayAndBox(const Vector3<>& rayPosition, const Vector3<>& ray,
                                      const Vector3<>& boxPosition, const RotationMatrix& boxRotation,
                                      float length, float width, float height, Vector3<>& intersection)
{
  // Considering an unrotated box at the origin, the ray has to be transformed:
  Vector3<> rayPos, v;
  transformRayToObjectAtOrigin(rayPosition, ray, boxPosition, boxRotation, rayPos, v);
  // Now three sides of the box have to be chosen to be intersected:
  float l_2(length * 0.5f);
  float w_2(width * 0.5f);
  float h_2(height * 0.5f);
  Vector3<> intersectionPos;
  Vector3<> foundPoints[3];
  int numberOfPoints(0);
  if(rayPos.x > 0)
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, l_2, 0, w_2, h_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  else
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, -l_2, 0, w_2, h_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  if(rayPos.y > 0)
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, w_2, 1, h_2, l_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  else
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, -w_2, 1, h_2, l_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  if(rayPos.z > 0)
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, h_2, 2, l_2, w_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  else
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, -h_2, 2, l_2, w_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  if(numberOfPoints)
  {
    int nearestPoint(0);
    float shortestDist((rayPos - foundPoints[0]).squareAbs());
    for(int i = 1; i < numberOfPoints; i++)
    {
      float currentDist((rayPos - foundPoints[i]).squareAbs());
      if(currentDist < shortestDist)
      {
        shortestDist = currentDist;
        nearestPoint = i;
      }
    }
    intersection = foundPoints[nearestPoint];
    intersection = boxRotation * intersection;
    intersection += boxPosition;
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewBikeMath::intersectRayAndPlane(const Vector3<>& point, const Vector3<>& v,
                                        const Vector3<>& plane, const Vector3<>& n,
                                        Vector3<>& intersection)
{
  Vector3<> p(plane - point);
  float denominator(n * v);
  if(denominator == 0.0)
  {
    return false;
  }
  float r(n * p / denominator);
  if(r < 0.0)
  {
    return false;
  }
  intersection = v;
  intersection *= r;
  intersection += point;
  return true;
}

void ViewBikeMath::transformRayToObjectAtOrigin(const Vector3<>& rayPosition, const Vector3<>& ray,
    const Vector3<>& objPosition, const RotationMatrix& objRotation,
    Vector3<>& transformedPosition, Vector3<>& transformedRay)
{
  transformedPosition = (rayPosition - objPosition);
  RotationMatrix inverseObjRotation(objRotation);
  inverseObjRotation.transpose();
  transformedPosition = inverseObjRotation * transformedPosition;
  transformedRay = ray;
  transformedRay = inverseObjRotation * transformedRay;
}

bool ViewBikeMath::intersectRayWithAxisAlignedPlane(const Vector3<>& p, const Vector3<>& v,
    float distance, int thirdAxis,
    float maxAbsAxis1, float maxAbsAxis2,
    Vector3<>& intersectionPos)
{
  const float* vv(&v.x);
  const float* pv(&p.x);
  float* intersectionPosv(&intersectionPos.x);
  if(vv[thirdAxis] != 0.0)
  {
    float s((distance - pv[thirdAxis]) / vv[thirdAxis]);
    int i(thirdAxis);
    intersectionPosv[i++] = distance;
    i %= 3;
    intersectionPosv[i] = s * vv[i] + pv[i];
    if(std::abs(intersectionPosv[i]) > maxAbsAxis1) return false;
    i = (thirdAxis + 2) % 3;
    intersectionPosv[i] = s * vv[i] + pv[i];
    if(std::abs(intersectionPosv[i]) > maxAbsAxis2) return false;
    return true;
  }
  else
  {
    //No possible solution
    return false;
  }
}

Pose3D ViewBikeMath::calculateFootPos(const SensorData& sensorData, const JointData& jointData, const JointData::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3D footPos;
  float sign = joint == JointData::LHipYawPitch ? -1.f : 1.f;
  footPos.translate(0, -sign * (robotDimensions.lengthBetweenLegs / 2), 0)
  .rotateX(-pi_4 * sign)
  .rotateZ(jointData.angles[joint]*sign)
  .rotateX(((jointData.angles[joint + 1] + pi_4)*sign))
  .rotateY(jointData.angles[joint + 2])
  .translate(0, 0, -robotDimensions.upperLegLength)
  .rotateY(jointData.angles[joint + 3])
  .translate(0, 0, -robotDimensions.lowerLegLength);

  footPos.translation += Vector3<>(0, 0, -robotDimensions.heightLeg5Joint); //because inverse kinematics subtracts this

  return footPos;
}

Pose3D ViewBikeMath::calculateHandPos(const JointData& jointData, const JointData::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3D handPos;
  float sign = joint == JointData::LShoulderPitch ? 1.f : -1.f;

  handPos.translate(robotDimensions.armOffset.x, sign *  robotDimensions.armOffset.y, robotDimensions.armOffset.z);

  handPos.rotateY(-jointData.angles[joint]);
  handPos.rotateZ(sign * jointData.angles[joint + 1]);
  handPos.translate(robotDimensions.upperArmLength, 0, 0);

  handPos.rotateX(jointData.angles[joint + 2] * sign);
  handPos.rotateZ(sign * jointData.angles[joint + 3]);


  return handPos;
}

Vector3<> ViewBikeMath::calcFootPos(const float& leg0, const float& leg1, const float& leg2, const float& leg3, const float& leg4, const float& leg5, const JointData::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3D footPos(0, 0, 0);
  footPos.translate(0, 0, -robotDimensions.heightLeg5Joint);
  float sign = joint == JointData::LHipYawPitch ? 1.f : -1.f;
  footPos.translate(0, sign * (robotDimensions.lengthBetweenLegs / 2), 0);
  footPos.rotateX(-pi_4 * sign);
  footPos.rotateZ(-leg0 * sign);
  footPos.rotateX(pi_4 * sign);
  footPos.rotateX(-leg1 * sign);
  footPos.rotateY(leg2);
  footPos.translate(0, 0, -robotDimensions.upperLegLength);
  footPos.rotateY(leg3);
  footPos.translate(0, 0, -robotDimensions.lowerLegLength);
  return Vector3<>(footPos.translation.x, footPos.translation.y, footPos.translation.z);
}

bool ViewBikeMath::calcLegJoints(const Vector3<>& footPos, const Vector3<>& footRotAng, const bool& left, const RobotDimensions& robotDimensions, float& leg0, float& leg1, float& leg2, float& leg3, float& leg4, float& leg5)
{
  float sign = (left) ? 1.f : -1.f;
  bool legTooShort = false;

  Vector3<> anklePos; //FLOAT
  anklePos = Vector3<>(footPos.x,  footPos.y, footPos.z + robotDimensions.heightLeg5Joint);

  anklePos -= Vector3<>(0.f, sign * (robotDimensions.lengthBetweenLegs / 2.f), 0.f);

  RotationMatrix rotateBecauseOfHip = RotationMatrix().rotateZ(sign * footRotAng.z).rotateX(-sign * pi_4);
  Vector3<> rotatedFootRotAng;


  anklePos = rotateBecauseOfHip * anklePos;
  leg0 = footRotAng.z;
  rotatedFootRotAng = rotateBecauseOfHip * footRotAng;

  leg2 = -std::atan2(anklePos.x, Vector2<>(anklePos.y, anklePos.z).abs());

  float diagonal = anklePos.abs();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (robotDimensions.upperLegLength * robotDimensions.upperLegLength -
              robotDimensions.lowerLegLength * robotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * robotDimensions.upperLegLength * diagonal);
  a1 = std::abs(a1) > 1.f ? 0 : std::acos(a1);

  float a2 = (robotDimensions.upperLegLength * robotDimensions.upperLegLength +
              robotDimensions.lowerLegLength * robotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * robotDimensions.upperLegLength * robotDimensions.lowerLegLength);
  const Range<float> clipping(-1.f, 1.f);
  if(!clipping.isInside(a2))
  {
    legTooShort = true;
  }
  a2 = std::abs(a2) > 1.f ? pi : std::acos(a2);

  leg1 = (-sign * std::atan2(anklePos.y, std::abs(anklePos.z))) - (pi / 4);
  leg2 -= a1;
  leg3 = pi - a2;

  RotationMatrix footRot = RotationMatrix().rotateX(leg1 * -sign).rotateY(leg2 + leg3);
  footRot = footRot.invert() * rotateBecauseOfHip;
  leg5 = std::asin(-footRot[2].y) * -sign;
  leg4 = -std::atan2(footRot[2].x, footRot[2].z) * -1;
  leg4 += footRotAng.y;
  leg5 += footRotAng.x;

  return legTooShort;
}

