/**
 * Implementation of class KickViewMath
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include "KickViewMath.h"
#include "Tools/Range.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/BHMath.h"

bool KickViewMath::intersectRayAndBox(const Vector3f& rayPosition, const Vector3f& ray,
                                      const Vector3f& boxPosition, const RotationMatrix& boxRotation,
                                      float length, float width, float height, Vector3f& intersection)
{
  // Considering an unrotated box at the origin, the ray has to be transformed:
  Vector3f rayPos, v;
  transformRayToObjectAtOrigin(rayPosition, ray, boxPosition, boxRotation, rayPos, v);
  // Now three sides of the box have to be chosen to be intersected:
  float l_2 = length * 0.5f;
  float w_2 = width * 0.5f;
  float h_2 = height * 0.5f;
  Vector3f intersectionPos;
  Vector3f foundPoints[3];
  int numberOfPoints = 0;
  if(rayPos.x() > 0)
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, l_2, 0, w_2, h_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  else
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, -l_2, 0, w_2, h_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  if(rayPos.y() > 0)
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, w_2, 1, h_2, l_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  else
  {
    if(intersectRayWithAxisAlignedPlane(rayPos, v, -w_2, 1, h_2, l_2, intersectionPos))
      foundPoints[numberOfPoints++] = intersectionPos;
  }
  if(rayPos.z() > 0)
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
    int nearestPoint = 0;
    float shortestDist = (rayPos - foundPoints[0]).squaredNorm();
    for(int i = 1; i < numberOfPoints; i++)
    {
      float currentDist = (rayPos - foundPoints[i]).squaredNorm();
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
    return false;
}

bool KickViewMath::intersectRayAndPlane(const Vector3f& point, const Vector3f& v,
                                        const Vector3f& plane, const Vector3f& n,
                                        Vector3f& intersection)
{
  Vector3f p = plane - point;
  float denominator = n.dot(v);
  if(denominator == 0.0)
    return false;

  float r = n.dot(p) / denominator;
  if(r < 0.0)
    return false;

  intersection = v;
  intersection *= r;
  intersection += point;
  return true;
}

void KickViewMath::transformRayToObjectAtOrigin(const Vector3f& rayPosition, const Vector3f& ray,
                                                const Vector3f& objPosition, const RotationMatrix& objRotation,
                                                Vector3f& transformedPosition, Vector3f& transformedRay)
{
  RotationMatrix inverseObjRotation = objRotation;
  inverseObjRotation.transpose();
  transformedPosition = inverseObjRotation * (rayPosition - objPosition);
  transformedRay = inverseObjRotation * ray;
}

bool KickViewMath::intersectRayWithAxisAlignedPlane(const Vector3f& p, const Vector3f& v,
                                                    float distance, int thirdAxis,
                                                    float maxAbsAxis1, float maxAbsAxis2,
                                                    Vector3f& intersectionPos)
{
  const float* vv = &v.x();
  const float* pv = &p.x();
  float* intersectionPosv = &intersectionPos.x();
  if(vv[thirdAxis] != 0.0)
  {
    float s = (distance - pv[thirdAxis]) / vv[thirdAxis];
    int i = thirdAxis;
    intersectionPosv[i++] = distance;
    i %= 3;
    intersectionPosv[i] = s * vv[i] + pv[i];
    if(std::abs(intersectionPosv[i]) > maxAbsAxis1)
      return false;
    i = (thirdAxis + 2) % 3;
    intersectionPosv[i] = s * vv[i] + pv[i];
    return std::abs(intersectionPosv[i]) <= maxAbsAxis2;
  }
  else
  {
    //No possible solution
    return false;
  }
}

Pose3f KickViewMath::calculateFootPos(const JointAngles& jointAngles, const Joints::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3f footPos;
  float sign = joint == Joints::lHipYawPitch ? -1.f : 1.f;
  footPos.translate(0, -sign * (robotDimensions.yHipOffset), 0)
    .rotateX(-pi_4 * sign)
    .rotateZ(jointAngles.angles[joint] * sign)
    .rotateX(((jointAngles.angles[joint + 1] + pi_4)*sign))
    .rotateY(jointAngles.angles[joint + 2])
    .translate(0, 0, -robotDimensions.upperLegLength)
    .rotateY(jointAngles.angles[joint + 3])
    .translate(0, 0, -robotDimensions.lowerLegLength);

  footPos.translation += Vector3f(0, 0, -robotDimensions.footHeight); //because inverse kinematics subtracts this

  return footPos;
}

Pose3f KickViewMath::calculateHandPos(const JointAngles& jointAngles, const Joints::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3f handPos;
  float sign = joint == Joints::lShoulderPitch ? 1.f : -1.f;

  handPos.translate(robotDimensions.armOffset.x(), sign *  robotDimensions.armOffset.y(), robotDimensions.armOffset.z());

  handPos.rotateY(-jointAngles.angles[joint]);
  handPos.rotateZ(sign * jointAngles.angles[joint + 1]);
  handPos.translate(robotDimensions.upperArmLength, 0, 0);

  handPos.rotateX(jointAngles.angles[joint + 2] * sign);
  handPos.rotateZ(sign * jointAngles.angles[joint + 3]);

  return handPos;
}

Vector3f KickViewMath::calcFootPos(const float& leg0, const float& leg1, const float& leg2, const float& leg3, const float& leg4, const float&
                                   leg5, const Joints::Joint& joint, const RobotDimensions& robotDimensions)
{
  Pose3f footPos;
  footPos.translate(0, 0, -robotDimensions.footHeight);
  float sign = joint == Joints::lHipYawPitch ? 1.f : -1.f;
  footPos.translate(0, sign * (robotDimensions.yHipOffset), 0);
  footPos.rotateX(-pi_4 * sign);
  footPos.rotateZ(-leg0 * sign);
  footPos.rotateX(pi_4 * sign);
  footPos.rotateX(-leg1 * sign);
  footPos.rotateY(leg2);
  footPos.translate(0, 0, -robotDimensions.upperLegLength);
  footPos.rotateY(leg3);
  footPos.translate(0, 0, -robotDimensions.lowerLegLength);
  return footPos.translation;
}

bool KickViewMath::calcLegJoints(const Vector3f& footPos, const Vector3f& footRotAng, bool left, const RobotDimensions& robotDimensions,
                                 float& leg0, float& leg1, float& leg2, float& leg3, float& leg4, float& leg5)
{
  float sign = left ? 1.f : -1.f;
  bool legTooShort = false;

  Vector3f anklePos = footPos + Vector3f(0.f, 0.f, robotDimensions.footHeight);

  anklePos -= Vector3f(0.f, sign * (robotDimensions.yHipOffset), 0.f);

  const RotationMatrix rotateBecauseOfHip = RotationMatrix::aroundZ(sign * footRotAng.z()).rotateX(-sign * pi_4);

  anklePos = rotateBecauseOfHip * anklePos;
  leg0 = footRotAng.z();
  leg2 = -std::atan2(anklePos.x(), Vector2f(anklePos.y(), anklePos.z()).norm());

  float diagonal = anklePos.norm();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (robotDimensions.upperLegLength * robotDimensions.upperLegLength -
              robotDimensions.lowerLegLength * robotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * robotDimensions.upperLegLength * diagonal);
  a1 = std::abs(a1) > 1.f ? 0 : std::acos(a1);

  float a2 = (robotDimensions.upperLegLength * robotDimensions.upperLegLength +
              robotDimensions.lowerLegLength * robotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * robotDimensions.upperLegLength * robotDimensions.lowerLegLength);
  if(!Rangef::OneRange().isInside(a2))
    legTooShort = true;

  a2 = std::abs(a2) > 1.f ? pi : std::acos(a2);

  leg1 = (-sign * std::atan2(anklePos.y(), std::abs(anklePos.z()))) - (pi / 4);
  leg2 -= a1;
  leg3 = pi - a2;

  RotationMatrix footRot = RotationMatrix::aroundX(leg1 * -sign).rotateY(leg2 + leg3);
  footRot = footRot.inverse() * rotateBecauseOfHip;
  leg5 = std::asin(-footRot(1, 2)) * -sign;
  leg4 = -std::atan2(footRot(0, 2), footRot(2, 2)) * -1;
  leg4 += footRotAng.y();
  leg5 += footRotAng.x();

  return legTooShort;
}
