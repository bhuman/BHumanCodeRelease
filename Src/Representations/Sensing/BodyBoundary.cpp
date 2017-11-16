#include "BodyBoundary.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Debugging/DebugDrawings3D.h"

Object::Object(Limbs::Limb origin, ObjectType oType, Vector3f offset, float radius) :
  origin(origin), oType(oType), offset(offset), radius(radius)
{}

void Object::update(const Pose3f* limbs)
{
  pose = limbs[origin].translated(offset);
}

Sphere::Sphere(Limbs::Limb origin, Vector3f offset, float radius) :
  Object(origin, Object::sphere, offset, radius)
{}

bool Sphere::isIntersectingWith(const Object& object, Geometry::LineSegment3D& smallestConnectingVector) const
{
  switch(object.oType)
  {
    case sphere:
    {
      const Sphere& obj = static_cast<const Sphere&>(object);
      smallestConnectingVector = Geometry::LineSegment3D(pose.translation, obj.pose.translation);
      return (obj.pose.translation - pose.translation).squaredNorm() <= sqr(radius + obj.radius);
    }
    case capsule:
    {
      const Capsule& obj = static_cast<const Capsule&>(object);
      const Pose3f thisInObj = obj.pose.inverse() * pose;
      if(std::abs(thisInObj.translation.y()) <= obj.length / 2.f)
      {
        smallestConnectingVector = Geometry::LineSegment3D(pose.translation, obj.pose.translated(Vector3f(0, thisInObj.translation.y(), 0)).translation);
        return Vector3f(thisInObj.translation.x(), 0, thisInObj.translation.z()).squaredNorm() <= sqr(radius + obj.radius);
      }
      else
      {
        if(thisInObj.translation.y() > 0)
        {
          smallestConnectingVector = Geometry::LineSegment3D(pose.translation, obj.pose.translated(Vector3f(0, obj.length / 2.f, 0)).translation);
          return (Vector3f(0, obj.length / 2.f, 0) - thisInObj.translation).squaredNorm() <= sqr(radius + obj.radius);
        }
        else
        {
          smallestConnectingVector = Geometry::LineSegment3D(pose.translation, obj.pose.translated(Vector3f(0, -obj.length / 2.f, 0)).translation);
          return (Vector3f(0, -obj.length / 2.f, 0) - thisInObj.translation).squaredNorm() <= sqr(radius + obj.radius);
        }
      }
    }
    default:
    {
      FAIL("Unknown object type.");
      return false;
    }
  }
}

void Sphere::draw()
{
  SPHERE3D("representation:BodyBoundary:robot", pose.translation.x(), pose.translation.y(), pose.translation.z(), radius, ColorRGBA::red);
}

Capsule::Capsule(Limbs::Limb origin, Vector3f offset, Vector3a rotations, float radius, float length) :
  Object(origin, Object::capsule, offset, radius), rotations(rotations), length(length)
{}

void Capsule::update(const Pose3f* limbs)
{
  pose = limbs[origin] * Pose3f(offset).rotateX(rotations.x()).rotateY(rotations.y()).rotateZ(rotations.z());
}

bool Capsule::isIntersectingWith(const Object& object, Geometry::LineSegment3D& smallestConnectingVector) const
{
  switch(object.oType)
  {
    case sphere:
    {
      const Sphere& obj = static_cast<const Sphere&>(object);
      const Pose3f objInThis = pose.inverse() * obj.pose;
      if(std::abs(objInThis.translation.y()) <= length / 2.f)
      {
        smallestConnectingVector = Geometry::LineSegment3D(pose.translated(Vector3f(0, objInThis.translation.y(), 0)).translation, obj.pose.translation);
        return Vector3f(objInThis.translation.x(), 0, objInThis.translation.z()).squaredNorm() <= sqr(radius + obj.radius);
      }
      else
      {
        if(objInThis.translation.y() > 0)
        {
          smallestConnectingVector = Geometry::LineSegment3D(pose.translated(Vector3f(0, length / 2.f, 0)).translation, obj.pose.translation);
          return (Vector3f(0, length / 2.f, 0) - objInThis.translation).squaredNorm() <= sqr(radius + obj.radius);
        }
        else
        {
          smallestConnectingVector = Geometry::LineSegment3D(pose.translated(Vector3f(0, -length / 2.f, 0)).translation, obj.pose.translation);
          return (Vector3f(0, -length / 2.f, 0) - objInThis.translation).squaredNorm() <= sqr(radius + obj.radius);
        }
      }
    }
    case capsule:
    {
      const Capsule& obj = static_cast<const Capsule&>(object);
      const float dis = Geometry::distance(Geometry::LineSegment3D(pose.translated(Vector3f(0, length / 2, 0)).translation, pose.translated(Vector3f(0, -length / 2, 0)).translation),
                                           Geometry::LineSegment3D(obj.pose.translated(Vector3f(0, obj.length / 2, 0)).translation, obj.pose.translated(Vector3f(0, -obj.length / 2, 0)).translation),
                                           smallestConnectingVector);
      return dis <= radius + obj.radius;
    }
    default:
    {
      FAIL("Unknown object type.");
      return false;
    }
  }
}

void Capsule::draw()
{
  const Vector3f leftPos(pose * Vector3f(0, length / 2.f, 0));
  const Vector3f rightPos(pose * Vector3f(0, -length / 2.f, 0));
  CYLINDERLINE3D("representation:BodyBoundary:robot", leftPos, rightPos, radius, ColorRGBA::yellow);
  SPHERE3D("representation:BodyBoundary:robot", leftPos.x(), leftPos.y(), leftPos.z(), radius, ColorRGBA::yellow);
  SPHERE3D("representation:BodyBoundary:robot", rightPos.x(), rightPos.y(), rightPos.z(), radius, ColorRGBA::yellow);
}

void BodyBoundary::update(const Pose3f* limbs)
{
  FOREACH_ENUM(BodyObject, i)
    objects[i]->update(limbs);
}

void BodyBoundary::updateOneArm(const Arms::Arm arm, const Pose3f limbs[Limbs::numOfLimbs])
{
  if(arm == Arms::left)
  {
    foreArmLeftVal.update(limbs);
    handLeftVal.update(limbs);
  }
  else
  {
    foreArmRightVal.update(limbs);
    handRightVal.update(limbs);
  }
}

void BodyBoundary::draw()
{
  DEBUG_DRAWING3D("representation:BodyBoundary:robot", "robot")
  {
    FOREACH_ENUM(BodyObject, i)
      objects[i]->draw();
  }
}

void BodyBoundaryParameters::initBodyBoundary(BodyBoundary& bodyBoundary)
{
  bodyBoundary.foreArmLeftVal = leftForeArm;
  bodyBoundary.foreArmRightVal = rightForeArm;
  bodyBoundary.handLeftVal = lHand;
  bodyBoundary.handRightVal = rHand;
  bodyBoundary.headVal = headSphere;
  bodyBoundary.earsVal = headCapsule;
  bodyBoundary.chestVal = bodySphere;
  bodyBoundary.bellyVal = bodyCapsule;
  bodyBoundary.hipLeftVal = lHip;
  bodyBoundary.hipRightVal = rHip;
  bodyBoundary.thighLeftVal = leftThigh;
  bodyBoundary.thighRightVal = rightThigh;
  bodyBoundary.tibiaLeftVal = leftTibia;
  bodyBoundary.tibiaRightVal = rightTibia;
}
