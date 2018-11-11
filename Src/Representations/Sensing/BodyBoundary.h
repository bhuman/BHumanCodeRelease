/**
 * @file BodyBoundary.h
 *
 * Declaration of struct BodyBoundary.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "RobotModel.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Geometry.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(Object,
{
  ENUM(ObjectType,
  {,
    sphere,
    capsule,
  });

  Object() = default;
  Object(Limbs::Limb origin, ObjectType oType, Vector3f offset, float radius);

  virtual void update(const Pose3f* limbs);
  virtual bool isIntersectingWith(const Object& object, Geometry::LineSegment3D& smallestConnectingVector) const = 0;
  virtual void draw() = 0,

  (Limbs::Limb)(Limbs::numOfLimbs) origin,
  (ObjectType)(numOfObjectTypes) oType,
  (Pose3f) pose,
  (Vector3f)(Vector3f::Zero()) offset,
  (float)(0.f) radius,
});

STREAMABLE_WITH_BASE(Sphere, Object,
{
  Sphere() = default;
  Sphere(Limbs::Limb origin, Vector3f offset, float radius);

  bool isIntersectingWith(const Object& object, Geometry::LineSegment3D& smallestConnectingVector) const override;
  void draw() override,
});

STREAMABLE_WITH_BASE(Capsule, Object,
{
  Capsule() = default;
  Capsule(Limbs::Limb origin, Vector3f offset, Vector3a rotations, float radius, float length);

  void update(const Pose3f* limbs) override;
  bool isIntersectingWith(const Object& object, Geometry::LineSegment3D& smallestConnectingVector) const override;
  void draw() override,

  (Vector3a)(Vector3a::Zero()) rotations,
  (float)(0.f) length,
});

/**
 * @struct BodyBoundary
 */
STREAMABLE(BodyBoundary,
{
  ENUM(BodyObject,
  {,
    foreArmLeft,
    foreArmRight,
    handLeft,
    handRight,

    firstNonArmObject,
    head = firstNonArmObject,
    ears,
    belly,
    chest,
    hipLeft,
    hipRight,
    thighLeft,
    thighRight,
    tibiaLeft,
    tibiaRight,
  });

  Object* objects[numOfBodyObjects];

  BodyBoundary()
  {
    objects[foreArmLeft] = &foreArmLeftVal;
    objects[foreArmRight] = &foreArmRightVal;
    objects[handLeft] = &handLeftVal;
    objects[handRight] = &handRightVal;
    objects[head] = &headVal;
    objects[ears] = &earsVal;
    objects[belly] = &bellyVal;
    objects[chest] = &chestVal;
    objects[hipLeft] = &hipLeftVal;
    objects[hipRight] = &hipRightVal;
    objects[thighLeft] = &thighLeftVal;
    objects[thighRight] = &thighRightVal;
    objects[tibiaLeft] = &tibiaLeftVal;
    objects[tibiaRight] = &tibiaRightVal;
  }

  void draw();
  void update(const Pose3f* limbs);
  void updateOneArm(const Arms::Arm arm, const Pose3f limbs[Limbs::numOfLimbs]),

  (Capsule) foreArmLeftVal,
  (Capsule) foreArmRightVal,
  (Sphere) handLeftVal,
  (Sphere) handRightVal,

  (Sphere) headVal,
  (Capsule) earsVal,
  (Capsule) bellyVal,
  (Sphere) chestVal,
  (Sphere) hipLeftVal,
  (Sphere) hipRightVal,
  (Capsule) thighLeftVal,
  (Capsule) thighRightVal,
  (Capsule) tibiaLeftVal,
  (Capsule) tibiaRightVal,

  (bool[8]) fAL,// debugOutput, not filled in a normal case
  (bool[8]) fAR,
  (bool[8]) hL,
  (bool[8]) hR,
});

STREAMABLE(BodyBoundaryParameters,
{
  void initBodyBoundary(BodyBoundary& bb),

  (Sphere)(Sphere(Limbs::head, Vector3f(0, 0, 55.f), 65.f)) headSphere,
  (Capsule)(Capsule(Limbs::head, Vector3f(0, 0, 54.f), Vector3a::Zero(), 50.f, 55.f)) headCapsule,

  (Capsule)(Capsule(Limbs::foreArmLeft, Vector3f(65, 0, 0), Vector3a(0, 0, 90_deg), 31.f, 100.f)) leftForeArm,
  (Sphere)(Sphere(Limbs::foreArmLeft, Vector3f(105.f, 0, -2.f), 39.f)) lHand,
  (Capsule)(Capsule(Limbs::foreArmRight, Vector3f(65.f, 0, 0), Vector3a(0, 0, 90_deg), 31.f, 100.f)) rightForeArm,
  (Sphere)(Sphere(Limbs::foreArmRight, Vector3f(105.f, 0, -2.f), 39.f)) rHand,

  (Capsule)(Capsule(Limbs::torso, Vector3f(-6.f, 0, 100.f), Vector3a(90_deg, 0, 10_deg), 47.f, 120.f)) bodyCapsule,
  (Sphere)(Sphere(Limbs::torso, Vector3f(0, 0, 145.f), 70.f)) bodySphere,

  (Sphere)(Sphere(Limbs::hipLeft, Vector3f(-10.f, -5.f, 0), 50.f)) lHip,
  (Sphere)(Sphere(Limbs::hipRight, Vector3f(-10.f, 5.f, 0), 50.f)) rHip,
  (Capsule)(Capsule(Limbs::thighLeft, Vector3f(5.f, 0, -65.f), Vector3a(96_deg, 0, 17_deg), 45.f, 60.f)) leftThigh,
  (Capsule)(Capsule(Limbs::thighRight, Vector3f(5.f, 0, -65.f), Vector3a(84_deg, 0, 17_deg), 45.f, 60.f)) rightThigh,
  (Capsule)(Capsule(Limbs::tibiaLeft, Vector3f(0.f, 5.f, -50.f), Vector3a(0, 100_deg, 90_deg), 45.f, 90.f)) leftTibia,
  (Capsule)(Capsule(Limbs::tibiaRight, Vector3f(0.f, -5.f, -50.f), Vector3a(0, 100_deg, 90_deg), 45.f, 90.f)) rightTibia,
});
