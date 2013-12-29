/**
 * @file BodyDynamics.h
 * Declaration of class BodyDynamics.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/SpatialVector.h"
#include "Tools/Math/SpatialTransform.h"
#include "Tools/Math/SpatialInertia.h"
#include "Tools/Math/SpatialInertiaDerivative.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/MotionControl/IndykickRequest.h"
#include "Representations/Infrastructure/JointData.h"

/**
 * Representation a body, i.e. a limb of the robot.
 */
STREAMABLE(Body,
{
public:
  bool isFinite() const
  {
    ASSERT(I.isFinite());
    ASSERT(X.isFinite());
    ASSERT(mode.isFinite());
    ASSERT(v.isFinite());
    ASSERT(a.isFinite());
    ASSERT(f.isFinite());
    ASSERT(Ic.isFinite());
    ASSERT(pc.isFinite());
    ASSERT(fc.isFinite());
    ASSERT(dIc.isFinite());
    return true;
  },

  (SpatialInertia) I, /**< Inertia. In contrast to the composite inertia, I is constant. */
  (SpatialTransform) X, /**< Transform from parent body coordinates to body coordinates. */
  (SpatialTransform) Xinv, /**< Inverse of X: Transform from body coordinates to parent body coordinates. */
  (SpatialTransform) XinOrigin, /**< Transform from body coordinates to coordinates of the robot's origin. */
  (SpatialVector<>) mode, /**< Free mode of the joint connecting the parent body to this body in this-body coordinates. */

  (SpatialVector<>) v, /**< Velocity of this body. */
  (SpatialVector<>) a, /**< Accelertion of this body. */
  (SpatialVector<>) f, /**< Force excerted by the parent body on this body. */

  /* Variables to compute the first derivative of the inverse dynamics. Composite force of the base should be equal to 'f' (does not apply to all other bodies). */
  (SpatialInertia) Ic, /**< Composite inertia. Local inertia of this body plus the composite inertias of the children bodies. */
  (SpatialVector<>) pc, /**< Composite momentum (force vector!). Local momentum plus the composite momenta of the children bodies. */
  (SpatialVector<>) fc, /**< Composite force. Local force plus the composite forces of the children bodies. Except for the base, this is different from f. */
  (SpatialInertiaDerivative) dIc, /**< Time derivative of the composite inertia. TODO: This requires matrix-inertia and inertia-matrix multiplication which go beyond coordinate transforms. */
});

/**
 * Representation of the dynamics of the entire robot.
 * The robot is modeled as a kinematic tree with the joints being the edges
 * and the bodies (robot limbs) being the vertices.
 */
STREAMABLE(BodyDynamics,
{
public:
  BodyDynamics(const MassCalibration& massCalibration) {init(massCalibration);}

  void init(const MassCalibration& massCalibration),

  (bool)(false) inertiasInitialized, /**< True if the inertia matrices have been set. */
  (Body[MassCalibration::numOfLimbs]) limbs,
  (IndykickRequest, SupportLeg) supportLeg,
});

class FutureBodyDynamics : public BodyDynamics {};

STREAMABLE(BodyDynamicsDerivatives,
{
public:
  STREAMABLE(Derivative,
  {,
    (SpatialVector<>) fcByqFoot, /**< Composite force at the support foot derived by q (joint position) in support foot coordinates. */
    (SpatialVector<>) fcByqDotFoot, /**< Composite force at the support foot derived by q-dot (joint velocity) in support foot coordinates. */
    (SpatialVector<>) fcByqDoubleDotFoot, /**< Composite force at the support foot derived by q-doulbe-dot (joint acceleration) in support foot coordinates. */
    (SpatialVector<>) fcByqGround, /**< Composite force at the support foot derived by q (joint position) in ground coordinates. */
    (SpatialVector<>) fcByqDotGround, /**< Composite force at the support foot derived by q-dot (joint velocity) in ground coordinates. */
    (SpatialVector<>) fcByqDoubleDotGround, /**< Composite force at the support foot derived by q-doulbe-dot (joint acceleration) in ground coordinates. */
  }),

  (Derivative[JointData::numOfJoints]) partialDerivatives, /**< The partial derivatives of the composite force by the joint dynamics at the support foot. */
  (IndykickRequest, SupportLeg) supportLeg, /**< The support leg for which this body dynamics derivatives have been calculated. */
});

class FutureBodyDynamicsDerivatives : public BodyDynamicsDerivatives {};
