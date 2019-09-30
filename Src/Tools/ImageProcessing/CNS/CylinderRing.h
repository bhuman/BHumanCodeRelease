#pragma once

#include "Tools/ImageProcessing/SIMD.h"
#include <Eigen/Geometry>

//! A cylindrical ring as a volume in 3D
/*! The cylinder ring is defined by a coordinate frame
    where Z is the axis, an inner radius, an outer radius
    and a Z-interval.

    See \c isInside for a formal definition
 */
class CylinderRing
{
public:
  //! Pose of the cylinder (Z is the axis)
  Eigen::Isometry3d cylinder2World;

  //! radial interval
  double rLow, rHigh;

  //! axial interval
  double zLow, zHigh;

  //! Empty cylinder
  CylinderRing() : cylinder2World(Eigen::Isometry3d::Identity()), rLow(0), rHigh(0),
    zLow(std::numeric_limits<double>::infinity()), zHigh(-std::numeric_limits<double>::infinity()) {};

  //! CylinderRing component-wise defined
  CylinderRing(const Eigen::Isometry3d& cylinder2World, double rLow, double rHigh, double zLow, double zHigh)
    : cylinder2World(cylinder2World), rLow(rLow), rHigh(rHigh), zLow(zLow), zHigh(zHigh)
  {}

  //! Returns whether \c p is in the cylinder ring
  /*! This is the formal definition of the cylinder ring volume. */
  bool isInside(const Eigen::Vector3d& p) const
  {
    Eigen::Vector3d pCylinder = cylinder2World.inverse() * p;
    double r2 = pCylinder(0) * pCylinder(0) + pCylinder(1) * pCylinder(1);
    return zLow <= pCylinder(2) && pCylinder(2) <= zHigh && r2 <= rHigh * rHigh && rLow * rLow <= r2;
  }

  //! Returns the interval of lambda for which \c p+lambda*v is inside the cylinder ring
  /*! \c p+lambda*v is interpreted as a ray, i.e. \c lambda>0. Sometimes the intersection
      can consist of two parts, in that case the interval-union of both is returned.

      \c p and \c v are in cylinder coordinates
   */
  void intersectWithCylinderRay(double& minLambda, double& maxLambda, const Eigen::Vector3d& p, const Eigen::Vector3d& v) const;

  //! As above, but \c p and \c v in world coordinates
  void intersectWithRay(double& minLambda, double& maxLambda, const Eigen::Vector3d& p, const Eigen::Vector3d& v) const
  {
    Eigen::Isometry3d world2Cylinder = cylinder2World.inverse();
    intersectWithCylinderRay(minLambda, maxLambda, world2Cylinder * p, world2Cylinder.linear()*v);
  }

  //! Slow reference implementation of \c intersectWithRay
  /*! Samples \c lambda in steps of \c eps and calls \c inside. */
  void intersectWithRaySlow(double& minLambda, double& maxLambda, const Eigen::Vector3d& p, const Eigen::Vector3d& v, double eps = 1E-4) const;

  //! Returns a point on one of the four (\c whichEdge=0..3) edges of the cylinder
  /*! \c angle [0..2*M_PI] parametrizes the point.
    For visualization.
   */
  Eigen::Vector3d sampleEdgePoint(double angle, int whichEdge) const;

protected:
  //! intersects [minLambda..maxLambda] with {lambda|a*lambda+b>=0}
  static void clip(double& minLambda, double& maxLambda, double a, double b);

  //! intersects [minL..maxL] with {lambda|a*lambda^2+b*lambda+c>=0}
  /*! a must be <0. */
  static void clip(double& minLambda, double& maxLambda, double a, double b, double c);
};
