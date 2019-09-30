#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>

//! Combination of an object pose and the detector response for that pose
/*! Used to returns the findings of \c ObjectCNSStereoDetector */
class IsometryWithResponse : public Eigen::Isometry3d
{
public:
  //! Detector response for the object pose \c *this
  double response;

  IsometryWithResponse(): Eigen::Isometry3d(), response(0) {}

  IsometryWithResponse(const Eigen::Isometry3d& iso, double response = 0) : Eigen::Isometry3d(iso), response(response) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IsometryWithResponse)

//! A comparison operator for sorting by descending responses
class MoreOnResponse
{
public:
  bool operator()(const IsometryWithResponse& a, const IsometryWithResponse& b) const
  {return a.response > b.response; }
};
