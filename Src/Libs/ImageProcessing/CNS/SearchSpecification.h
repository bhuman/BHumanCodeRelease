#pragma once

#include "CylinderRing.h"
#include <Eigen/Geometry>
#include <vector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Isometry3d)

//! Specification of where and in which discretization to search for an object
/*! See the member definition and \c ObjectCNSStereoDetector::search (..) for
  a description of the search-process itself. */
class SearchSpecification
{
public:
  //! The origin of the object is searched for in this volume
  CylinderRing positionSpace;

  //! Discretization in viewing direction
  /*! The steps a chosen such that the object size changes by
      \c stepInPixelGlobalDiscretization pixel. This is
      is obtained by calling \c ObjectCNSStereoDetector (object2World, stepInPixelGlobalDiscretization, DIM_TRANS_VIEWING);
   */
  double stepInPixelGlobalDiscretization = 6;

  //! Step size during refinement
  double stepInPixelDuringRefinement = 3;

  //! List of different orientations searched for
  std::vector<Eigen::Isometry3d> object2WorldOrientation;

  //! How many objects to return (the \c nResponses best responses)
  int nResponses = 1;

  //! How many iterations of \c refine to perform on the results of the global search
  int nRefineIterations = 3;

  //! If \c true, object frames passed to search are additionally refined
  bool refineExisting = true;

  //! A block of \c blockX*blockY pixel is search with the same rasterized shape
  /*! Must be a multiple of 16 for technical reasons. */
  int blockX = 64;
  int blockY = 48;
};
