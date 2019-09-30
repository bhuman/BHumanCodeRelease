#pragma once

#include "Tools/ImageProcessing/SIMD.h"
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <vector>
#include <iostream>

//! Simple camera model class
/*
  Camera model class using the model also used in OpenCV:

  http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html

  This can been seen as stripped-down version of sensor-model. Some of the functions were taken from sensor-model.
 */

class CameraModelOpenCV
{
public:
  //! Create empty camera model
  CameraModelOpenCV();

  //! Constructor with all coefficients
  CameraModelOpenCV(const Eigen::Isometry3d& camera2World, double width, double height,
                    double scale_x, double scale_y, double offset_x, double offset_y,
                    double k1 = 0, double k2 = 0, double p1 = 0, double p2 = 0);

  //! Project ball in world coordinates into image plane
  bool worldBall2ImageCircleClipped(const Eigen::Vector3d& p_world, double worldRadius, double& x, double& y, double& r) const;

  //! Reconstruct ball in world coordinates from image coordinates and known ball radius (in world)
  void imageCircle2WorldBall(double x, double y, double radius, double worldRadius, Eigen::Vector3d& pBall) const;

  //! Construct ray that leads to the image position in camera coordinates
  void image2CameraRay(double x, double y, Eigen::Vector3d& p, Eigen::Vector3d& v) const;

  //! Construct ray that leads to the image position in world coordinates
  void image2WorldRay(double x, double y, Eigen::Vector3d& p, Eigen::Vector3d& v) const;

  //! Simple projection into image space from world coordinates
  void world2Image(const Eigen::Vector3d& p_world, double& x, double& y) const;

  //! Simple projection into image space from world coordinates (with clipping indication)
  bool world2ImageClipped(const Eigen::Vector3d& v, double& x, double& y) const;

  //! Simple projection into image space from camera coordinates
  void camera2Image(const Eigen::Vector3d& p_cam, double& x, double& y) const;

  //! Simple projection into image space from camera coordinates (with clipping indication)
  bool camera2ImageClipped(const Eigen::Vector3d& p_cam, double& x, double& y) const;

  //! Apply radial distortion at x, y
  void distort(double& xDist, double& yDist, double x, double y) const;

  /*! Overloaded function */
  void distort(double& x, double& y) const {distort(x, y, x, y); }

  //! Apply distortion at x, y and compute the Jacobian
  void distort(double& xDist, double& yDist, Eigen::Matrix2d& jac, double x, double y) const;

  //! Radial part of the distortion model
  double rPrimeOfR(double r) const {return r * (1 + k1 * r * r + k2 * r * r * r * r);}

  //! Inverse of \c rPrimeOfR
  /*! Uses linear interpolation on the pre-calculated table \c rOfRPrimeTab.
      If \c rPrime>rPrimeMax \c Nan is returned.
   */
  double rOfRPrime(double rPrime) const {return rOfRPrime2Factor(rPrime * rPrime) * rPrime;}

  enum {DEFAULT_REFINEMENT_CTR = 1};

  //! Computes undistorted image coordinates according to distortion parameters
  /*! There is no analytical solution, so an approximate formula is used.
      The result is refined by a \c refinementCtr step of Newton's method.
   */
  void undistort(double& xUndist, double& yUndist, double x, double y, int refinementCtr = DEFAULT_REFINEMENT_CTR) const;

  //! Overloaded function
  void undistort(double& x, double& y, int refinementCtr = DEFAULT_REFINEMENT_CTR) const {undistort(x, y, x, y, refinementCtr);};

  //! Set frame of camera in world coordinates
  void setCamera2World(const Eigen::Isometry3d& cam2world);

  //! Computes the table used for undistortion
  /*! Computes also both rMax and rMaxPrime. And sets \c isDistorted */
  void computeRTab();

  //! Return frame of camera in world coordinates
  Eigen::Isometry3d getCamera2World() const;

  //! Extends v1 to an orthonormal system [v1 v2 v3]
  void smOrthogonalVector(const Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& v3) const;
  Eigen::Vector3d getOrthoVec(Eigen::Vector3d v) const;

  //! Camera pose in world coordinates
  Eigen::Isometry3d camera2World;

  //! Pinhole camera parameters
  double width, height, scale_x, scale_y, offset_x, offset_y;

  //! distortion camera parameters (see openCV)
  double k1, k2, p1, p2;

  //! If in distortion r<rMax the distortion function is unique (similar for undistortion)
  double rMax, rMaxPrime;

  //! See \c rOfRPrimeTab
  double rPrime2Step;

  //! Precomputed table for computing \c rOfRPrimeFactor fast
  /*! \c rPrimeToR[i]=rOfRPrimeFactor(i*rPrime2Step).
      Values are available up to \c rMaxPrime*rMaxPrime.
   */
  std::vector<double> rOfRPrime2FactorTab;

  //! Whether this camera has distortion or not (coefficient as in the default constructor)
  bool isDistorted;

  //! auxiliary function for radial undistortion
  /*! If rPrime = rPrimeOfR(r), then \c rOfRPrimeFactor(rPrime*rPrime)*rPrime=r
   */
  double rOfRPrime2Factor(double rPrime2) const;

  //! Scales the calibration, corresponding to a scaled image
  void scale(double factor)
  {
    scale_x  *= factor;
    scale_y  *= factor;
    offset_x *= factor;
    offset_y *= factor;
    width    *= factor;
    height   *= factor;
  }

  //! Returns a rotation that moves the image center by \c dX,dY
  /*! Precisely, p(cameraRotation(dX,dY)*Vector3d(0,0,1)) \approx p(Vector3d(0,0,1))+(dX,dY)
   */
  Eigen::Isometry3d cameraRotation(double dX, double dY) const
  {
    Eigen::Vector3d axis(-dY / scale_y, dX / scale_x, 0);
    double angle = axis.norm();
    if(angle == 0)
      angle = 1E-9;
    return Eigen::Isometry3d(Eigen::AngleAxisd(angle, axis / angle));
  }

  //! Computes a coefficient, how much at undistorted (xU,yU) the distortion deviates from a flat mapping
  /*! The result is defined as max_{small dX,dY} |distort(xU+dX,yU+dY)-distort(xU,yU)|/|(dX, dY)|
   */
  double distortionAt(double xU, double yU) const;

  //! Computes the maximum \c distortionAt in the image
  double computeWorstDistortion() const;

  //! Returns whether \c x,y is in the image.
  bool isInImage(double x, double y) const {return 0 <= x && x < width && 0 <= y && y < height;}

  //! Returns a rotation of the camera that moves \c (xOld,yOld) to \c xNew,yNew
  /*! A point which maps to \c xOld,yOld in the old camera frame maps to \c xNew,yNew
      in the new camera frame and the result returned is the rotation from new to
  old camera frame that achieves it. Being a rotation this holds for all points
  in space. Several rotations have this property. Among them the one is chosen
  that has the smallest rotation angle.
   */
  Eigen::Isometry3d rotationNewCamera2OldCamera(double xOld, double yOld, double xNew, double yNew) const;

  //! Computes a camera that is in the middle and has averaged parameters
  static CameraModelOpenCV average(const CameraModelOpenCV& camA, const CameraModelOpenCV& camB);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Define input streaming
std::istream& operator>>(std::istream& i, CameraModelOpenCV& c);

//! Define output streaming
std::ostream& operator<<(std::ostream& o, const CameraModelOpenCV& c);

//! Returns the smallest rotation X, such that v1=X*v0
inline Eigen::Isometry3d fromTo(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1)
{
  double n0 = v0.norm(), n1 = v1.norm();
  if(n0 == 0 || n1 == 0)
    return Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis = v0.cross(v1);
  double len = axis.norm(), angle = atan2(len, v0.dot(v1));
  if(len == 0)
    return Eigen::Isometry3d::Identity();
  return Eigen::Isometry3d(Eigen::AngleAxisd(angle, axis / len));
}

//! Finds the closest Isometry3d to a given affine transformation
/*! The purpose of this function is to restore the orthonormality
    of the rotation part of the matrix. Call this function after
    loading Isometries and once in a while when numerical errors
    can accumulate.
 */
inline Eigen::Isometry3d normalize(const Eigen::Isometry3d& a2b)
{
  Eigen::Isometry3d result(a2b);
  result.linear() = a2b.rotation();
  return result;
}

//! Interpolates between two isometry \c a and \c b
/*! \c alpha=0 returns a, \c alpha=1 returns b, the orientation part
    uses spherical interpolation, the translation part is linear.
 */
Eigen::Isometry3d slerp(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double alpha);
