#include "CameraModelOpenCV.h"
#include <Eigen/SVD>
#include <limits>

using namespace std;

CameraModelOpenCV::CameraModelOpenCV()
  : camera2World(Eigen::Isometry3d::Identity()),
    width(0), height(0), scale_x(0), scale_y(0), offset_x(0), offset_y(0),
    k1(0), k2(0), p1(0), p2(0), rMax(0), rMaxPrime(0), isDistorted(false)
{
}

CameraModelOpenCV::CameraModelOpenCV(const Eigen::Isometry3d& camera2World, double width, double height,
                                     double scale_x, double scale_y, double offset_x, double offset_y,
                                     double k1, double k2, double p1, double p2)
  : camera2World(normalize(camera2World)), width(width), height(height), scale_x(scale_x), scale_y(scale_y),
    offset_x(offset_x), offset_y(offset_y), k1(k1), k2(k2), p1(p1), p2(p2)
{
  computeRTab();
}

bool CameraModelOpenCV::worldBall2ImageCircleClipped(const Eigen::Vector3d& p_world, double worldRadius, double& x, double& y, double& r) const
{
  // Isometry3d point into camera coordinates
  Eigen::Vector3d p_cam(camera2World.inverse() * p_world);

  x = y = r = numeric_limits<double>::quiet_NaN();
  if(p_cam(2) < worldRadius)
    return false;
  Eigen::Vector3d v2, v3;
  smOrthogonalVector(p_cam, v2, v3);
  double x1, y1, x2, y2, x3, y3, x4, y4;
  camera2Image(p_cam + v2 * worldRadius, x1, y1);
  if(!isfinite(x1) || !isfinite(y1))
    return false;
  camera2Image(p_cam - v2 * worldRadius, x2, y2);
  if(!isfinite(x2) || !isfinite(y2))
    return false;
  camera2Image(p_cam + v3 * worldRadius, x3, y3);
  if(!isfinite(x3) || !isfinite(y3))
    return false;
  camera2Image(p_cam - v3 * worldRadius, x4, y4);
  if(!isfinite(x4) || !isfinite(y4))
    return false;
  x = (x1 + x2 + x3 + x4) / 4;
  y = (y1 + y2 + y3 + y4) / 4;
  r = (x - x1) * (x - x1) + (y - y1) * (y - y1)
      + (x - x2) * (x - x2) + (y - y2) * (y - y2)
      + (x - x3) * (x - x3) + (y - y3) * (y - y3)
      + (x - x4) * (x - x4) + (y - y4) * (y - y4);
  r = sqrt(r / 4);
  return ((0 <= x) && (x < width) && (0 <= y) && (y < height));
}

void CameraModelOpenCV::imageCircle2WorldBall(double x, double y, double radius, double worldRadius, Eigen::Vector3d& pBall) const
{
  Eigen::Vector3d p, v1, v2, v3, v4;
  image2CameraRay(x + radius, y, p, v1);
  v1.normalize();
  image2CameraRay(x - radius, y, p, v2);
  v2.normalize();
  image2CameraRay(x, y + radius, p, v3);
  v3.normalize();
  image2CameraRay(x, y - radius, p, v4);
  v4.normalize();
  Eigen::Vector3d v = 0.25 * (v1 + v2 + v3 + v4);
  double wR = (v1 - v).squaredNorm() + (v2 - v).squaredNorm() + (v3 - v).squaredNorm() + (v4 - v).squaredNorm();
  wR = sqrt(wR / 4);
  // pBallCamera,wR would be the ball fitting (x,y,radius) if it was 1 away from the camera center
  Eigen::Vector3d pBallCamera = p + worldRadius / wR * v;

  pBall = camera2World * pBallCamera;
}

void CameraModelOpenCV::image2CameraRay(double x, double y, Eigen::Vector3d& p, Eigen::Vector3d& v) const
{
  x = (x - offset_x) / scale_x;
  y = (y - offset_y) / scale_y;
  p = Eigen::Vector3d(0, 0, 0);
  undistort(x, y);
  v = Eigen::Vector3d(x, y, 1.0);
  v *= 1 / v.norm();
}

void CameraModelOpenCV::image2WorldRay(double x, double y, Eigen::Vector3d& p, Eigen::Vector3d& v) const
{
  image2CameraRay(x, y, p, v);
  p = camera2World * p;
  v = camera2World.linear() * v;
}

void CameraModelOpenCV::world2Image(const Eigen::Vector3d& p_world, double& x, double& y) const
{
  // Isometry3d point into camera coordinates
  Eigen::Vector3d p_cam(camera2World.inverse() * p_world);

  camera2Image(p_cam, x, y);
}

bool CameraModelOpenCV::world2ImageClipped(const Eigen::Vector3d& p_world, double& x, double& y) const
{
  // Isometry3d point into camera coordinates
  Eigen::Vector3d p_cam(camera2World.inverse() * p_world);

  // Project into image and return
  return camera2ImageClipped(p_cam, x, y);
}

bool CameraModelOpenCV::camera2ImageClipped(const Eigen::Vector3d& p_cam, double& x, double& y) const
{
  if(p_cam(2) <= 0)
    return false;
  else
  {
    // Projection
    x = p_cam(0) / p_cam(2);
    y = p_cam(1) / p_cam(2);

    // Distortion
    distort(x, y);

    // Scaling and offset
    x = scale_x * x + offset_x;
    y = scale_y * y + offset_y;
  }

  return ((0 <= x) && (x < width) && (0 <= y) && (y < height));
}

void CameraModelOpenCV::camera2Image(const Eigen::Vector3d& p_cam, double& x, double& y) const
{
  if(p_cam(2) >= 0)
  {
    x = p_cam(0) / p_cam(2);
    y = p_cam(1) / p_cam(2);
    distort(x, y);

    x = scale_x * x + offset_x;
    y = scale_y * y + offset_y;
  }
  else
  {
    x = y = numeric_limits<float>::quiet_NaN();
  }
}

void CameraModelOpenCV::distort(double& xDist, double& yDist, double x, double y) const
{
  if(!isDistorted)
  {
    xDist = x;
    yDist = y;
    return;
  }
  double r2 = x * x + y * y;
  if(r2 > rMax * rMax) // singularity or not unique
  {
    xDist = numeric_limits<float>::quiet_NaN();
    yDist = numeric_limits<float>::quiet_NaN();
    return;
  }

  double rFactor = 1 + k1 * r2 + k2 * r2 * r2;
  double newX = x * rFactor + 2 * p1 * x * y        +   p2 * (r2 + 2 * x * x);
  double newY = y * rFactor +   p2 * (r2 + 2 * y * y) + 2 * p1 * x * y;

  xDist = newX;
  yDist = newY;
}

void CameraModelOpenCV::distort(double& xDist, double& yDist, Eigen::Matrix2d& jac, double x, double y) const
{
  double r2 = x * x + y * y;
  if(r2 > rMax * rMax) // singularity or not unique
  {
    xDist = numeric_limits<float>::quiet_NaN();
    yDist = numeric_limits<float>::quiet_NaN();
    return;
  }

  double rFactor = 1 + k1 * r2 + k2 * r2 * r2;
  double newX = x * rFactor + 2 * p1 * x * y        +   p2 * (r2 + 2 * x * x);
  double newY = y * rFactor +   p2 * (r2 + 2 * y * y) + 2 * p1 * x * y;

  double dR2DX      = 2 * x, dR2DY = 2 * y;
  double dRFactorDR2 = k1 + k2 * 2 * r2;
  double dRFactorDX = dRFactorDR2 * dR2DX, dRFactorDY = dRFactorDR2 * dR2DY;
  jac <<
      rFactor + x* dRFactorDX + 2 * p1* y + p2*(dR2DX + 4 * x), x* dRFactorDY + 2 * p1* x + p2* dR2DY,
              y* dRFactorDX + p2* dR2DX + 2 * p1* y,                 rFactor + y* dRFactorDY + p2*(dR2DY + 4 * y) + 2 * p1* x;

  xDist = newX;
  yDist = newY;
}

void CameraModelOpenCV::undistort(double& xUndist, double& yUndist, double x, double y, int refinementCtr) const
{
  if(!isDistorted)
  {
    xUndist = x;
    yUndist = y;
    return;
  }
  double r2 = x * x + y * y;
  if(r2 > rMaxPrime * rMaxPrime) // singularity or not unique
  {
    xUndist = numeric_limits<float>::quiet_NaN();
    yUndist = numeric_limits<float>::quiet_NaN();
    return;
  }
  // Warning, this is an approximate inversion
  double rFactor = rOfRPrime2Factor(r2);
  double xNew = x * rFactor;
  double yNew = y * rFactor;
  double r2New = xNew * xNew + yNew * yNew;
  xNew +=  - 2 * p1 * xNew * yNew        - p2 * (r2New + 2 * xNew * xNew);
  yNew += - p2 * (r2New + 2 * yNew * yNew) - 2 * p1 * xNew * yNew;

  for(int i = 0; i < refinementCtr; i++)
  {
    double xReproj, yReproj;
    Eigen::Matrix2d jac;
    distort(xReproj, yReproj, jac, xNew, yNew);
    Eigen::Vector2d correction = jac.inverse() * (Eigen::Vector2d(x - xReproj, y - yReproj));
    xNew += correction(0);
    yNew += correction(1);
  }

  xUndist = xNew;
  yUndist = yNew;
}

double CameraModelOpenCV::rOfRPrime2Factor(double rPrime2) const
{
  assert(rPrime2 >= 0);
  double lambda = rPrime2 / rPrime2Step;
  int rIndex = static_cast<int>(floor(lambda));
  lambda -= rIndex;
  assert(0 <= lambda && lambda <= 1);
  if(rIndex + 1 >= static_cast<int>(rOfRPrime2FactorTab.size()))
    return numeric_limits<float>::quiet_NaN();
  return (1 - lambda) * rOfRPrime2FactorTab[rIndex] + lambda * rOfRPrime2FactorTab[rIndex + 1];
}

void CameraModelOpenCV::setCamera2World(const Eigen::Isometry3d& _camera2World)
{
  camera2World = _camera2World;
}

Eigen::Isometry3d CameraModelOpenCV::getCamera2World() const
{
  return camera2World;
}

void CameraModelOpenCV::smOrthogonalVector(const Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& v3) const
{
  v2 = getOrthoVec(v1);
  v3 = v1.cross(v2);
  v3.normalize();
}

Eigen::Vector3d CameraModelOpenCV::getOrthoVec(Eigen::Vector3d v) const
{
  Eigen::Vector3d res;

  if(fabs(v(0)) < fabs(v(1)))
    if(fabs(v(2)) < fabs(v(0)))
      res =  Eigen::Vector3d(0.0, 0.0, 1.0);
    else
      res = Eigen::Vector3d(1.0, 0.0, 0.0);
  else if(fabs(v(2)) < fabs(v(1)))
    res = Eigen::Vector3d(0.0, 0.0, 1.0);
  else
    res = Eigen::Vector3d(0.0, 1.0, 0.0);

  if(v.norm() <  1E-9)
    return res;

  res = res.cross(v);
  res.normalize();
  return res;
}

void CameraModelOpenCV::computeRTab()
{
  isDistorted = (k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0);
  if(!isDistorted)
  {
    rMax = rMaxPrime = numeric_limits<double>::infinity();
    rOfRPrime2FactorTab.clear();
    return;
  }

  double lastRDistorted = 0, lastR = 0;
  rPrime2Step = 0.01;
  double rStep = rPrime2Step / 100; // Be sure we don't make more than one rPrime2Step with each rStep
  rOfRPrime2FactorTab.clear();
  rOfRPrime2FactorTab.push_back(1);  // At r=0 the tabulated factor converges to 1
  for(double r = rStep; r < 4; r += rStep)
  {
    double rDistorted = r * (1 + k1 * r * r + k2 * r * r * r * r);
    if(rDistorted > lastRDistorted)
    {
      if(rDistorted * rDistorted - lastRDistorted * lastRDistorted > rPrime2Step)
        break;
      double nextROfRPrimeEntry = sqrt(rOfRPrime2FactorTab.size() * rPrime2Step);
      if(nextROfRPrimeEntry <= rDistorted)
      {
        // interpolate the next entry from lastRDistorted and rDistorted
        double lambda = (nextROfRPrimeEntry - lastRDistorted) / (rDistorted - lastRDistorted);
        assert(-0.01 <= lambda && lambda <= 1); // There can be very slight extrapolation to the left
        rOfRPrime2FactorTab.push_back(lambda * r / rDistorted + (1 - lambda)*lastR / lastRDistorted);
      }
      lastRDistorted = rDistorted;
      lastR          = r;
    }
    else
      break;
  }
  rMaxPrime = sqrt((rOfRPrime2FactorTab.size() - 1) * rPrime2Step);
  rMax = rOfRPrime2FactorTab.back() * rMaxPrime;
  assert(rMaxPrime > 0.1 && rMax > 0.1);
}

double CameraModelOpenCV::distortionAt(double xU, double yU) const
{
  double x, y;
  Eigen::Matrix2d jac;
  distort(x, y, jac, xU, yU);
  Eigen::Matrix2d A;
  A <<
    jac(0, 0) - 1, jac(0, 1),
        jac(1, 0),   jac(1, 1) - 1;
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.singularValues()(1);
}

double CameraModelOpenCV::computeWorstDistortion() const
{
  assert(false && "not implemented");
  return 0;
}

Eigen::Isometry3d CameraModelOpenCV::rotationNewCamera2OldCamera(double xOld, double yOld, double xNew, double yNew) const
{
  Eigen::Vector3d p, vOld, vNew;
  image2CameraRay(xOld, yOld, p, vOld);
  image2CameraRay(xNew, yNew, p, vNew);
  return fromTo(vNew, vOld);
}

Eigen::Isometry3d slerp(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double alpha)
{
  Eigen::Quaterniond qA = Eigen::Quaterniond(Eigen::AngleAxisd(a.rotation()));
  Eigen::Quaterniond qB = Eigen::Quaterniond(Eigen::AngleAxisd(a.rotation()));
  Eigen::Quaterniond q  = qA.slerp(alpha, qB);
  Eigen::Isometry3d result(q);
  result.translation() = a.translation() + alpha * (b.translation() - a.translation());
  return result;
}

CameraModelOpenCV CameraModelOpenCV::average(const CameraModelOpenCV& camA, const CameraModelOpenCV& camB)
{
  return CameraModelOpenCV(slerp(camA.camera2World, camB.camera2World, 0.5),
                           (camA.width + camB.width) / 2, (camA.height + camB.height) / 2,
                           (camA.scale_x + camB.scale_x) / 2, (camA.scale_y + camB.scale_y) / 2,
                           (camA.offset_x + camB.offset_x) / 2, (camA.offset_y + camB.offset_y) / 2,
                           (camA.k1 + camB.k1) / 2, (camA.k2 + camB.k2) / 2,
                           (camA.p1 + camB.p1) / 2, (camA.p2 + camB.p2) / 2);
}

istream& operator>>(istream& input, CameraModelOpenCV& c)
{
  Eigen::Matrix4d camera2WorldHom;
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
    {
      input >> camera2WorldHom(i, j);
    }
  c.camera2World = Eigen::Isometry3d(camera2WorldHom);
  input >> c.width >> c.height >> c.scale_x >> c.scale_y >> c.offset_x >> c.offset_y >> c.k1 >> c.k1 >> c.p1 >> c.p2;
  c.computeRTab();

  return input;
}

ostream& operator<<(ostream& o, const CameraModelOpenCV& c)
{
  o << c.camera2World.matrix() << c.width << c.height << c.scale_x << c.scale_y << c.offset_x << c.offset_y << c.k1 << c.k2 << c.p1 << c.p2;
  return o;
}
