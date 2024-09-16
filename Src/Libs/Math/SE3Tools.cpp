/**
 * @file SE3Tools.cpp
 *
 * Implementation of some tools for the SE3 Lie-group and the associated algebra se3
 *
 * The vector representation of elements from the algebra has the translational parameters as the first three elements and the rotational as the next three
 * [translation, rotation].transpose
 * this is important to construct the se3 element from the vector as well as for the construction of the adjoint matrix
 *
 * Sources:
 * [1] J.-L. Blanco, “A tutorial on SE(3) transformation parameterizations and on-manifold optimization,” MAPIR: Grupo de Percepci ́ on y Rob ́ otica Dpto. de Ingenier ́ıa de Sistemas y Autom ́atica, Málaga, Technical #012010, Oct. 2014. Accessed: May 09, 2023. [Online]. Available: http://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
 * [2] L. Meyer, K. H. Strobl, and R. Triebel, “The Probabilistic Robot Kinematics Model and its Application to Sensor Fusion,” in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2022, pp. 3263–3270. doi: 10.1109/IROS47612.2022.9981399.
 *
 * @author Yannik Meinken
 */

#include "SE3Tools.h"

namespace SE3
{
  Matrix3f skewMatrix(const Vector3f& vec)
  {
    // [1]
    Matrix3f skew;
    skew << 0.f, -vec.z(), vec.y(),
        vec.z(), 0.f, -vec.x(),
        -vec.y(), vec.x(), 0.f;
    return skew;
  }

  Vector3f skewToVector(const Matrix3f& m)
  {
    Vector3f s;
    s << m(2, 1), m(0, 2), m(1, 0);

    return s;
  }

  Matrix6f adjoint(const Pose3f& pose)
  {
    // [2]
    Matrix6f adj;
    adj << pose.rotation, skewMatrix(pose.translation) * pose.rotation,
        Matrix3f::Zero(), pose.rotation;

    return adj;
  }

  Matrix4f hat(const Vector6f& s)
  {
    // [2]
    Matrix4f se;
    se << skewMatrix(s.tail<3>()), s.head<3>(),
        Vector4f::Zero().transpose();
    return se;
  }

  Vector6f vee(const Matrix4f& se)
  {
    Vector6f s;
    s << se.col(3).head(3), skewToVector(se.block<3, 3>(0, 0));

    return s;
  }

  Pose3f exp(const Vector6f& s)
  {
    // [1]
    const Angle theta = s.tail(3).norm();
    const auto skew = skewMatrix(s.tail(3));

    auto R = RotationMatrix(Matrix3f::Identity());
    auto V = Matrix3f(Matrix3f::Identity());
    if(theta != 0.f)
    {
      R += std::sin(theta) / theta * skew + (1 - std::cos(theta)) / (theta * theta) * skew * skew;
      V += (1 - std::cos(theta)) / (theta * theta) * skew + (theta - std::sin(theta)) / (theta * theta * theta) * skew * skew;
    }

    return Pose3f(R, V * s.head(3));
  }

  Vector6f ln(const Pose3f& p)
  {
    Pose3f x(p.rotation.normalized(), p.translation);
    // [1]
    Matrix3f skew;
    auto V = Matrix3f(Matrix3f::Identity());
    const float trace = x.rotation.trace();
    const Angle theta = std::acos(0.5f * (trace - 1));

    if(theta == 0)
      skew = Matrix3f::Identity();
    else
    {
      if(trace == -1.f)
      {
        // find the diagonal element that is not -1 to avoid division by 0
        auto nonZeroIndex = 0;
        auto maxDiagonalElement = std::abs(x.rotation(0, 0) + 1);
        for(auto i = 1; i <= x.rotation.rows(); i++)
        {
          if(std::abs(x.rotation(i, i) + 1) > maxDiagonalElement)
          {
            nonZeroIndex = i;
            maxDiagonalElement = std::abs(x.rotation(i, i) + 1);
          }
        }

        // Shivesh Kumar "Modern Robot Control Architectures, Lecture 4" (2022)
        Vector3f one = Vector3f::Zero();
        one[nonZeroIndex] = 1;
        skew = theta * skewMatrix(1 / sqrt(2 * (1 + x.rotation(nonZeroIndex, nonZeroIndex))) * (x.rotation.block<3, 1>(0, nonZeroIndex) + one));
      }
      else
        skew = theta / (2 * std::sin(theta)) * (x.rotation - x.rotation.inverse());
      V += (1 - std::cos(theta)) / (theta * theta) * skew + (theta - std::sin(theta)) / (theta * theta * theta) * skew * skew;
    }

    Vector6f v;
    v << V.inverse() * x.translation, skewToVector(skew);
    return v;
  }
}
