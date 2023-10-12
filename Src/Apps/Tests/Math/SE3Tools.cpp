#include "Math/SE3Tools.h"
#include "Math/Random.h"

#include <gtest/gtest.h>
#include <limits>

GTEST_TEST(SE3Tools, veeInversesHat)
{
  Vector6f se3Vec;
  se3Vec << 0, 0, 0, 0, 0, 1;

  Matrix4f m;
  m << 0, -1, 0, 0,
      1, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  EXPECT_TRUE(SE3::hat(se3Vec) == m);
  EXPECT_EQ(se3Vec, SE3::vee(m));

  for(unsigned i = 0; i < 1000; i++)
  {
    const Vector6f vec(Vector6f::Random());
    EXPECT_EQ(vec, SE3::vee(SE3::hat(vec)));
  }
}

GTEST_TEST(SE3Tools, skewSymmetricMatrix)
{
  for(unsigned n = 0; n < 1000; n++)
  {
    const Vector3f vec(Vector3f::Random());
    auto skew = SE3::skewMatrix(vec);

    EXPECT_EQ(0, skew.trace());

    for(unsigned i = 0; i < skew.cols(); i++)
    {
      for(unsigned j = 0; j < skew.rows(); j++)
      {
        EXPECT_EQ(skew(i, j), -skew(j, i));
      }
    }

    EXPECT_EQ(SE3::skewToVector(skew), vec);
  }
}

GTEST_TEST(SE3Tools, lnInversesExp)
{
  Vector6f se3Vec;

  // rotate z
  se3Vec << 0, 0, 0, 0, 0, Constants::pi;
  Pose3f pose {};
  pose.rotation.rotateZ(Constants::pi);
  auto result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_NEAR((se3Vec - SE3::ln(pose)).squaredNorm(), 0.f, std::numeric_limits<float>::epsilon());

  //rotate y
  se3Vec << 0, 0, 0, 0, 1, 0;
  pose = Pose3f();
  pose.rotation.rotateY(1);
  result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_NEAR((se3Vec - SE3::ln(pose)).squaredNorm(), 0.f, std::numeric_limits<float>::epsilon());

  //rotate x
  se3Vec << 0, 0, 0, 1, 0, 0;
  pose = Pose3f();
  pose.rotation.rotateX(1);
  result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_NEAR((se3Vec - SE3::ln(pose)).squaredNorm(), 0.f, std::numeric_limits<float>::epsilon());

  // translate x
  se3Vec << 1, 0, 0, 0, 0, 0;
  pose = Pose3f(Vector3f(1, 0, 0));
  result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_EQ(se3Vec, SE3::ln(pose));

  // translate y
  se3Vec << 0, 1, 0, 0, 0, 0;
  pose = Pose3f(Vector3f(0, 1, 0));
  result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_EQ(se3Vec, SE3::ln(pose));

  // translate z
  se3Vec << 0, 0, 1, 0, 0, 0;
  pose = Pose3f(Vector3f(0, 0, 1));
  result = SE3::exp(se3Vec);
  EXPECT_EQ(pose, result) << "Expected: " << pose.translation.transpose() << pose.rotation.getPackedAngleAxis().transpose()
                          << "\n Actual:" << result.translation.transpose() << result.rotation.getPackedAngleAxis().transpose();
  EXPECT_EQ(se3Vec, SE3::ln(pose));

  for(unsigned i = 0; i < 1000; i++)
  {
    Vector6f vec(Vector6f::Random());
    auto result = SE3::ln(SE3::exp(vec));
    EXPECT_TRUE(vec.isApprox(result)) << "Expected: " << vec.transpose()
                                      << "\nActual: " << result.transpose();
  }
}
