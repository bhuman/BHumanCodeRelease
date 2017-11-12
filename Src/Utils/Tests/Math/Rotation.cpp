#include "Tools/Math/Random.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Angle.h"

#include "gtest/gtest.h"

inline static void test_splitZRotation(const Quaternionf& rot)
{
  Quaternionf zRot;
  const Quaternionf xyRot = Rotation::splitOffZRotation(rot, zRot);

  EXPECT_TRUE(rot.isApprox(zRot * xyRot)) << "zRot: \n" << zRot.vec() << "\n" << zRot.w() <<
                                          "\nxyRot: \n" << xyRot.vec() << "\n" << xyRot.w(); // Factored matrix can be reconstructed.
  const Vector3f z = Vector3f::UnitZ();
  EXPECT_TRUE(z.isApprox(zRot * z)) << "z: \n" << z <<
                                    "\nzRot: \n" << zRot.vec() << "\n" << zRot.w(); // Rotation axis of Qz is actually z.
  const Vector3f xy = Rotation::AngleAxis::pack(AngleAxisf(xyRot));
  EXPECT_LE(xy.z(), 1e-6f); // Qxy does not rotate around z.
  const Eigen::Vector3f QTz = rot.inverse() * z;
  const Eigen::Vector3f QxyTz = xyRot.inverse() * z;
  EXPECT_TRUE(QTz.isApprox(QxyTz)) << "QTz: \n" << QTz <<
                                   "\nQxyTz: \n" << QxyTz; // With respect to the vertical Q and Qxy are the same.
}

GTEST_TEST(Rotation, removeZ)
{
  test_splitZRotation(Quaternionf::Identity());
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi * 3.f / 4.f, 0, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi * 3.f / 4.f, 0, pi / 4.f))));
  for(int i = 0; i < 1000; ++i)
  {
    test_splitZRotation(Quaternionf(AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized())));
  }

  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, 0, 0.5))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, 0.0001f, 0.5))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi, 0, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, pi, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, 0, pi))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi / 2, 0, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, pi / 2, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, 0, pi / 2))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi * 2, 0, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, pi * 2, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, 0, pi * 2))));

  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(0, pi, 0.2f))));
}

GTEST_TEST(Rotation, AngleAxisPackaging)
{
  for(int i = 0; i < 1000; ++i)
  {
    const AngleAxisf aaOrig = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    const AngleAxisf aa = Rotation::AngleAxis::unpack(Rotation::AngleAxis::pack(aaOrig));
    const Vector3f randomVec = Vector3f::Random();
    EXPECT_TRUE((aaOrig * randomVec).isApprox(aa * randomVec)) << "randomVec: \n" << randomVec <<
        "\naaOrig.angle: " << aaOrig.angle() <<
        "\naaOrig.axis: \n" << aaOrig.axis() <<
        "\naa.angle: " << aa.angle() <<
        "\naa.axis: \n" << aa.axis();
  }
}
