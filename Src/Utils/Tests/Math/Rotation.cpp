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

  EXPECT_TRUE(rot.isApprox(zRot * xyRot)); // Factored matrix can be reconstruted.
  const Vector3f z = Vector3f::UnitZ();
  EXPECT_TRUE(z.isApprox(zRot * z)); // Rotation axis of Qz is actually z.
  const Vector3f xy = Rotation::AngleAxis::pack(AngleAxisf(xyRot));
  EXPECT_LE(xy.z(), 1e-6f); // Qxy does not rotate around z.
  const Eigen::Vector3f QTz = rot.inverse() * z;
  const Eigen::Vector3f QxyTz = xyRot.inverse() * z;
  EXPECT_TRUE(QTz.isApprox(QxyTz)); // With respect to the vertical Q and Qxy are the same.
}

TEST(Rotation, removeZ)
{
  test_splitZRotation(Quaternionf::Identity());
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi * 3.f / 4.f, 0, 0))));
  test_splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi * 3.f / 4.f, 0, pi / 4.f))));
  for(int i = 0; i < 1000; ++i)
  {
    test_splitZRotation(Quaternionf(AngleAxisf(randomFloat(-pi, pi), Vector3f::Random().normalized())));
  }
  //Rotation::splitZRotation(Quaternionf(Rotation::AngleAxis::unpack(Vector3f(pi / 3.f, pi / 2.f, pi / 4.f))), xyRot, zRot);
  //EXPECT_TRUE(Vector3f::UnitZ().isApprox(zRot * Vector3f::UnitZ()));
}

TEST(Rotation, AngleAxisPackaging)
{
  for(int i = 0; i < 1000; ++i)
  {
    const AngleAxisf aaOrig = AngleAxisf(randomFloat(-pi, pi), Vector3f::Random().normalized());
    const AngleAxisf aa = Rotation::AngleAxis::unpack(Rotation::AngleAxis::pack(aaOrig));
    const Vector3f randomVec = Vector3f::Random();
    EXPECT_TRUE((aaOrig * randomVec).isApprox(aa * randomVec));
  }
}
