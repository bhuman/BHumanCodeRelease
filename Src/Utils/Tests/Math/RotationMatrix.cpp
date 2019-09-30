#include "Tools/Math/Random.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Angle.h"

#include "gtest/gtest.h"

#ifndef NDEBUG
#define RUNS 1000
#else
#define RUNS 1000000
#endif

GTEST_TEST(RotationMatrix, getAngleAxis)
{
  Vector3f vec(1, 2, 3);
  AngleAxisf aa(pi, Vector3f::UnitX());
  RotationMatrix r = aa;
  Vector3f r1 = aa * vec;
  Vector3f r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(pi - 0.000001f, Vector3f::UnitY());
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(pi, Vector3f::UnitZ());
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi, Vector3f::UnitX());
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi + 0.00001f, Vector3f::UnitY());
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi, Vector3f::UnitZ());
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  EXPECT_TRUE(r1.isApprox(r2));

  r = aa = AngleAxisf(-3.14060259f, Vector3f(-0.496929348f, 0.435349584f, 0.750687659f));
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  if(!r1.isApprox(r2, 1e-4f))
    EXPECT_TRUE(r1.isApprox(r2, 1e-4f));

  aa = AngleAxisf::Identity();
  r = RotationMatrix::Identity() * 0.99999f;
  r1 = aa * vec;
  r2 = r.getAngleAxis() * vec;
  if(!r1.isApprox(r2))
    EXPECT_TRUE(r1.isApprox(r2));

  for(int i = 0; i < RUNS; ++i)
  {
    vec = Vector3f::Random();
    r = aa = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    r1 = aa * vec;
    r2 = r.getAngleAxis() * vec;
    if(!r1.isApprox(r2, 1e-2f))
      EXPECT_TRUE(r1.isApprox(r2, 1e-2f))
          << "r1:\n"
          << r1 << "\n"
          << "r2\n"
          << r2 << "\n";
  }
}

GTEST_TEST(RotationMatrix, getPackedAngleAxisFaulty)
{
  Vector3f vec(1, 2, 3);
  AngleAxisf aa(pi - 0.0001f, Vector3f::UnitX());
  RotationMatrix r = aa;
  Vector3f r1 = aa * vec;
  Vector3f r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(pi - 0.0001f, Vector3f::UnitY());
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(pi - 0.0001f, Vector3f::UnitZ());
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi + 0.0001f, Vector3f::UnitX());
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi + 0.0001f, Vector3f::UnitY());
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(-pi + 0.0001f, Vector3f::UnitZ());
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));

  r = aa = AngleAxisf(-3.14060259f, Vector3f(-0.496929348f, 0.435349584f, 0.750687659f));
  r1 = aa * vec;
  r2 = Rotation::AngleAxis::unpack(r.getPackedAngleAxisFaulty()) * vec;
  EXPECT_FALSE(r1.isApprox(r2));
}

GTEST_TEST(RotationMatrix, normalize)
{
  for(int i = 0; i < RUNS; ++i)
  {
    const Vector3f rVec = Vector3f::Random();
    const AngleAxisf aa = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    const RotationMatrix r(aa);

    const Vector3f r1 = r * rVec;
    const Vector3f r2 = r.normalized() * rVec;
    EXPECT_TRUE(r1.isApprox(r2))
        << "r1:\n"
        << r1 << "\n"
        << "r2\n"
        << r2 << "\n";
  }
}

GTEST_TEST(RotationMatrix, rotateX)
{
  for(int i = 0; i < RUNS; ++i)
  {
    const Vector3f rVec = Vector3f::Random();
    const float rot = Random::uniform(-pi, pi);
    const AngleAxisf aa = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    const RotationMatrix q = aa * Rotation::aroundX(rot);
    const RotationMatrix r = RotationMatrix(aa).rotateX(rot);

    const Vector3f r1 = q * rVec;
    const Vector3f r2 = r * rVec;
    EXPECT_TRUE(r1.isApprox(r2))
        << "r1:\n"
        << r1 << "\n"
        << "r2\n"
        << r2 << "\n";
  }
}

GTEST_TEST(RotationMatrix, rotateY)
{
  for(int i = 0; i < RUNS; ++i)
  {
    const Vector3f rVec = Vector3f::Random();
    const float rot = Random::uniform(-pi, pi);
    const AngleAxisf aa = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    const RotationMatrix q = aa * Rotation::aroundY(rot);
    const RotationMatrix r = RotationMatrix(aa).rotateY(rot);

    const Vector3f r1 = q * rVec;
    const Vector3f r2 = r * rVec;
    EXPECT_TRUE(r1.isApprox(r2))
        << "r1:\n"
        << r1 << "\n"
        << "r2\n"
        << r2 << "\n";
  }
}

GTEST_TEST(RotationMatrix, rotateZ)
{
  for(int i = 0; i < RUNS; ++i)
  {
    const Vector3f rVec = Vector3f::Random();
    const float rot = Random::uniform(-pi, pi);
    const AngleAxisf aa = AngleAxisf(Random::uniform(-pi, pi), Vector3f::Random().normalized());
    const RotationMatrix q = aa * Rotation::aroundZ(rot);
    const RotationMatrix r = RotationMatrix(aa).rotateZ(rot);

    const Vector3f r1 = q * rVec;
    const Vector3f r2 = r * rVec;
    if(!r1.isApprox(r2))
      EXPECT_TRUE(r1.isApprox(r2))
          << "r1:\n"
          << r1 << "\n"
          << "r2\n"
          << r2 << "\n";
  }
}

GTEST_TEST(RotationMatrix, getXAngle)
{
  float angle = 0_deg;
  RotationMatrix r = RotationMatrix::aroundX(angle);
  float xAngle = r.getXAngle();
  EXPECT_FLOAT_EQ(xAngle, angle);

  angle = 90_deg;
  r = RotationMatrix::aroundX(angle);
  xAngle = r.getXAngle();
  EXPECT_FLOAT_EQ(xAngle, angle);

  angle = 180_deg;
  r = RotationMatrix::aroundX(angle);
  xAngle = r.getXAngle();
  EXPECT_NEAR(0.f, Angle::normalize(xAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = -180_deg;
  r = RotationMatrix::aroundX(angle);
  xAngle = r.getXAngle();
  EXPECT_NEAR(0.f, Angle::normalize(xAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = 360_deg;
  r = RotationMatrix::aroundX(angle);
  xAngle = r.getXAngle();
  EXPECT_FLOAT_EQ(xAngle, Angle::normalize(angle));

  for(int i = 0; i < RUNS; ++i)
  {
    angle = Random::uniform() * pi;
    r = RotationMatrix::aroundX(angle);
    xAngle = r.getXAngle();
    EXPECT_NEAR(xAngle, angle, 1e-3f);
  }
}

GTEST_TEST(RotationMatrix, getYAngle)
{
  float angle = 0_deg;
  RotationMatrix r = RotationMatrix::aroundY(angle);
  float yAngle = r.getYAngle();
  EXPECT_FLOAT_EQ(yAngle, angle);

  angle = 90_deg;
  r = RotationMatrix::aroundY(angle);
  yAngle = r.getYAngle();
  EXPECT_FLOAT_EQ(yAngle, angle);

  angle = 180_deg;
  r = RotationMatrix::aroundY(angle);
  yAngle = r.getYAngle();
  EXPECT_NEAR(0.f, Angle::normalize(yAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = -180_deg;
  r = RotationMatrix::aroundY(angle);
  yAngle = r.getYAngle();
  EXPECT_NEAR(0.f, Angle::normalize(yAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = 360_deg;
  r = RotationMatrix::aroundY(angle);
  yAngle = r.getYAngle();
  EXPECT_FLOAT_EQ(yAngle, Angle::normalize(angle));

  for(int i = 0; i < RUNS; ++i)
  {
    const float angle = Random::uniform() * pi;
    const RotationMatrix r = RotationMatrix::aroundY(angle);
    float yAngle = r.getYAngle();
    EXPECT_NEAR(yAngle, angle, 1e-3f);
  }
}

GTEST_TEST(RotationMatrix, getZAngle)
{
  float angle = 0_deg;
  RotationMatrix r = RotationMatrix::aroundZ(angle);
  float zAngle = r.getZAngle();
  EXPECT_FLOAT_EQ(zAngle, angle);

  angle = 90_deg;
  r = RotationMatrix::aroundZ(angle);
  zAngle = r.getZAngle();
  EXPECT_FLOAT_EQ(zAngle, angle);

  angle = 180_deg;
  r = RotationMatrix::aroundZ(angle);
  zAngle = r.getZAngle();
  EXPECT_NEAR(0.f, Angle::normalize(zAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = -180_deg;
  r = RotationMatrix::aroundZ(angle);
  zAngle = r.getZAngle();
  EXPECT_NEAR(0.f, Angle::normalize(zAngle - angle), 4.f * std::numeric_limits<float>::epsilon());

  angle = 360_deg;
  r = RotationMatrix::aroundZ(angle);
  zAngle = r.getZAngle();
  EXPECT_FLOAT_EQ(zAngle, Angle::normalize(angle));

  for(int i = 0; i < RUNS; ++i)
  {
    angle = Random::uniform() * pi;
    r = RotationMatrix::aroundZ(angle);
    zAngle = r.getZAngle();
    EXPECT_NEAR(zAngle, angle, 1e-3f);
  }
}
