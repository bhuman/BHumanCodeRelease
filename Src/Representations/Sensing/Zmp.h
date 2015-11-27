#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/RotationMatrix.h"

STREAMABLE(Zmp,
{
  private:
    void drawFoot(const Pose2f& offset, const float scale, const bool left) const;

  public:
    void draw() const,

  (Vector2f)(Vector2f::Zero()) comInLeftSole, //just for testing
  (Vector2f)(Vector2f::Zero()) comInRightSole,//just for testing
  (Vector2f)(Vector2f::Zero()) zmpInLeftSole, /**<The zmp assuming that the left foot is the support foot*/
  (Vector2f)(Vector2f::Zero()) zmpInRightSole, /**<The zmp assuming that the right foot is the support foot*/
  (Vector2f)(Vector2f::Zero()) predComInLeftSole,
  (Vector2f)(Vector2f::Zero()) predComInRightSole,
  (Vector2f)(Vector2f::Zero()) rotZmpInLeftSole,
  (Vector2f)(Vector2f::Zero()) rotZmpInRightSole,
  (Vector2f)(Vector2f::Zero()) jointZmpInLeftSole,
  (Vector2f)(Vector2f::Zero()) jointZmpInRightSole,
});
