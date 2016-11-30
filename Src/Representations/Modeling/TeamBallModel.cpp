/**
 * @file Representations/Modeling/TeamBallModel.cpp
 *
 * Implementation of drawings for the team ball model
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "TeamBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void TeamBallModel::verify() const
{
  if(isValid)
  {
    ASSERT(std::isfinite(position.x()));
    ASSERT(std::isfinite(position.y()));
    ASSERT(std::isfinite(velocity.x()));
    ASSERT(std::isfinite(velocity.y()));
  }
}

void TeamBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamBallModel", "drawingOnField");

  DEBUG_DRAWING3D("representation:TeamBallModel", "field")
  {
    const Vector3f ballPos3d = Vector3f(position.x(), position.y(), 0.0f);
    const Vector3f ballSpeed3d = Vector3f(velocity.x(), velocity.y(), 0.0f);
    SPHERE3D("representation:TeamBallModel", ballPos3d.x(), ballPos3d.y(), 35.f, 35.f, ColorRGBA(128, 64, 0));
    if(ballSpeed3d.squaredNorm() > 0.9f)
    {
      CYLINDERARROW3D("representation:TeamBallModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(128, 64, 0));
    }
  }

  COMPLEX_DRAWING("representation:TeamBallModel")
  {
    CIRCLE("representation:TeamBallModel", position.x(), position.y(), 30, 20, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::black);
    ARROW("representation:TeamBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::blue);
    DRAWTEXT("representation:TeamBallModel", 1000, 1000, 200, ColorRGBA::black, isValid);
  }
}
