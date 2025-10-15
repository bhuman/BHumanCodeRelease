/**
 * @file Representations/Modeling/TeamBallModel.cpp
 *
 * Implementation of drawings for the TeamBallModel
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 * @contributor Liam Hurwitz
 */

#include "TeamBallModel.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

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
  DEBUG_DRAWING3D("representation:TeamBallModel", "field")
  {
    if(isValid)
    {
      const Vector3f ballPos3d = Vector3f(position.x(), position.y(), 0.0f);
      const Vector3f ballSpeed3d = Vector3f(velocity.x(), velocity.y(), 0.0f);
      SPHERE3D("representation:TeamBallModel", ballPos3d.x(), ballPos3d.y(), 35.f, 35.f, ColorRGBA(128, 64, 0));
      if(ballSpeed3d.squaredNorm() > 0.9f)
      {
        CYLINDER_ARROW3D("representation:TeamBallModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(128, 64, 0));
      }
    }
  }

  DEBUG_DRAWING("representation:TeamBallModel", "drawingOnField")
  {
    if(isValid)
    {
      ColorRGBA teamBallColor(255, 0, 200, isValid ? 240 : 70);
      CIRCLE("representation:TeamBallModel", position.x(), position.y(), 100, 20, Drawings::solidPen, teamBallColor, Drawings::solidBrush, teamBallColor);
      ARROW("representation:TeamBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, teamBallColor);
    }
  }
}
