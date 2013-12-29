/**
 * @file Representations/Modeling/CombinedWorldModel.cpp
 * Implementation of a debug drawing of the combined world model
 * @author Katharina Gillmann
 */

#include "CombinedWorldModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Vector3.h"

GaussianPositionDistribution::GaussianPositionDistribution(const Vector2<>& robotPosition, const Matrix2x2<>& covariance)
: robotPosition(robotPosition),
  covariance(covariance) {}

void CombinedWorldModel::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:CombinedWorldModel", "field",
    // draw opponent robots
    for(std::vector<GaussianPositionDistribution>::const_iterator i = positionsOpponentTeam.begin(); i != positionsOpponentTeam.end(); ++i)
    {
      float xExpansion, yExpansion, rotation;
      Covariance::errorEllipse(i->covariance, xExpansion, yExpansion, rotation);
      CYLINDER3D("representation:CombinedWorldModel", i->robotPosition.x, i->robotPosition.y, 0.0f, 0.0f, 0.0f, 0.0f, 35.0f, 20.0f, ColorRGBA(0, 0, 255, 100));
    }
    // draw own team
    for(std::vector<Pose2D>::const_iterator i = positionsOwnTeam.begin(); i != positionsOwnTeam.end(); ++i)
    {
      CYLINDER3D("representation:CombinedWorldModel", i->translation.x, i->translation.y, 0.0f, 0.0f, 0.0f, 0.0f, 35.0f, 60.0f, ColorRGBA(0, 0, 0, 100));
    }

    // draw global ball
    const Vector3<> ballPos3d = Vector3<>(ballStateOthers.position.x, ballStateOthers.position.y, 0.0f);
    const Vector3<> ballSpeed3d = Vector3<>(ballStateOthers.velocity.x, ballStateOthers.velocity.y, 0.0f);
    SPHERE3D("representation:CombinedWorldModel", ballPos3d.x, ballPos3d.y, 35.f, 35.f, ColorRGBA(128, 64, 0));
    if (ballSpeed3d.squareAbs() > 0.9f)
    {
      CYLINDERARROW3D("representation:CombinedWorldModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(128, 64, 0));
    }
  );



  DECLARE_DEBUG_DRAWING("representation:CombinedWorldModel", "drawingOnField");

  COMPLEX_DRAWING("representation:CombinedWorldModel",
  {
    for(std::vector<Pose2D>::const_iterator i = positionsOwnTeam.begin(); i != positionsOwnTeam.end(); ++i)
    {
      CROSS("representation:CombinedWorldModel", i->translation.x, i->translation.y, 20, 40, Drawings::ps_solid, ColorRGBA(ColorClasses::black));
      CIRCLE("representation:CombinedWorldModel", i->translation.x, i->translation.y, 500, 20, Drawings::ps_solid, ColorRGBA(ColorClasses::blue), Drawings::bs_null, ColorRGBA());
    }

    for(std::vector<GaussianPositionDistribution>::const_iterator i = positionsOpponentTeam.begin(); i != positionsOpponentTeam.end(); ++i)
    {
      float xExpansion, yExpansion, rotation;
      Covariance::errorEllipse(i->covariance, xExpansion, yExpansion, rotation);
      CROSS("representation:CombinedWorldModel", i->robotPosition.x, i->robotPosition.y, 20, 40, Drawings::ps_solid, ColorRGBA(ColorClasses::blue));
      /*ELLIPSE("representation:CombinedWorldModel", i->robotPosition , sqrt(3.0f) * xExpansion, sqrt(3.0f) * yExpansion, rotation,
        10, Drawings::ps_solid, ColorRGBA(100,100,255,100), Drawings::bs_solid, ColorRGBA(0,0,255,100));
      ELLIPSE("representation:CombinedWorldModel", i->robotPosition, sqrt(2.0f) * xExpansion, sqrt(2.0f) * yExpansion, rotation,
        10, Drawings::ps_solid, ColorRGBA(150,150,100,100), Drawings::bs_solid, ColorRGBA(0,255,0,100));
      ELLIPSE("representation:CombinedWorldModel", i->robotPosition, xExpansion, yExpansion, rotation,
        10, Drawings::ps_solid, ColorRGBA(255,100,100,100), Drawings::bs_solid, ColorRGBA(255,255,0,100));*/
      CIRCLE("representation:CombinedWorldModel", i->robotPosition.x, i->robotPosition.y, 600, 20, Drawings::ps_solid, ColorRGBA(ColorClasses::yellow), Drawings::bs_null, ColorRGBA());
    }

    CIRCLE("representation:CombinedWorldModel", ballState.position.x, ballState.position.y, 30, 20, Drawings::ps_solid, ColorRGBA(ColorClasses::blue), Drawings::bs_null, ColorRGBA());
    ARROW("representation:CombinedWorldModel", ballState.position.x, ballState.position.y, ballState.position.x + ballState.velocity.x, ballState.position.y + ballState.velocity.y, 5, 1, ColorRGBA(ColorClasses::blue));
    //CIRCLE("representation:CombinedWorldModel", expectedEndPosition.x, expectedEndPosition.y, 30, 20,Drawings::ps_solid, ColorRGBA(ColorClasses::red),Drawings::bs_null, ColorRGBA());
    DRAWTEXT("representation:CombinedWorldModel", 1000, 1000, 200, ColorRGBA(ColorClasses::black), ballIsValid);
  });

  COMPLEX_DRAWING("representation:CombinedWorldModel",
  {
    CIRCLE("representation:CombinedWorldModel", ballStateOthers.position.x, ballStateOthers.position.y, 30, 20, Drawings::ps_solid, ColorRGBA(ColorClasses::red), Drawings::bs_null, ColorRGBA());
    ARROW("representation:CombinedWorldModel", ballStateOthers.position.x, ballStateOthers.position.y, ballStateOthers.position.x + ballStateOthers.velocity.x, ballStateOthers.position.y + ballStateOthers.velocity.y, 5, 1, ColorRGBA(ColorClasses::red));
  });
}

