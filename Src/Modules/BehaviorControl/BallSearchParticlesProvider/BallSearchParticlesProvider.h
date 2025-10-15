/**
 * @file BallParticlesAreasProvider.h
 *
 * This file declares a module that describes particles on the field where the ball should be searched.
 *
 * @author Moritz Oppermann
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/BallSearchParticles.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Tools/Math/Projection.h"

MODULE(BallSearchParticlesProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(CameraMatrix),
  REQUIRES(ObstacleModel),
  REQUIRES(CameraInfo),
  REQUIRES(ExtendedGameState),
  REQUIRES(GameState),
  REQUIRES(FrameInfo),
  USES(BallSearchParticles),
  PROVIDES(BallSearchParticles),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(200) numOfParticles,
    (float)(3500.f) maxViewDistance,
    (float)(200) obstacleOffset,
    (unsigned)(500) cellWidth,
    (unsigned)(500) cellHeight,
    (int)(8000) refereeTolerance,
  }),
});

class BallSearchParticlesProvider : public BallSearchParticlesProviderBase
{
public:
  void update(BallSearchParticles&) override;
private:
  void draw() const;

  /**
   * Resets the particles according to the game state.
   * e.g. place the particles randomly during playing but place them into the corners during a corner kick.
   */
  void reset(BallSearchParticles&) const;

  /**
   * Creates random coordinates inside the field borders.
   * @return Random coordinates inside the field borders.
   */
  Vector2f createRandomParticle() const;

  /**
   * Calculate a list of sectors. Adds an obstacle sector for each obstacles.
   * @return A list of sectors.
   */
  std::list<SectorWheel::Sector> calculateObstacleSectors() const;

  /**
   * Determines whether a particle can be seen from the robot's perspective.
   * @param particle The particle.
   * @param polygon The part of the field that can be viewed from a robot's perspective.
   * @return Whether the particle can be seen.
   */
  bool isParticleInView(const Vector2f& particle, const std::vector<Vector2f>& polygon) const;

  /**
   * Determines whether the view to a particle is blocked by an obstacle.
   * @param particle The particle.
   * @param wheel The sector wheel with obstacles.
   * @return Whether the particle is blocked by an obstacle.
   */
  bool isBlockedByObstacle(const Vector2f& particle, const std::list<SectorWheel::Sector>& wheel) const;

  /**
   * Calculates the position the agent should look at next.
   * @param agent The agent.
   * @return The field coordinates where the agent should search next.
   */
  Vector2f positionToSearch(const Agent& agent) const;

  /**
   * Determines whether a particle is inside the agents voronoi region.
   * @param agent The agent.
   * @return Whether a particle is inside the agents voronoi region.
   */
  bool anyParticlesInRegion(const Agent& agent) const;
};
