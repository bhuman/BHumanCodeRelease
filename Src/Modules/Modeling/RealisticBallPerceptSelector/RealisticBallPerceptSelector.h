/**
 * @file RealisticBallPerceptSelector.h
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/RealisticBallPercepts.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"

MODULE(RealisticBallPerceptSelector,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(PlayersPercept),
  USES(ObstacleModel),
  REQUIRES(RealisticBallPercepts),
  USES(BallModel),
  PROVIDES(BallPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(1000) robotFrontDisplacement,
    (Angle)(5_deg) robotSideAngleDisplacement,
    (int)(2000) timeUntilForget,
    (float)(300.f) maxDistanceFromLastModel,
  }),
});

/**
 * @class RealisticBallPerceptSelector
 */
class RealisticBallPerceptSelector : public RealisticBallPerceptSelectorBase
{
public:
  RealisticBallPerceptSelector();

private:
  struct RobotData
  {
  public:
    const float distance;
    const Angle start;
    const Angle end;
    RobotData(const float distance, const Angle start, const Angle end) : distance(distance), start(start), end(end) {}
  };
  std::vector<RobotData> robots;

  void update(BallPercept& ballPercept);

  void updateRobots();
};
