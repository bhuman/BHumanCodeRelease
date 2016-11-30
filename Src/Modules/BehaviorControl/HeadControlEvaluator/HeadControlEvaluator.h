/**
 * @author <a href="mailto:andisto@tzi.de">Andreas Stolpmann</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/HeadControlEvaluation.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"

MODULE(HeadControlEvaluator,
{,
  REQUIRES(BallPercept),
  REQUIRES(CirclePercept),
  REQUIRES(FrameInfo),
  REQUIRES(FieldLineIntersections),
  REQUIRES(FieldLines),
  REQUIRES(PlayersPercept),
  REQUIRES(OwnTeamInfo),
  PROVIDES(HeadControlEvaluation),
});

class HeadControlEvaluator : public HeadControlEvaluatorBase
{
public:
  void update(HeadControlEvaluation& headControlEvaluation);
};
