/**
 * @file DribbleTargetProvider.h
 *
 * This file declares a module that calculates the ball position after the execution of the DribbleToGoal-Skill.
 *
 * @author Nico Holsten
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/SectorWheel.h"

MODULE(DribbleTargetProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FieldRating),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  PROVIDES(DribbleTarget),
  DEFINES_PARAMETERS(
  {,
    (int)(100) iterationSteps, /**< Num of iteration steps. */
    (float)(10.f) stepLength, /**< Iterate this much per step. */
    (float)(1200.f) obstacleDistance, /**< Ignore obstacles further away than this distance. */
    (Angle)(3_deg) minFreeSector,
    (float)(1000.f) dribbleRange, /**< Planned look ahead dribble kick range. */
    (float)(500.f) safeDistanceToFieldBoarder, /**< Dribble range + this threshold shall not touch the field border. */
    (float)(15.f) searchStepDrawWidth,
    (float)(5.f) searchStepDrawScale,
    (int)(10) searchStepDrawModulo,
  }),
});

class DribbleTargetProvider : public DribbleTargetProviderBase
{
public:
  void update(DribbleTarget& theDribbleTarget) override;
  DribbleTargetProvider();

private:
  struct DribbleTargets
  {
    DribbleTargets(const Vector2f& startFieldPosition, const Vector2f& endFieldPosition, const Angle& dribbleAngle) :
      startFieldPosition(startFieldPosition), endFieldPosition(endFieldPosition), dribbleAngle(dribbleAngle) {};

    Vector2f startFieldPosition;
    Vector2f endFieldPosition;
    Angle dribbleAngle;
  };
  std::vector<DribbleTargets> checkedDribbleTargets;
  Angle calculateDribbleAngle(Vector2f position); // Intentional not a const reference
  float getDistanceToFieldBorder(const Vector2f& startPosition, const Angle& direction);
  void calculateSectorWheel();
  std::list<SectorWheel::Sector> kickAngles;
  Angle lastDribbleAngle;
  const Vector2f leftGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the opponent's goal.
  const Vector2f rightGoalPost = Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the opponent's goal.
  const Vector2f bottomRightField = Vector2f(theFieldDimensions.xPosOwnGoalLine + 10.f, theFieldDimensions.yPosRightTouchline + 10.f);
  const Vector2f topLeftField = Vector2f(theFieldDimensions.xPosOpponentGoalLine - 10.f, theFieldDimensions.yPosLeftTouchline - 10.f);
  const Rangef xMinMax = Rangef(theFieldDimensions.xPosOwnGoalLine + 10.f, theFieldDimensions.xPosOpponentGoalLine - 10.f);
  const Rangef yMinMax = Rangef(theFieldDimensions.yPosRightTouchline + 10.f, theFieldDimensions.yPosLeftTouchline - 10.f);
};
