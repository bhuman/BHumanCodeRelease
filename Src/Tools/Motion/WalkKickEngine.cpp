#include "WalkKickEngine.h"
#include "Tools/Streams/InStreams.h"

WalkKickEngine::WalkKickEngine(const WalkKicks& walkKicks) :
  walkKicks(walkKicks)
{}

void WalkKickEngine::start(WalkKicks::Type kickType, Legs::Leg kickLeg)
{
  currentKickType = kickType;
  currentKickLeg = kickLeg;
  const WalkKick& kick = walkKicks.kicks[kickType];

  // init controlPoints vector
  std::vector<Vector2f> controlPoints;
  controlPoints.reserve(kick.keyFrames.size() + 2);
  controlPoints.emplace_back(0.f, 0.f);
  for(size_t i = 0; i < kick.keyFrames.size(); ++i)
    controlPoints.emplace_back(kick.keyFrames[i].phase, 0.f);
  controlPoints.emplace_back(1.f, 0.f);

  float sign = kickLeg == kick.kickLeg ? 1.f : -1.f;

  // x splines
  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = kick.keyFrames[j].position.x();
  positionSplines[0].initClamped(controlPoints, 0.f, 0.f);

  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = sign * kick.keyFrames[j].rotation.x();
  rotationSplines[0].initClamped(controlPoints, 0.f, 0.f);

  // y splines
  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = sign * kick.keyFrames[j].position.y();
  positionSplines[1].initClamped(controlPoints, 0.f, 0.f);

  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = kick.keyFrames[j].rotation.y();
  rotationSplines[1].initClamped(controlPoints, 0.f, 0.f);

  // z splines
  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = kick.keyFrames[j].position.z();
  positionSplines[2].initClamped(controlPoints, 0.f, 0.f);

  for(size_t j = 0; j < kick.keyFrames.size(); ++j)
    controlPoints[j + 1].y() = sign * kick.keyFrames[j].rotation.z();
  rotationSplines[2].initClamped(controlPoints, 0.f, 0.f);
}

void WalkKickEngine::getState(float phase, Vector3f& position, Vector3f& rotation)
{
  for(int i = 0; i < 3; ++i)
  {
    position[i] = positionSplines[i](phase);
    rotation[i] = rotationSplines[i](phase);
  }
}
