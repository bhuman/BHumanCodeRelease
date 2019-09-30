/**
 * @file WalkKickEngine.cpp
 *
 * This file implements a module that provides a walk kick generator.
 *
 * @author Alexis Tsogias
 * @author Arne Hasselbring
 */

#include "WalkKickEngine.h"
#include "Tools/Streams/InStreams.h"
#include <ctype.h>
#include <regex>

MAKE_MODULE(WalkKickEngine, motionControl);

WalkKickEngine::WalkKickEngine()
{
  readWalkKicks();
  // Directly text-to-speech-synthesizing the enum names is hard to understand.
  // Therefore add a space before every camel case.
  FOREACH_ENUM(WalkKicks::Type, t)
    kickTypesSpeech[t] = std::regex_replace(TypeRegistry::getEnumName(t), std::regex("([A-Z])"), " $1");
}

void WalkKickEngine::update(WalkKickGenerator& walkKickGenerator)
{
  DEBUG_RESPONSE_ONCE("module:WalkKickEngine:reloadKicks")
    readWalkKicks();

  walkKickGenerator.start = [this, &walkKickGenerator](const WalkKickVariant& walkKickVariant) { start(walkKickGenerator, walkKickVariant); };
  walkKickGenerator.getState = [this](float phase, Vector3f& position, Vector3f& rotation) { getState(phase, position, rotation); };
}

void WalkKickEngine::readWalkKicks()
{
  FOREACH_ENUM(WalkKicks::Type, kick)
    if(kick != WalkKicks::none)
    {
      const std::string kickName = TypeRegistry::getEnumName(kick);
      InMapFile stream("WalkKicks/" + kickName + ".cfg");
      ASSERT(stream.exists());
      stream >> walkKicks[kick];
    }
}

void WalkKickEngine::start(WalkKickGenerator& walkKickGenerator, const WalkKickVariant& walkKickVariant)
{
  const WalkKick& kick = walkKicks[walkKickVariant.kickType];

  // init controlPoints vector
  std::vector<Vector2f> controlPoints;
  controlPoints.reserve(kick.keyFrames.size() + 2);
  controlPoints.emplace_back(0.f, 0.f);
  for(size_t i = 0; i < kick.keyFrames.size(); ++i)
    controlPoints.emplace_back(kick.keyFrames[i].phase, 0.f);
  controlPoints.emplace_back(1.f, 0.f);

  const float sign = walkKickVariant.kickLeg == kick.kickLeg ? 1.f : -1.f;

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

  walkKickGenerator.requiresPreStep = kick.requiresPreStep;
  walkKickGenerator.preStepSize = Pose2f(kick.preStepSize.rotation * sign, kick.preStepSize.translation.x(), kick.preStepSize.translation.y() * sign);
  walkKickGenerator.stepSize = Pose2f(kick.stepSize.rotation * sign, kick.stepSize.translation.x(), kick.stepSize.translation.y() * sign);

  if(playKickSounds)
    SystemCall::say(((kickTypesSpeech[walkKickVariant.kickType]) + (walkKickVariant.kickLeg == Legs::left ? " left" : " right")).c_str());
}

void WalkKickEngine::getState(float phase, Vector3f& position, Vector3f& rotation)
{
  for(int i = 0; i < 3; ++i)
  {
    position[i] = positionSplines[i](phase);
    rotation[i] = rotationSplines[i](phase);
  }
}
