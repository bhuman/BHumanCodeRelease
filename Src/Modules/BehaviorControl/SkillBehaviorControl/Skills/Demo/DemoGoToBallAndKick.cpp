/**
 * @file DemoGoToBallAndKick.cpp
 *
 * This file implements an implementation of the DemoGoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include <algorithm>

STREAMABLE(PowerKickPair,
{
  PowerKickPair() = default;
  PowerKickPair(const KickInfo::KickType kick, const float power),

  (KickInfo::KickType) kick, /**< Kick type. */
  (float) power, /**< Kick power. Some kicks may be softer because of the unknown underground. */
});
inline PowerKickPair::PowerKickPair(const KickInfo::KickType kick, const float power) : kick(kick), power(power) {}

SKILL_IMPLEMENTATION(DemoGoToBallAndKickImpl,
{,
  IMPLEMENTS(DemoGoToBallAndKick),
  CALLS(LookAtBall),
  CALLS(WalkToBallAndKick),
  CALLS(WalkToPoint),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(MotionInfo),
  LOADS_PARAMETERS(
  {,
    (Vector2f) waitPosition, /**< We can not trust the detected ball, therefor wait behind the known ball location. */
    (int) ballSeenDuration, /**< The ball was seen in this time frame, therefore we can trust the detection. */
    (int) lastKickDuration, /**< Ignore ball detections when we recently kicked. */
    (std::vector<PowerKickPair>) availableKicks, /**< All allowed kicks. */
    (unsigned int) kickRepetitionNumber, /**< Number of kick repetitions for the random selection. */
    (unsigned int) numberOfSampledKicksBeforeSoftReset, /**< After this many kicks add used kicks back into the selection list. */
  }),
});

class DemoGoToBallAndKickImpl : public DemoGoToBallAndKickImplBase
{
  void execute(const DemoGoToBallAndKick&) override
  {
    theLookAtBallSkill();
    // Last kick must happend a few seconds before or the ball was seen just recently
    if((theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballSeenDuration  // We recently saw the ball
        || theFrameInfo.getTimeSince(theMotionInfo.lastKickTimestamp) > lastKickDuration) // The last kick was long ago
       && theBallModel.timeWhenLastSeen > theMotionInfo.lastKickTimestamp + lastKickDuration) // The detected ball timestamp must be AFTER the last kick
      kickBall();
    else
      // Otherwise we assume the ball detection is old and the last kick move the ball away. (Prevent double kicks)
      goToLastKnownPosition();
  }

  void kickBall()
  {
    // Robot just kicked. Select new kick
    if(lastKick != theMotionInfo.lastKickTimestamp)
      selectNextKick();
    // alignPrecisely = true for InWalkKicks forces the robot to come to a full stop before kicking.
    const KickPrecision alignPrecisely = kickType.kick == KickInfo::forwardFastLeft || kickType.kick == KickInfo::forwardFastRight
                                         ? KickPrecision::notPrecise : KickPrecision::precise;
    theWalkToBallAndKickSkill({.targetDirection = -theKickInfo[kickType.kick].rotationOffset,
                               .kickType = kickType.kick,
                               .alignPrecisely = alignPrecisely,
                               .kickLength = mapToRange(kickType.power, 0.f, 1.f, theKickInfo[kickType.kick].range.min, theKickInfo[kickType.kick].range.max)});
  }

  void goToLastKnownPosition()
  {
    theWalkToPointSkill({.target = {theBallModel.estimate.position + waitPosition}});
  }

  void reset(const DemoGoToBallAndKick&) override
  {
    selectNextKick();
  }

  void setUpRandomKickList()
  {
    // Make sure the parameters are useable
    ASSERT(availableKicks.size() * kickRepetitionNumber > numberOfSampledKicksBeforeSoftReset);
    ASSERT(availableKicks.size() > 0);
    initKickList = true;
    randomKickSelectionList.clear();
    for(const PowerKickPair& kick : availableKicks)
    {
      // In case the strong forward kick is not allowed, simply do not add it
      if(kick.kick == KickInfo::forwardFastLeft && theDamageConfigurationBody.sides[Legs::right].weakLeg)
        continue;
      if(kick.kick == KickInfo::forwardFastRight && theDamageConfigurationBody.sides[Legs::left].weakLeg)
        continue;
      for(unsigned int i = 0; i < kickRepetitionNumber; i++)
        randomKickSelectionList.push_back(kick);
    }
    if(randomKickSelectionList.size() < numberOfSampledKicksBeforeSoftReset || randomKickSelectionList.size() <= 0)
      FAIL("Not enough available kicks for DemoGoToBallAndKickSkill with number of kick: " << randomKickSelectionList.size() << ", and minimum kick number of: " << numberOfSampledKicksBeforeSoftReset);
    usedKickSelectionList.clear();
  }

  void selectNextKick()
  {
    lastKick = theMotionInfo.lastKickTimestamp;
    const int index = Random::uniformInt(static_cast<int>(randomKickSelectionList.size() - 1));
    kickType = randomKickSelectionList[index];
    randomKickSelectionList.erase(std::next(randomKickSelectionList.begin(), index));
    usedKickSelectionList.push_back(kickType);
    if(usedKickSelectionList.size() > numberOfSampledKicksBeforeSoftReset)
    {
      const int index = Random::uniformInt(static_cast<int>(usedKickSelectionList.size() - 1));
      randomKickSelectionList.push_back(usedKickSelectionList[index]);
      usedKickSelectionList.erase(std::next(usedKickSelectionList.begin(), index));
    }
  }

  void preProcess() override
  {
    // Once initialize the kick list
    if(!initKickList)
      setUpRandomKickList();
  }

  void preProcess(const DemoGoToBallAndKick&) override {}

  PowerKickPair kickType; /**< The currently planned kick type. */
  bool initKickList = false; /**< Is the list of the currently available kicks initialized? */
  std::vector<PowerKickPair> randomKickSelectionList; /**< Sample list of kicks that are used to decide the next kick. */
  std::vector<PowerKickPair> usedKickSelectionList; /**< Used kicks. */
  unsigned int lastKick = 0;
};

MAKE_SKILL_IMPLEMENTATION(DemoGoToBallAndKickImpl);
