/**
 * @file BehaviorControl.cpp
 *
 * This file implements the module that describes the robot behavior.
 *
 * @author Jesse Richter-Klug
 */

#include "BehaviorControl.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/Debugging/Annotation.h"
#include <regex>

MAKE_MODULE(BehaviorControl, behaviorControl, BehaviorControl::getExtModuleInfo);

#define SKILL(name) (*theSkillRegistry.getSkill<Skills::name##Skill>(#name))

BehaviorControl::BehaviorControl() :
  theSkillRegistry("skills.cfg", const_cast<ActivationGraph&>(theActivationGraph), theArmMotionRequest, theBehaviorStatus, theHeadMotionRequest, theMotionRequest, theTeamTalk),
  theCardRegistry(const_cast<ActivationGraph&>(theActivationGraph))
{
  theSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<Card>::first));
}

std::vector<ModuleBase::Info> BehaviorControl::getExtModuleInfo()
{
  auto result = BehaviorControl::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<Card>::first, result);

  return result;
}

void BehaviorControl::update(ActivationGraph& activationGraph)
{
  activationGraph.graph.clear();
  activationGraph.currentDepth = 0;

  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.shootingTo = Vector2f::Zero();

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theSkillRegistry.modifyAllParameters();
  theCardRegistry.modifyAllParameters();

  theSkillRegistry.preProcess(theFrameInfo.time);
  theCardRegistry.preProcess(theFrameInfo.time);

  ASSERT(activationGraph.graph.empty());
  activationGraph.graph.emplace_back("BehaviorControl", 0, TypeRegistry::getEnumName(status), theFrameInfo.time, 0, std::vector<std::string>());
  this->execute();
  activationGraph.graph[0].state = TypeRegistry::getEnumName(status);

  theCardRegistry.postProcess();
  theSkillRegistry.postProcess();

  theLibCheck.performCheck(theMotionRequest);
}

void BehaviorControl::execute()
{
#ifdef TARGET_SIM
  MODIFY("module:BehaviorControl:status", status);
#endif

  //is usb mounted? - Sound
  if(theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 3)
  {
    if(SystemCall::usbIsMounted())
      SKILL(Say)("USB mounted");
    else
      SKILL(Say)("USB not mounted");
  }

  if((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 && theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0)
     || (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 && theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
    SKILL(Say)((std::regex_replace(TypeRegistry::getEnumName(theRobotHealth.jointWithMaxTemperature), std::regex("([A-Z])"), " $1") + " " + std::to_string(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature])).c_str());

  if((status == gettingUp || status == penalized || status == playing) && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 3)
    status = sittingDown;

  if((status == penalized || status == playing) && !theCameraStatus.ok)
    status = cameraStatusFAILED;

#ifndef NDEBUG
  if((status == penalized || status == playing) && theRobotHealth.batteryLevel <= 1)
    status = lowBattery;
#endif

  if(status == sittingDown || status == lowBattery || status == cameraStatusFAILED)
  {
    SKILL(Activity)(BehaviorStatus::unknown);
    SKILL(LookForward)();
    SKILL(SpecialAction)(SpecialActionRequest::sitDown);

    if(SKILL(SpecialAction).isDone())
      status = inactive;

    return;
  }

  if(status == inactive)
  {
    SKILL(Activity)(BehaviorStatus::unknown);
    if(SystemCall::getMode() == SystemCall::simulatedRobot)
    {
      SKILL(LookForward)();
      SKILL(SpecialAction)(SpecialActionRequest::standHigh);
      if(SKILL(SpecialAction).isDone())
        status = playing;
    }
    else
    {
      SKILL(LookAtAngles)(JointAngles::off, JointAngles::off);
      SKILL(SpecialAction)(SpecialActionRequest::playDead);
      if(theEnhancedKeyStates.getHitStreakFor(KeyStates::chest, 1000, 0) > 0)
        status = gettingUp;
    }
    return;
  }

  // On a real robot, directly interpolating from playDead to sstandHigh is bad, therefore, a stand is requested in between.
  if(status == gettingUp)
  {
    SKILL(Activity)(BehaviorStatus::unknown);
    SKILL(LookForward)();
    SKILL(Stand)();
    if(SKILL(Stand).isDone())
      status = playing;
    return;
  }

  if(theRobotInfo.penalty != PENALTY_NONE)
  {
    // Is this the first frame that we are penalized?
    if(status != penalized)
    {
      ANNOTATION("Behavior", "Penalized " + theRobotInfo.getPenaltyAsString());
      SystemCall::say("Penalized");
    }

    status = penalized;
    SKILL(Activity)(BehaviorStatus::unknown);
    SKILL(LookForward)();
    SKILL(SpecialAction)(SpecialActionRequest::standHigh);

    return;
  }

  // Is this the first frame that we are unpenalized?
  if(status == penalized)
  {
    ANNOTATION("Behavior", "Unpenalized");
    SystemCall::say("Not penalized");
  }

  // we don't want to be inactive any longer or we were playing before
  status = playing;

  CardBase* card = theCardRegistry.getCard(rootCard);
  if(card)
    return card->call();

  // error
  SKILL(Annotation)("NO VALID BEHAVIOR DEFINED!");
  SKILL(Activity)(BehaviorStatus::unknown);
  SKILL(LookForward)();
  SKILL(SpecialAction)(SpecialActionRequest::sitDown);
}
