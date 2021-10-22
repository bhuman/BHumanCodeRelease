/**
 * @file ArmContact.cpp
 *
 * This file implements implementations of the ArmContact
 * and ArmContactSingleArm skills.
 *
 * @author Arne Hasselbring
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/ArmContactModel.h"

SKILL_IMPLEMENTATION(ArmContactImpl,
{,
  IMPLEMENTS(ArmContact),
  IMPLEMENTS(ArmContactSingleArm),
  CALLS(KeyFrameSingleArm),
  REQUIRES(ArmContactModel),
  REQUIRES(FrameInfo),
  DEFINES_PARAMETERS(
  {,
    (int)(6000) stayBackTime, /**< The duration for which an arm stays back after contact. */
    (int)(2000) deadTime, /**< The duration for which new contacts are ignored after the arm has been back. */
  }),
});

class ArmContactImpl : public ArmContactImplBase
{
  void execute(const ArmContact&) override
  {
    setRequest(Arms::left);
    setRequest(Arms::right);
  }

  void reset(const ArmContact&) override
  {
    timesOfLastAction[Arms::left] = timesOfLastAction[Arms::right] = 0;
    timesOfLastContact[Arms::left] = timesOfLastContact[Arms::right] = 0;
  }

  void execute(const ArmContactSingleArm& p) override
  {
    setRequest(p.arm);
  }

  void reset(const ArmContactSingleArm& p) override
  {
    timesOfLastAction[p.arm] = 0;
    timesOfLastContact[p.arm] = 0;
  }

  void setRequest(Arms::Arm arm)
  {
    if(SystemCall::getMode() == SystemCall::simulatedRobot)
      return;
    if(theArmContactModel.status[arm].contact &&
       (theArmContactModel.status[arm].pushDirection == ArmContactModel::backward ||
        theArmContactModel.status[arm].pushDirection == ArmContactModel::left ||
        theArmContactModel.status[arm].pushDirection == ArmContactModel::right) &&
       theFrameInfo.getTimeSince(timesOfLastAction[arm]) >= deadTime)
      timesOfLastContact[arm] = theFrameInfo.time;
    if(theFrameInfo.getTimeSince(timesOfLastContact[arm]) < stayBackTime)
    {
      theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, arm);
      timesOfLastAction[arm] = theFrameInfo.time;
    }
  }

  unsigned timesOfLastContact[Arms::numOfArms]; /**< The last time when a contact has been detected that should be reacted upon. */
  unsigned timesOfLastAction[Arms::numOfArms]; /**< The last time when an arm was taken back by this skill. */
};

MAKE_SKILL_IMPLEMENTATION(ArmContactImpl);
