/**
 * @file BehaviorControl.cpp
 * Implementation of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "BehaviorControl.h"
#include "Tools/Streams/InStreams.h"

BehaviorControl::BehaviorControl()
  : Cabsl<BehaviorControl>(const_cast<ActivationGraph*>(&theActivationGraph))
{
  InMapFile stream("behaviorControl.cfg");
  ASSERT(stream.exists());
  stream >> parameters;
}

void BehaviorControl::update(ActivationGraph& activationGraph)
{
  Parameters p(parameters); // make a copy, to make "unchanged" work
  MODIFY("parameters:BehaviorControl", p);
  if(theFrameInfo.time)
  {
    beginFrame(theFrameInfo.time);
    for(OptionInfos::Option option : p.roots)
      Cabsl<BehaviorControl>::execute(option);
    endFrame();

    theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_DEFAULT;
    switch(theBehaviorStatus.role)
    {
      case Role::striker:
        theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KICK;
        break;
      case Role::keeper:
        theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KEEPER;
        break;
      case Role::defender:
        theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_DEFENSIVE;
    }
    if(theSideConfidence.confidenceState == SideConfidence::CONFUSED)
      theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_LOST;
  }
}

MAKE_MODULE(BehaviorControl, behaviorControl)
