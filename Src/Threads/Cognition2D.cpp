/**
 * @file Threads/Cognition2D.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 * @author Arne Hasselbring
 */

#include "Cognition2D.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Representations/Communication/BHumanMessageOutputGenerator.h"

REGISTER_EXECUTION_UNIT(Cognition2D)

bool Cognition2D::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

void Cognition2D::afterModules()
{
  if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator")
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).send
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame())
  {
    BH_TRACE_MSG("before BHumanMessageOutputGenerator::send()");
    static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).send();
  }
}
