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
#include "Tools/Debugging/Debugging.h"
#include "Tools/Settings.h"

REGISTER_EXECUTION_UNIT(Cognition2D)

Cognition2D::Cognition2D()
: theSPLMessageHandler(inTeamMessages, outTeamMessage)
{
#ifndef TARGET_ROBOT
  theSPLMessageHandler.startLocal(Global::getSettings().teamPort, static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();
  theSPLMessageHandler.start(Global::getSettings().teamPort, bcastAddr.c_str());
#endif
}

bool Cognition2D::beforeFrame()
{
  // read from team comm udp socket
  theSPLMessageHandler.receive();

  return LogDataProvider::isFrameDataComplete();
}

void Cognition2D::beforeModules()
{
  BH_TRACE_MSG("before TeamData");
  // push teammate data in our system
  if(Blackboard::getInstance().exists("TeamData") &&
     static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate)
  {
    while(!inTeamMessages.empty())
      static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate(inTeamMessages.takeBack());
  }

  DECLARE_PLOT("module:SPLMessageHandler:standardMessageDataBufferUsageInPercent");
}

void Cognition2D::afterModules()
{
  if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator")
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame)
  {
    static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate(&outTeamMessage);

    BH_TRACE_MSG("before theSPLMessageHandler.send()");
    theSPLMessageHandler.send();
  }
}
