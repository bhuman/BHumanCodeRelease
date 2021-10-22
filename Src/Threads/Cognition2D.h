/**
 * @file Threads/Cognition2D.h
 *
 * This file declares the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Communication/SPLMessageHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts
#include "Tools/Framework/FrameExecutionUnit.h"

/**
 * @class Cognition
 *
 * The execution unit for the cognition thread.
 */
class Cognition2D : public FrameExecutionUnit
{
private:
  SPLMessageHandler::Buffer inTeamMessages;
  RoboCup::SPLStandardMessage outTeamMessage;
  SPLMessageHandler theSPLMessageHandler;

public:

  Cognition2D();
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
};
