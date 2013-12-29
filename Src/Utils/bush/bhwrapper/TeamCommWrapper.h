#pragma once

#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/ProcessFramework/TeamHandler.h"

/**
 * A wrapper for the BHuman team communication. You just have to call send
 * and receive in every frame and you can use the TEAM_OUTPUT macro if the
 * global team out is initialized (see Framework class).
 */
class TeamCommWrapper
{
  int teamPort;
  MessageQueue theTeamReceiver;
  MessageQueue& theTeamSender;
  MessageHandler* messageHandler;
  TeamHandler theTeamHandler;

public:
  TeamCommWrapper(MessageQueue& teamOut, int teamPort, MessageHandler* messageHandler);
  ~TeamCommWrapper();

  void receive();
  void send();
};
