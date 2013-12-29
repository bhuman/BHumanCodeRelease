#include "TeamCommWrapper.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/StringTools.h"

TeamCommWrapper::TeamCommWrapper(MessageQueue& teamOut, int teamPort, MessageHandler* messageHandler)
  : teamPort(teamPort), theTeamSender(teamOut), messageHandler(messageHandler), INIT_TEAM_COMM
{
  theTeamSender.setSize(1384); // 1 package without checksum and size
  theTeamReceiver.setSize(4 * 1450); // >= 4 packages

  theTeamHandler.start(teamPort, "255.255.255.255");

  receive();

  Session::getInstance().log(TRACE, "TeamCommWrapper: Team communication initialized (port: " + toString(teamPort) + ").");
}

TeamCommWrapper::~TeamCommWrapper()
{
  Session::getInstance().log(TRACE, "TeamCommWrapper: Team communication stopped (port: " + toString(teamPort) + ").");
}

void TeamCommWrapper::receive()
{
  RECEIVE_TEAM_COMM;
  theTeamReceiver.handleAllMessages(*messageHandler);
  theTeamReceiver.clear();
}

void TeamCommWrapper::send()
{
  SEND_TEAM_COMM;
  theTeamSender.clear();
}

