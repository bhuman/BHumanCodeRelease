/**
* @file ReceivedSplStandardMessagesProvider.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "ReceivedSPLStandardMessagesProvider.h"

MAKE_MODULE(ReceivedSPLStandardMessagesProvider, Cognition Infrastructure)

PROCESS_WIDE_STORAGE(ReceivedSPLStandardMessagesProvider) ReceivedSPLStandardMessagesProvider::theInstance = 0;

ReceivedSPLStandardMessagesProvider::ReceivedSPLStandardMessagesProvider()
{
  theInstance = this;
  memset(recentMessagesTimestamps, 0, sizeof(recentMessagesTimestamps));
}

ReceivedSPLStandardMessagesProvider::~ReceivedSPLStandardMessagesProvider()
{
  theInstance = 0;
}

void ReceivedSPLStandardMessagesProvider::update(ReceivedSPLStandardMessages& messages)
{
  for(int i = 0; i < TeammateData::numOfPlayers; ++i)
  {
    messages.messages[i] = recentMessages[i];
    messages.timestamps[i] = recentMessagesTimestamps[i];
  }
}

void ReceivedSPLStandardMessagesProvider::addMessage(SPLStandardMessage& msg)
{
  if(theInstance)
  {
    if(msg.playerNum >= 0 && msg.playerNum < TeammateData::numOfPlayers)
    {
      theInstance->recentMessages[msg.playerNum] = msg;
      theInstance->recentMessagesTimestamps[msg.playerNum] = theInstance->theFrameInfo.time;
    }
    else
    {
      OUTPUT_WARNING("Received SPLStandardMessage with player number '" << msg.playerNum << "'.");
    }
  }
}