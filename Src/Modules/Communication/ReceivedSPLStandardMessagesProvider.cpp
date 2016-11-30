/**
 * @file ReceivedSplStandardMessagesProvider.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */
#include "ReceivedSPLStandardMessagesProvider.h"
#include "Tools/Settings.h"

MAKE_MODULE(ReceivedSPLStandardMessagesProvider, communication)

thread_local ReceivedSPLStandardMessagesProvider* ReceivedSPLStandardMessagesProvider::theInstance = nullptr;

ReceivedSPLStandardMessagesProvider::ReceivedSPLStandardMessagesProvider()
{
  theInstance = this;
}

ReceivedSPLStandardMessagesProvider::~ReceivedSPLStandardMessagesProvider()
{
  theInstance = nullptr;
}

void ReceivedSPLStandardMessagesProvider::update(ReceivedSPLStandardMessages& messages)
{
  constexpr int maxPlayerNum = Settings::highestValidPlayerNumber;
  // Check for correct sizes of message / timestamp buffers
  if(recentMessages.size() != static_cast<unsigned int>(maxPlayerNum + 1))
  {
    recentMessages.resize(maxPlayerNum + 1);
    recentMessagesTimestamps.resize(maxPlayerNum + 1, 0);
  }
  if(messages.messages.size() != static_cast<unsigned int>(maxPlayerNum + 1))
  {
    messages.messages.resize(maxPlayerNum + 1);
    messages.timestamps.resize(maxPlayerNum + 1, 0);
  }
  for(int i = Settings::lowestValidPlayerNumber; i <= maxPlayerNum; ++i)
  {
    messages.messages[i] = recentMessages[i];
    messages.timestamps[i] = recentMessagesTimestamps[i];
  }
}

void ReceivedSPLStandardMessagesProvider::addMessage(SPLStandardMessage& msg)
{
  if(theInstance)
  {
    if(msg.playerNum >= Settings::lowestValidPlayerNumber &&
       msg.playerNum <= Settings::highestValidPlayerNumber &&
       msg.playerNum < static_cast<unsigned char>(theInstance->recentMessages.size()))
    {
      theInstance->recentMessages[msg.playerNum] = msg;
      theInstance->recentMessagesTimestamps[msg.playerNum] = theInstance->theFrameInfo.time;
    }
    else
    {
      OUTPUT_WARNING("Received SPLStandardMessage with invalid player number '" << msg.playerNum << "'.");
    }
  }
}
