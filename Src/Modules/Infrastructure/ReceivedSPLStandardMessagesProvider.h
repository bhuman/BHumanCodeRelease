/**
* @file ReceivedSplStandardMessagesProvider.h
* Contains the recently received SPLStandardMessage of each teammate
* to be used in DropInChallenge
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"

MODULE(ReceivedSPLStandardMessagesProvider,
{,
  REQUIRES(FrameInfo),
  PROVIDES(ReceivedSPLStandardMessages),
});

class ReceivedSPLStandardMessagesProvider : public ReceivedSPLStandardMessagesProviderBase
{
private:
  static PROCESS_LOCAL ReceivedSPLStandardMessagesProvider* theInstance;

  std::vector<SPLStandardMessageWithoutData> recentMessages;
  std::vector<unsigned> recentMessagesTimestamps;

public:
  ReceivedSPLStandardMessagesProvider();
  ~ReceivedSPLStandardMessagesProvider();

  void update(ReceivedSPLStandardMessages& messages);

  static void addMessage(SPLStandardMessage& msg);
};
