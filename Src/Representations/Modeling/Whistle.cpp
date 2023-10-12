/**
 * @file Whistle.cpp
 *
 * Identified whistle sound.
 *
 * @author Thomas Röfer
 */

#include "Whistle.h"
#include "Framework/Blackboard.h"
#include "Math/BHMath.h"
#include "Representations/Infrastructure/FrameInfo.h"

void Whistle::operator>>(BHumanMessage& m) const
{
  WhistleCompact compact(*this);
  Streaming::streamIt(*m.out, "theWhistle", compact);
}

void Whistle::operator<<(const BHumanMessage& m)
{
  WhistleCompact compact(*this);
  Streaming::streamIt(*m.in, "theWhistle", compact);
}

WhistleCompact::WhistleCompact(const Whistle& whistle)
  : listening(whistle.channelsUsedForWhistleDetection > 0)
{
  if(Blackboard::getInstance().exists("FrameInfo"))
  {
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
    if(theFrameInfo.getTimeSince(whistle.lastTimeWhistleDetected) < static_cast<int>(bit(8 + 4))) // see encoding in teamMessage.def
      recentWhistle = RecentWhistle(whistle.confidenceOfLastWhistleDetection, whistle.lastTimeWhistleDetected);
  }
}

void WhistleCompact::onRead()
{
  whistle->channelsUsedForWhistleDetection = listening ? 2 : 0;
  if(recentWhistle.has_value())
  {
    whistle->confidenceOfLastWhistleDetection = recentWhistle.value().confidenceOfLastWhistleDetection;
    whistle->lastTimeWhistleDetected = recentWhistle.value().lastTimeWhistleDetected;
  }
  else
  {
    whistle->confidenceOfLastWhistleDetection = 0.f;
    whistle->lastTimeWhistleDetected = 0;
  }
}
