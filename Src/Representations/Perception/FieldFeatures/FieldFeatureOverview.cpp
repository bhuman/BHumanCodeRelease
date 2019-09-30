/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldFeatureOverview.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"

#define PLOT_SINGE_TSL(name) \
  PLOT("representation:FieldFeatureOverview:timeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));

void FieldFeatureOverview::draw() const
{
  if(Blackboard::getInstance().exists("FrameInfo"))
  {
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);

    PLOT_SINGE_TSL(penaltyArea);
    PLOT_SINGE_TSL(midCircle);
    PLOT_SINGE_TSL(midCorner);
    PLOT_SINGE_TSL(outerCorner);
    PLOT_SINGE_TSL(goalFrame);

    PLOT("representation:FieldFeatureOverview:timeSinceLast", theFrameInfo.getTimeSince(combinedStatus.lastSeen));
  }
}

void FieldFeatureOverview::operator>>(BHumanMessage& m) const
{
  static_assert(numOfFeatures <= 8, "The container is to small. Ajust it!");
  uint8_t isRightSidedContainer = 0;
  FOREACH_ENUM(Feature, i)
    (isRightSidedContainer <<= 1) |= statuses[i].isRightSided ? 1 : 0;

  m.theBHumanArbitraryMessage.queue.out.bin << isRightSidedContainer;

  FOREACH_ENUM(Feature, i)
  {
    const FieldFeatureStatus& status = statuses[i];
    m.theBHumanArbitraryMessage.queue.out.bin << static_cast<int8_t>(status.rotation / 180_deg * 127.f);
    m.theBHumanArbitraryMessage.queue.out.bin << static_cast<int8_t>(static_cast<int>(status.translation.x()) >> 6);
    m.theBHumanArbitraryMessage.queue.out.bin << static_cast<int8_t>(static_cast<int>(status.translation.y()) >> 6);
    m.theBHumanArbitraryMessage.queue.out.bin << static_cast<uint8_t>(std::min((m.theBHumanStandardMessage.timestamp - status.lastSeen) >> 3, 0xFFu));
  }

  m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

bool FieldFeatureOverview::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());

  combinedStatus.isValid = false;
  combinedStatus.lastSeen = 0;

  {
    static_assert(numOfFeatures <= 8, "The container is to small. Ajust it!");
    uint8_t isRightSidedContainer;
    m.bin >> isRightSidedContainer;
    int runner = 1 << (numOfFeatures - 1);
    FOREACH_ENUM(Feature, i)
    {
      statuses[i].isRightSided = (isRightSidedContainer & runner) != 0;
      runner >>= 1;
    }
  }

  FOREACH_ENUM(Feature, i)
  {
    FieldFeatureStatus& status = statuses[i];
    int8_t container;

    m.bin >> container;
    status.rotation = Angle(static_cast<float>(container) * 180_deg / 127.f);

    m.bin >> container;
    status.translation.x() = static_cast<float>(static_cast<int>(container) << 6);
    m.bin >> container;
    status.translation.y() = static_cast<float>(static_cast<int>(container) << 6);

    m.bin >> container;
    const FrameInfo* theFrameInfo = nullptr;
    if(Blackboard::getInstance().exists("FrameInfo"))
      theFrameInfo = static_cast<const FrameInfo*>(&(Blackboard::getInstance()["FrameInfo"]));
    if(theFrameInfo)
    {
      //this not 100% correct, but it is just for human reading
      status.lastSeen = theFrameInfo->time - (static_cast<unsigned>(container) << 3);

      if((status.isValid = theFrameInfo->getTimeSince(status.lastSeen) < 300))
        combinedStatus.isValid = true;

      combinedStatus.lastSeen = std::max(combinedStatus.lastSeen, status.lastSeen);
    }
  }

  return true;
}
