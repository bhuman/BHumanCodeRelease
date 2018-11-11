/**
 * @file FieldCoverage.cpp
 * @author Andreas Stolpmann
 */

#include "FieldCoverage.h"
void FieldCoverage::operator>>(BHumanMessage& m) const
{
  const GridLine& line = lines[lineToSendNext];

  m.theBHumanArbitraryMessage.queue.out.bin << static_cast<char>(line.y);
  for(size_t x = 0; x < line.timestamps.size(); ++x)
    m.theBHumanArbitraryMessage.queue.out.bin << static_cast<unsigned short>(line.timestamps[x] / 100);
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
}

void FieldCoverage::operator<<(const BHumanMessage& m)
{
  lines.clear();
}

bool FieldCoverage::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());

  GridLine line;

  char y;
  m.bin >> y;
  line.y = static_cast<int>(y);

  unsigned short timestamp;
  while(m.getBytesLeft())
  {
    m.bin >> timestamp;
    line.timestamps.emplace_back(toLocalTimestamp(static_cast<unsigned>(timestamp) * 100));
  }

  lines.emplace_back(line);

  return true;
}
