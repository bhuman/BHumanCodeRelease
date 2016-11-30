/**
* @file FieldCoverage.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "FieldCoverage.h"
#include "Representations/Infrastructure/FrameInfo.h"

FieldCoverage::GridLine& FieldCoverage::operator[](const int y)
{
  return lines[y];
}

const FieldCoverage::GridLine& FieldCoverage::operator[](const int y) const
{
  return lines[y];
}

FieldCoverageLineCompressed::FieldCoverageLineCompressed(const FieldCoverage::GridLine& line)
{
  y = static_cast<char>(line.y);
  baseTime = static_cast<int>(line.timestamps[0]);
  for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
  {
    timestamps[x] = static_cast<short>((static_cast<int>(line.timestamps[x]) - baseTime) / 20);
    values[x] = static_cast<short>((static_cast<int>(line.values[x]) - baseTime) / 20);
  }
}

FieldCoverageLineCompressed::operator FieldCoverage::GridLine() const
{
  FieldCoverage::GridLine line;
  line.y = static_cast<int>(y);

  for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
  {
    line.timestamps[x] = static_cast<unsigned>(static_cast<int>(timestamps[x] * 20) + baseTime);
    line.values[x] = static_cast<unsigned>(static_cast<int>(values[x] * 20) + baseTime);
  }

  return line;
}
