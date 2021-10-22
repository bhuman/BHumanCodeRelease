/**
 * @file SectorWheelUtil.h
 *
 * This file defines and implements functions that are useful when working with SectorWheels in list form.
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once

#include "SectorWheel.h"
#include "Tools/Math/Angle.h"

namespace SectorWheelUtil
{
  using Sector = SectorWheel::Sector;

  /**
   * Gets the Sector of the SectorWheel corresponding to an angle
   * @param angle      The angle
   * @param sectorList A list of sectors derived from a SectorWheel
   * @return The Sector containing the angle
   */
  inline Sector getSector(const Angle& angle, const std::list<Sector>& sectorList)
  {
    for(const Sector& sec : sectorList)
    {
      if(sec.angleRange.isInside(angle))
        return sec;
    }
    return Sector();
  }

  /**
   * Gets the sectors next to the sector of the given angle.
   * @param angle      The angle of the own sector
   * @param sectorList A list of sectors derived from a SectorWheel
   * @return An unordered list containing the sectors next to the aim sector
   */
  inline std::list<Sector> getNextSectors(const Angle& angle, const std::list<Sector>& sectorList)
  {
    std::list<Sector> nextSectors = std::list<Sector>();
    for(auto it = sectorList.cbegin(); it != sectorList.cend(); ++it)
    {
      if(it->angleRange.isInside(angle))
      {
        if(it != sectorList.cbegin())
        {
          --it;
          nextSectors.push_back(*it);
          ++it;
        }
        else
          nextSectors.push_back(*sectorList.crbegin());
        if(it != --sectorList.cend())
          nextSectors.push_back(*(++it));
        else
          nextSectors.push_back(*sectorList.cbegin());
        return nextSectors;
      }
    }
    return nextSectors;
  }
}
