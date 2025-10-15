/**
 * @file Robots.h
 *
 * This file defines a representation that maps robot names to robot types
 * and for NAOs, also to the serial numbers of their heads and bodies.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Settings.h"

STREAMABLE(Robots,
{
  struct RobotId : public Streamable
  {
    std::string name;
    Settings::RobotType robotType;
    std::string headId;
    std::string bodyId;

  protected:
    void read(In& stream) override
    {
      STREAM(name);
      STREAM(robotType);
      if(robotType == Settings::nao)
      {
        STREAM(headId);
        STREAM(bodyId);
      }
    }

    void write(Out&) const override {}
  },

  (std::vector<RobotId>) robotsIds,
});
