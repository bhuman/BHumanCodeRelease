/**
 * @file Presets.h
 *
 * This file defines a class to represent the preset per team the code
 * will be deployed with.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

struct Presets : public Streamable
{
  STREAMABLE(Preset,
  {
    bool operator==(const Preset& other) const
    {
      return name == other.name
        && number == other.number
        && fieldPlayerColor == other.fieldPlayerColor
        && goalkeeperColor == other.goalkeeperColor
        && scenario == other.scenario
        && location == other.location
        && wlanConfig == other.wlanConfig
        && volume == other.volume
        && magicNumber == other.magicNumber
        && players == other.players;
    },

    (std::string) name, /**< The name of the preset. */
    (int) number, /**< The team number. */
    (std::string) fieldPlayerColor, /**< The jersey color of the field players. */
    (std::string) goalkeeperColor, /**< The jersey color of the goal keeper. */
    (std::string) scenario, /**< The scenario. */
    (std::string) location, /**< The location. */
    (std::string) wlanConfig, /**< The wifi profile to set when deploying. */
    (int) volume, /**< The volume to set when deploying. */
    (int) magicNumber, /**< The magic number to set when deploying. */
    (std::vector<std::string>) players, /**< The names of the robots for the different player numbers ("_" for no robot). */
  });

  std::vector<Preset*> teams; /**< The presets for the teams.*/

  /** Destructor. Delete the teams. */
  ~Presets()
  {
    for(const Preset* team : teams)
      delete team;
  }

  /**
   * Is this object equal to another?
   * @param other The other object.
   * @return Are they equal?
   */
  bool operator==(const Presets& other) const
  {
    if(teams.size() != other.teams.size())
      return false;
    for(size_t i = 0; i < teams.size(); ++i)
      if(*teams[i] != *other.teams[i])
        return false;
    return true;
  }

protected:
  /**
   * Read this object from a stream.
   * This handles that \c teams is actually a vector of pointers.
   * @param stream The stream this object is read from.
   */
  void read(In& stream) override
  {
    std::vector<Preset> teams;
    STREAM(teams);
    for(const Preset* team : this->teams)
      delete team;
    this->teams.clear();
    for(const Preset& team : teams)
      this->teams.push_back(new Preset(team));
  }

  /**
   * Writes this object into a stream.
   * This handles that \c teams is actually a vector of pointers.
   * @param stream The stream this object is written to.
   */
  void write(Out& stream) const override
  {
    std::vector<Preset> teams;
    for(const Preset* team : this->teams)
      teams.push_back(*team);
    STREAM(teams);
  }
};
