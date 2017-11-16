/**
 * @file GameInfo.h
 * The file declares a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"

struct GameInfo : public RoboCup::RoboCupGameControlData, public Streamable
{
private:
  using RoboCup::RoboCupGameControlData::header; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::version; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::teams; // Make teams private, the information is provided in other representations.

public:
  unsigned timeLastPackageReceived = 0;

  bool isMixedTeamCompetition() const { return gameType == GAME_MIXEDTEAM_ROUNDROBIN || gameType == GAME_MIXEDTEAM_PLAYOFF; }

  GameInfo();

  /** Draws the game time in the scene view. */
  void draw() const;

  std::string getStateAsString() const;

  friend class NaoProvider; // access to packetNumber

protected:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out);
};

/** The game info as sent by the GameController */
STREAMABLE_WITH_BASE(RawGameInfo, GameInfo, COMMA public BHumanMessageParticle<idGameInfo>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override,
});

namespace RoboCup
{
  void operator<< (RoboCupGameControlData r, const B_HULKs::OwnTeamInfo& bhOti);
}
