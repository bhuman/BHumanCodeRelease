/**
 * @file GameInfo.h
 * The file declares a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author Thomas Röfer
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/AutoStreamable.h"

struct GameInfo : public RoboCup::RoboCupGameControlData, public Streamable
{
private:
  using RoboCup::RoboCupGameControlData::header; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::version; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::teams; // Make teams private, the information is provided in other representations.

public:
  unsigned timeLastPacketReceived = 0;

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
  void serialize(In* in, Out* out) override;

private:
  static void reg();
};

/** The game info as sent by the GameController */
STREAMABLE_WITH_BASE(RawGameInfo, GameInfo,
{,
});
