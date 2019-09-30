/**
 * @file TeamInfo.h
 * The file declares a struct that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * @author Thomas Röfer
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Streamable.h"

struct TeamInfo : public RoboCup::TeamInfo, public Streamable
{
private:
  using RoboCup::TeamInfo::penaltyShot;   // Hide, because it is not streamed.
  using RoboCup::TeamInfo::singleShots;   // Hide, because it is not streamed.

public:
  TeamInfo();

  /**
   * Returns the number of the player that is substituted by a given robot or its own number if it does not substitute anyone.
   * @param number The number of the robot.
   * @return The number of the player that is substituted or its own number if it does not substitute anyone.
   */
  int getSubstitutedPlayerNumber(int number) const;

  /** Draws the score in the scene view. */
  void draw() const;

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

STREAMABLE_WITH_BASE(OwnTeamInfo, TeamInfo,
{
  OwnTeamInfo();
  void draw() const,
});

STREAMABLE_WITH_BASE(OpponentTeamInfo, TeamInfo,
{
  OpponentTeamInfo();
  void draw() const { TeamInfo::draw(); },
});
