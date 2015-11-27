/**
 * @file TeamInfo.h
 * The file declares a struct that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

struct TeamInfo : public RoboCup::TeamInfo, public Streamable
{
private:
  using RoboCup::TeamInfo::penaltyShot; // Hide, because it is not streamed.
  using RoboCup::TeamInfo::singleShots; // Hide, because it is not streamed.

public:
  TeamInfo();

  /** Draws the score in the scene view. */
  void draw() const;
private:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out);
};

struct OwnTeamInfo : public TeamInfo
{
  OwnTeamInfo();
  void draw() const;
};

struct OpponentTeamInfo : public TeamInfo
{
  OpponentTeamInfo();
  void draw() const { TeamInfo::draw(); }
};
