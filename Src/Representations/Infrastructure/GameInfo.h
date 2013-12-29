/**
* @file GameInfo.h
* The file declares a class that encapsulates the structure RoboCupGameControlData
* defined in the file RoboCupGameControlData.h that is provided with the GameController.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

class GameInfo : public RoboCup::RoboCupGameControlData, public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out);

  using RoboCup::RoboCupGameControlData::teams; // Make teams private, the information is provided in other representations.

public:
  unsigned timeLastPackageReceived;

  /** Default constructor. */
  GameInfo();

  /** Draws the game time in the scene view. */
  void draw() const;
};
