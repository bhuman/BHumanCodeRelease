/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/Streamable.h"

struct RobotInfo : public RoboCup::RobotInfo, public Streamable
{
public:
  ENUM(Mode,
  {,
    unstiff,
    active,
    calibration,
  });

  int number; /**< The number of the robot. */

  Mode mode; /**< The current mode. */

  RobotInfo();

  std::string getPenaltyAsString() const;

  bool isGoalkeeper() const;

  /** Draw the representation. */
  void draw() const;

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override;

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override;

private:
  static void reg();
};
