/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author Thomas Röfer
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

struct RobotInfo : public RoboCup::RobotInfo, public Streamable
{
public:
  int number; /**< The number of the robot. */

  RobotInfo();

  std::string getPenaltyAsString() const;

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
