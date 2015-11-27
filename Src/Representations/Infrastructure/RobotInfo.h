/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

struct RobotInfo : public RoboCup::RobotInfo, public Streamable
{
public:
  ENUM(NaoVersion,
  {,
    V33,
    V4,
    V5,
  }); // need to be sorted

  ENUM(NaoType,
  {,
    H21,
    H25,
  }); // need to be sorted

  ENUM(RobotFeature,
  {,
    hands,
    grippyFingers,
    wristYaws,
    zGyro,
    tactileHandSensores,
    tactileHeadSensores,
    headLEDs,
  });

  int number; /**< The number of the robot. */
  NaoVersion naoVersion = V5;
  NaoType naoBodyType = H21;
  NaoType naoHeadType = H21;

  RobotInfo();

  bool hasFeature(const RobotFeature feature) const;

private:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out);
};
