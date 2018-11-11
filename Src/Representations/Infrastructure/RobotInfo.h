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
#include "Tools/Streams/Enum.h"
#include "Tools/Settings.h"

struct RobotInfo : public RoboCup::RobotInfo, public Streamable
{
public:
  ENUM(NaoVersion,
  {,
    V32,
    V33,
    V4,
    V5,
    V6,
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
    zAngle,
    zGyro,
    tactileHandSensores,
    tactileHeadSensores,
    headLEDs,
  });

  int number; /**< The number of the robot. */

  NaoVersion headVersion = V4;
  NaoType headType = H21;
  NaoVersion bodyVersion = V5;
  NaoType bodyType = H21;

  RobotInfo();

  bool hasFeature(const RobotFeature feature) const;
  
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
