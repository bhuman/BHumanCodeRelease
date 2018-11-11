/**
 * @file Modules/Infrastructure/NaoProvider.h
 * The file declares a module that provides information from the Nao via DCM.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#ifdef TARGET_ROBOT
#include "Platform/Nao/NaoBody.h"
#endif
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Module/Module.h"

MODULE(NaoProvider,
{,
  REQUIRES(JointCalibration),
  REQUIRES(JointLimits),
  REQUIRES(LEDRequest),

  PROVIDES(FrameInfo),
  REQUIRES(FrameInfo),

  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  USES(RobotInfo),
  PROVIDES(SystemSensorData),
  USES(JointRequest), // Will be accessed in send()
});

#ifdef TARGET_ROBOT

/**
 * @class NaoProvider
 * A module that provides information from the Nao.
 */
class NaoProvider : public NaoProviderBase
{
private:
  static thread_local NaoProvider* theInstance; /**< The only instance of this module. */

  NaoBody naoBody;
  RoboCup::RoboCupGameControlData gameControlData; /**< The last game control data received. */
  unsigned gameControlTimeStamp = 0; /**< The time when the last gameControlData was received (kind of). */
  float clippedLastFrame[Joints::numOfJoints]; /**< Array that indicates whether a certain joint value was clipped in the last frame (and what was the value)*/
  unsigned lastBodyTemperatureReadTime = 0;

public:
  NaoProvider();
  ~NaoProvider();

  /** The method is called by process Motion to send the requests to the Nao. */
  static void finishFrame();
  static void waitForFrameData();

private:
  void update(FrameInfo& frameInfo) override;
  void update(FsrSensorData& fsrSensorData) override;
  void update(InertialSensorData& inertialSensorData) override;
  void update(JointSensorData& jointSensorData) override;
  void update(KeyStates& keyStates) override;
  void update(OpponentTeamInfo& opponentTeamInfo) override;
  void update(OwnTeamInfo& ownTeamInfo) override;
  void update(RawGameInfo& rawGameInfo) override;
  void update(RobotInfo& robotInfo) override;
  void update(SystemSensorData& systemSensorData) override;

  /** The function sends a command to the Nao. */
  void send();
};

#else
//TARGET_ROBOT not defined here (Simulator).

class NaoProvider : public NaoProviderBase
{
private:
  void update(FrameInfo& frameInfo) {}
  void update(FsrSensorData& fsrSensorData) {}
  void update(InertialSensorData& inertialSensorData) {}
  void update(JointSensorData& jointSensorData) {}
  void update(KeyStates& keyStates) {}
  void update(OpponentTeamInfo& opponentTeamInfo) {}
  void update(OwnTeamInfo& ownTeamInfo) {}
  void update(RawGameInfo& rawGameInfo) {}
  void update(RobotInfo& robotInfo) {}
  void update(SystemSensorData& systemSensorData) {}
  void send();

public:
  static void finishFrame() {}
  static void waitForFrameData() {}
};

#endif
