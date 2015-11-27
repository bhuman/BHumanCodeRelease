/**
* @file CognitionConfigurationDataProvider.h
* This file declares a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/ColorCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/TeamInfo.h"

MODULE(CognitionConfigurationDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  USES(CameraCalibrationNext),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions),
  PROVIDES(CameraCalibration),
  PROVIDES_WITHOUT_MODIFY(ColorTable),
  PROVIDES(RobotDimensions),
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(HeadLimits),
});

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static PROCESS_LOCAL CognitionConfigurationDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions = nullptr;
  CameraCalibration* theCameraCalibration = nullptr;
  ColorCalibration* theColorCalibration = nullptr;
  RobotDimensions* theRobotDimensions = nullptr;
  DamageConfigurationBody* theDamageConfigurationBody = nullptr;
  DamageConfigurationHead* theDamageConfigurationHead = nullptr;
  HeadLimits* theHeadLimits = nullptr;
  ColorCalibration colorCalibration;

  void update(FieldDimensions& fieldDimensions);
  void update(CameraCalibration& cameraCalibration);
  void update(ColorTable& cameraCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(DamageConfigurationBody& damageConfigurationBody);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(HeadLimits& headLimits);

  void readFieldDimensions();
  void readCameraCalibration();
  void readColorCalibration();
  void readRobotDimensions();
  void readDamageConfigurationBody();
  void readDamageConfigurationHead();
  void readHeadLimits();

public:
  /**
  * Default constructor.
  */
  CognitionConfigurationDataProvider();

  /**
  * Destructor.
  */
  ~CognitionConfigurationDataProvider();

  /**
   * Called from a MessageQueue to distribute messages
   * @param message The message that can be read
   * @return true if the message was handled
   */
  static bool handleMessage(InMessage& message);
};
