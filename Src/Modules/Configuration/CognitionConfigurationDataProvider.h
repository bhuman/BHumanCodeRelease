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
  PROVIDES_WITH_DRAW(FieldDimensions),
  PROVIDES_WITH_MODIFY(CameraCalibration),
  PROVIDES(ColorTable),
  PROVIDES_WITH_MODIFY(RobotDimensions),
  PROVIDES_WITH_MODIFY(DamageConfiguration),
  PROVIDES_WITH_MODIFY_AND_DRAW(HeadLimits),
});

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions = nullptr;
  CameraCalibration* theCameraCalibration = nullptr;
  ColorCalibration* theColorCalibration = nullptr;
  RobotDimensions* theRobotDimensions = nullptr;
  DamageConfiguration* theDamageConfiguration = nullptr;
  HeadLimits* theHeadLimits = nullptr;
  ColorCalibration colorCalibration;

  void update(FieldDimensions& fieldDimensions);
  void update(CameraCalibration& cameraCalibration);
  void update(ColorTable& cameraCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(DamageConfiguration& damageConfiguration);
  void update(HeadLimits& headLimits);

  void readFieldDimensions();
  void readCameraCalibration();
  void readColorCalibration();
  void readCoachParameters();
  void readRobotDimensions();
  void readDamageConfiguration();
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
