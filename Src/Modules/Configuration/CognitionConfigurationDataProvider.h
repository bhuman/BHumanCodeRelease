/**
* @file CognitionConfigurationDataProvider.h
* This file declares a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/TeamInfo.h"

MODULE(CognitionConfigurationDataProvider)
  USES(CameraCalibration)
  USES(CameraSettings)
  REQUIRES(OwnTeamInfo)
  PROVIDES_WITH_DRAW(FieldDimensions)
  PROVIDES_WITH_MODIFY(CameraSettings)
  PROVIDES_WITH_MODIFY(CameraCalibration)
  PROVIDES_WITH_MODIFY(RobotDimensions)
  PROVIDES_WITH_MODIFY(DamageConfiguration)
  PROVIDES_WITH_MODIFY_AND_DRAW(HeadLimits)
END_MODULE

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions;
  CameraSettings* theCameraSettings;
  CameraCalibration* theCameraCalibration;
  RobotDimensions* theRobotDimensions;
  DamageConfiguration* theDamageConfiguration;
  HeadLimits* theHeadLimits;

  void update(FieldDimensions& fieldDimensions);
  void update(CameraSettings& cameraSettings);
  void update(CameraCalibration& cameraCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(DamageConfiguration& damageConfiguration);
  void update(HeadLimits& headLimits);

  void readFieldDimensions();
  void readCameraSettings();
  void readCameraCalibration();
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
};
