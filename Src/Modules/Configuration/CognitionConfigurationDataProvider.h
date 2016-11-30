/**
 * @file CognitionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Configuration/Behavior2015Parameters.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Module.h"

MODULE(CognitionConfigurationDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  USES(CameraCalibrationNext),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions),
  PROVIDES_WITHOUT_MODIFY(IntersectionRelations),
  PROVIDES(CameraCalibration),
  PROVIDES(CameraSettings),
  PROVIDES(FieldColors),
  PROVIDES(RobotDimensions),
  PROVIDES(Behavior2015Parameters),
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(HeadLimits),
});

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static thread_local CognitionConfigurationDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions = nullptr;
  FieldColors* theFieldColors = nullptr;
  IntersectionRelations* theIntersectionRelations = nullptr;
  CameraCalibration* theCameraCalibration = nullptr;
  CameraSettings* theCameraSettings = nullptr;
  RobotDimensions* theRobotDimensions = nullptr;
  Behavior2015Parameters* theBehavior2015Parameters = nullptr;
  DamageConfigurationBody* theDamageConfigurationBody = nullptr;
  DamageConfigurationHead* theDamageConfigurationHead = nullptr;
  HeadLimits* theHeadLimits = nullptr;

  void update(CameraCalibration& cameraCalibration);
  void update(CameraSettings& cameraSettings);
  void update(FieldDimensions& fieldDimensions);
  void update(FieldColors& fieldColors);
  void update(IntersectionRelations& intersectionRelations);
  void update(RobotDimensions& robotDimensions);
  void update(Behavior2015Parameters& behavior2015Parameters);
  void update(DamageConfigurationBody& damageConfigurationBody);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(HeadLimits& headLimits);

  void readFieldDimensions();
  void readFieldColors();
  void readCameraCalibration();
  void readCameraSettings();
  void readRobotDimensions();
  void readBehavior2015Parameters();
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
