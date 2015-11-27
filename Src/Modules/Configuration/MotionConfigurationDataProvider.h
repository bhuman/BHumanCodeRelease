/**
 * @file MotionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/UsConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/StiffnessData.h"

MODULE(MotionConfigurationDataProvider,
{,
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(StiffnessSettings),
  PROVIDES(JointCalibration),
  PROVIDES(MassCalibration),
  PROVIDES(MotionSettings),
  PROVIDES(RobotDimensions),
  PROVIDES(UsConfiguration),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions),
});

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase
{
private:
  DamageConfigurationBody* theDamageConfigurationBody = nullptr;
  DamageConfigurationHead* theDamageConfigurationHead = nullptr;
  StiffnessSettings* theStiffnessSettings = nullptr;
  JointCalibration* theJointCalibration = nullptr;
  MassCalibration* theMassCalibration = nullptr;
  MotionSettings* theMotionSettings = nullptr;
  RobotDimensions* theRobotDimensions = nullptr;
  UsConfiguration* theUsConfiguration = nullptr;
  FieldDimensions* theFieldDimensions = nullptr;

  void update(DamageConfigurationBody& damageConfigurationBody);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(FieldDimensions& fieldDimensions);
  void update(JointCalibration& jointCalibration);
  void update(MassCalibration& massCalibration);
  void update(MotionSettings& motionSettings);
  void update(RobotDimensions& robotDimensions);
  void update(StiffnessSettings& stiffnessSettings);
  void update(UsConfiguration& usConfiguration);

  void readDamageConfigurationBody();
  void readDamageConfigurationHead();
  void readFieldDimensions();
  void readJointCalibration();
  void readMassCalibration();
  void readMotionSettings();
  void readRobotDimensions();
  void readStiffnessSettings();
  void readUsConfiguration();

public:
  MotionConfigurationDataProvider();
  ~MotionConfigurationDataProvider();
};
