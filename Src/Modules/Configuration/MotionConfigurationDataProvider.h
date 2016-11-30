/**
 * @file MotionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/StiffnessData.h"

MODULE(MotionConfigurationDataProvider,
{,
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions),
  PROVIDES(HeadLimits),
  PROVIDES(IMUCalibration),
  PROVIDES(JointCalibration),
  PROVIDES(MassCalibration),
  PROVIDES(RobotDimensions),
  PROVIDES(StiffnessSettings),
});

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase
{
private:
  DamageConfigurationBody* theDamageConfigurationBody = nullptr;
  DamageConfigurationHead* theDamageConfigurationHead = nullptr;
  FieldDimensions* theFieldDimensions = nullptr;
  HeadLimits* theHeadLimits = nullptr;
  IMUCalibration* theIMUCalibration = nullptr;
  JointCalibration* theJointCalibration = nullptr;
  MassCalibration* theMassCalibration = nullptr;
  RobotDimensions* theRobotDimensions = nullptr;
  StiffnessSettings* theStiffnessSettings = nullptr;

  void update(DamageConfigurationBody& damageConfigurationBody);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(FieldDimensions& fieldDimensions);
  void update(HeadLimits& headLimits);
  void update(IMUCalibration& imuCalibration);
  void update(JointCalibration& jointCalibration);
  void update(MassCalibration& massCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(StiffnessSettings& stiffnessSettings);

  void readDamageConfigurationBody();
  void readDamageConfigurationHead();
  void readFieldDimensions();
  void readHeadLimits();
  void readIMUCalibration();
  void readJointCalibration();
  void readMassCalibration();
  void readRobotDimensions();
  void readStiffnessSettings();

public:
  MotionConfigurationDataProvider();
  ~MotionConfigurationDataProvider();
};
