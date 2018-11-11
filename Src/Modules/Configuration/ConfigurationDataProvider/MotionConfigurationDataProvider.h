/**
 * @file MotionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Lukas Post
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/GlobalOptions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/WalkKicks.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Tools/Module/Module.h"

MODULE(MotionConfigurationDataProvider,
{,
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(FieldDimensions),
  PROVIDES(GlobalOptions),
  PROVIDES(HeadLimits),
  PROVIDES(IMUCalibration),
  PROVIDES(JointCalibration),
  PROVIDES(JointLimits),
  PROVIDES(MassCalibration),
  PROVIDES(RobotDimensions),
  PROVIDES(StiffnessSettings),
  PROVIDES(WalkKicks),
});

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase
{
public:
  MotionConfigurationDataProvider();

private:
  std::unique_ptr<DamageConfigurationBody> theDamageConfigurationBody;
  std::unique_ptr<DamageConfigurationHead> theDamageConfigurationHead;
  std::unique_ptr<FieldDimensions> theFieldDimensions;
  std::unique_ptr<GlobalOptions> theGlobalOptions;
  std::unique_ptr<HeadLimits> theHeadLimits;
  std::unique_ptr<IMUCalibration> theIMUCalibration;
  std::unique_ptr<JointCalibration> theJointCalibration;
  std::unique_ptr<JointLimits> theJointLimits;
  std::unique_ptr<MassCalibration> theMassCalibration;
  std::unique_ptr<RobotDimensions> theRobotDimensions;
  std::unique_ptr<StiffnessSettings> theStiffnessSettings;
  std::unique_ptr<WalkKicks> theWalkKicks;

  void update(DamageConfigurationBody& damageConfigurationBody) override {update(damageConfigurationBody, theDamageConfigurationBody);}
  void update(DamageConfigurationHead& damageConfigurationHead) override {update(damageConfigurationHead, theDamageConfigurationHead);}
  void update(FieldDimensions& fieldDimensions) override {update(fieldDimensions, theFieldDimensions);}
  void update(GlobalOptions& globalOptions) override {update(globalOptions, theGlobalOptions);}
  void update(HeadLimits& headLimits) override {update(headLimits, theHeadLimits);}
  void update(IMUCalibration& imuCalibration) override {update(imuCalibration, theIMUCalibration);}
  void update(JointCalibration& jointCalibration) override {update(jointCalibration, theJointCalibration);}
  void update(JointLimits& jointLimits) override;
  void update(MassCalibration& massCalibration) override {update(massCalibration, theMassCalibration);}
  void update(RobotDimensions& robotDimensions) override;
  void update(StiffnessSettings& stiffnessSettings) override {update(stiffnessSettings, theStiffnessSettings);}
  void update(WalkKicks& walkKicks) override;

  template<typename T> void update(T& representation, std::unique_ptr<T>& theRepresentation)
  {
    if(theRepresentation)
    {
      representation = *theRepresentation;
      theRepresentation = nullptr;
    }
  }

  void readWalkKicks();

  template<typename T> void read(std::unique_ptr<T>& theRepresentation, const char* fileName = nullptr)
  {
    ASSERT(!theRepresentation);
    theRepresentation = std::make_unique<T>();
    loadModuleParameters(*theRepresentation, TypeRegistry::demangle(typeid(T).name()).c_str(), fileName);
  }
};
