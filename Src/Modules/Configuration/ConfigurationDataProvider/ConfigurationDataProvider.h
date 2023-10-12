/**
 * @file ConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/BHAssert.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RelativeFieldColorsParameters.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/StaticJointPoses.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/KeyframeMotionParameters.h"
#include "Representations/MotionControl/WalkModifier.h"
#include <memory>

MODULE(ConfigurationDataProvider,
{,
  USES(CameraCalibrationNext),
  PROVIDES(BallSpecification),
  USES(BallSpecification),
  PROVIDES(BehaviorParameters),
  PROVIDES(CameraCalibration),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraSettings),
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(FieldDimensions),
  PROVIDES(FootOffset),
  PROVIDES(HeadLimits),
  PROVIDES(IMUCalibration),
  PROVIDES(JointCalibration),
  PROVIDES(JointLimits),
  PROVIDES(KeyframeMotionParameters),
  PROVIDES(KickInfo),
  PROVIDES(MassCalibration),
  PROVIDES(RelativeFieldColorsParameters),
  PROVIDES(RobotDimensions),
  PROVIDES(StaticJointPoses),
  PROVIDES(StiffnessSettings),
  PROVIDES(WalkModifier),
});

class ConfigurationDataProvider : public ConfigurationDataProviderBase
{
private:
  std::unique_ptr<BallSpecification> theBallSpecification;
  std::unique_ptr<BehaviorParameters> theBehaviorParameters;
  std::unique_ptr<CameraCalibration> theCameraCalibration;
  std::unique_ptr<CameraIntrinsics> theCameraIntrinsics;
  std::unique_ptr<CameraSettings> theCameraSettings;
  std::unique_ptr<DamageConfigurationBody> theDamageConfigurationBody;
  std::unique_ptr<DamageConfigurationHead> theDamageConfigurationHead;
  std::unique_ptr<FieldDimensions> theFieldDimensions;
  std::unique_ptr<FootOffset> theFootOffset;
  std::unique_ptr<HeadLimits> theHeadLimits;
  std::unique_ptr<IMUCalibration> theIMUCalibration;
  std::unique_ptr<JointCalibration> theJointCalibration;
  std::unique_ptr<JointLimits> theJointLimits;
  std::unique_ptr<KeyframeMotionParameters> theKeyframeMotionParameters;
  std::unique_ptr<KickInfo> theKickInfo;
  std::unique_ptr<MassCalibration> theMassCalibration;
  std::unique_ptr<RelativeFieldColorsParameters> theRelativeFieldColorsParameters;
  std::unique_ptr<RobotDimensions> theRobotDimensions;
  std::unique_ptr<StaticJointPoses> theStaticJointPoses;
  std::unique_ptr<StiffnessSettings> theStiffnessSettings;
  std::unique_ptr<WalkModifier> theWalkModifier;

  void update(BallSpecification& ballSpecification) override {update(ballSpecification, theBallSpecification);}
  void update(BehaviorParameters& behaviorParameters) override {update(behaviorParameters, theBehaviorParameters);}
  void update(CameraCalibration& cameraCalibration) override;
  void update(CameraIntrinsics& cameraIntrinsics) override { update(cameraIntrinsics, theCameraIntrinsics); }
  void update(CameraSettings& cameraSettings) override {update(cameraSettings, theCameraSettings);}
  void update(DamageConfigurationBody& damageConfigurationBody) override {update(damageConfigurationBody, theDamageConfigurationBody);}
  void update(DamageConfigurationHead& damageConfigurationHead) override {update(damageConfigurationHead, theDamageConfigurationHead);}
  void update(FieldDimensions& fieldDimensions) override {update(fieldDimensions, theFieldDimensions);}
  void update(FootOffset& footOffset) override { update(footOffset, theFootOffset); }
  void update(HeadLimits& headLimits) override {update(headLimits, theHeadLimits);}
  void update(IMUCalibration& imuCalibration) override {update(imuCalibration, theIMUCalibration);}
  void update(JointCalibration& jointCalibration) override {update(jointCalibration, theJointCalibration);}
  void update(JointLimits& jointLimits) override;
  void update(KeyframeMotionParameters& keyframeMotionParameters) override { update(keyframeMotionParameters, theKeyframeMotionParameters); }
  void update(KickInfo& kickInfo) override;
  void update(MassCalibration& massCalibration) override {update(massCalibration, theMassCalibration);}
  void update(RelativeFieldColorsParameters& relativeFieldColorsParameters) override { update(relativeFieldColorsParameters, theRelativeFieldColorsParameters); }
  void update(RobotDimensions& robotDimensions) override;
  void update(StaticJointPoses& staticJointPoses) override { update(staticJointPoses, theStaticJointPoses); }
  void update(StiffnessSettings& stiffnessSettings) override {update(stiffnessSettings, theStiffnessSettings);}
  void update(WalkModifier& walkModifier) override {update(walkModifier, theWalkModifier);}

  template<typename T> void update(T& representation, std::unique_ptr<T>& theRepresentation)
  {
    if(theRepresentation)
    {
      representation = *theRepresentation;
      theRepresentation = nullptr;
    }
  }

  template<typename T> void read(std::unique_ptr<T>& theRepresentation, const char* fileName = nullptr)
  {
    ASSERT(!theRepresentation);
    theRepresentation = std::make_unique<T>();
    loadModuleParameters(*theRepresentation, TypeRegistry::demangle(typeid(T).name()).c_str(), fileName);
  }

public:
  ConfigurationDataProvider();
};
