/**
 * @file ConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/GlobalOptions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RelativeFieldColorsParameters.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/KeyframeMotionParameters.h"
#include "Representations/MotionControl/WalkModifier.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"

MODULE(ConfigurationDataProvider,
{,
  USES(CameraCalibrationNext),
  PROVIDES_WITHOUT_MODIFY(IntersectionRelations),
  PROVIDES(BallSpecification),
  REQUIRES(BallSpecification),
  PROVIDES(CameraCalibration),
  PROVIDES(CameraSettings),
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(FieldDimensions),
  PROVIDES(FootOffset),
  PROVIDES(GlobalOptions),
  PROVIDES(HeadLimits),
  PROVIDES(IMUCalibration),
  PROVIDES(JointCalibration),
  PROVIDES(JointLimits),
  PROVIDES(KeyframeMotionParameters),
  PROVIDES(KickInfo),
  PROVIDES(MassCalibration),
  PROVIDES(RelativeFieldColorsParameters),
  PROVIDES(RobotDimensions),
  PROVIDES(SetupPoses),
  PROVIDES(StiffnessSettings),
  PROVIDES(WalkModifier),
});

class ConfigurationDataProvider : public ConfigurationDataProviderBase
{
private:
  static thread_local ConfigurationDataProvider* theInstance; /**< Points to the only instance of this class in this thread or is nullptr if there is none. */

  std::unique_ptr<BallSpecification> theBallSpecification;
  std::unique_ptr<CameraCalibration> theCameraCalibration;
  std::unique_ptr<CameraSettings> theCameraSettings;
  std::unique_ptr<DamageConfigurationBody> theDamageConfigurationBody;
  std::unique_ptr<DamageConfigurationHead> theDamageConfigurationHead;
  std::unique_ptr<FieldDimensions> theFieldDimensions;
  std::unique_ptr<FootOffset> theFootOffset;
  std::unique_ptr<GlobalOptions> theGlobalOptions;
  std::unique_ptr<HeadLimits> theHeadLimits;
  std::unique_ptr<IMUCalibration> theIMUCalibration;
  std::unique_ptr<IntersectionRelations> theIntersectionRelations;
  std::unique_ptr<JointCalibration> theJointCalibration;
  std::unique_ptr<JointLimits> theJointLimits;
  std::unique_ptr<KeyframeMotionParameters> theKeyframeMotionParameters;
  std::unique_ptr<KickInfo> theKickInfo;
  std::unique_ptr<MassCalibration> theMassCalibration;
  std::unique_ptr<RelativeFieldColorsParameters> theRelativeFieldColorsParameters;
  std::unique_ptr<RobotDimensions> theRobotDimensions;
  std::unique_ptr<SetupPoses> theSetupPoses;
  std::unique_ptr<StiffnessSettings> theStiffnessSettings;
  std::unique_ptr<WalkModifier> theWalkModifier;

  void update(BallSpecification& ballSpecification) override {update(ballSpecification, theBallSpecification);}
  void update(CameraCalibration& cameraCalibration) override;
  void update(CameraSettings& cameraSettings) override {update(cameraSettings, theCameraSettings);}
  void update(DamageConfigurationBody& damageConfigurationBody) override {update(damageConfigurationBody, theDamageConfigurationBody);}
  void update(DamageConfigurationHead& damageConfigurationHead) override {update(damageConfigurationHead, theDamageConfigurationHead);}
  void update(FieldDimensions& fieldDimensions) override {update(fieldDimensions, theFieldDimensions);}
  void update(FootOffset& footOffset) override { update(footOffset, theFootOffset); }
  void update(GlobalOptions& globalOptions) override {update(globalOptions, theGlobalOptions);}
  void update(HeadLimits& headLimits) override {update(headLimits, theHeadLimits);}
  void update(IMUCalibration& imuCalibration) override {update(imuCalibration, theIMUCalibration);}
  void update(IntersectionRelations& intersectionRelations) override {update(intersectionRelations, theIntersectionRelations);}
  void update(JointCalibration& jointCalibration) override {update(jointCalibration, theJointCalibration);}
  void update(JointLimits& jointLimits) override;
  void update(KeyframeMotionParameters& KeyframeMotionParameters) override {update(KeyframeMotionParameters, theKeyframeMotionParameters);}
  void update(KickInfo& kickInfo) override;
  void update(MassCalibration& massCalibration) override {update(massCalibration, theMassCalibration);}
  void update(RelativeFieldColorsParameters& relativeFieldColorsParameters) override { update(relativeFieldColorsParameters, theRelativeFieldColorsParameters); }
  void update(RobotDimensions& robotDimensions) override;
  void update(StiffnessSettings& stiffnessSettings) override {update(stiffnessSettings, theStiffnessSettings);}
  void update(SetupPoses& setupPoses) override {update(setupPoses, theSetupPoses);}
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
  ~ConfigurationDataProvider();
};
