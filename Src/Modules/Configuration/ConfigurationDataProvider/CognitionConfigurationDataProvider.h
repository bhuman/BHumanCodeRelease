/**
 * @file CognitionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
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
  PROVIDES(FieldDimensions),
  PROVIDES_WITHOUT_MODIFY(IntersectionRelations),
  PROVIDES(BallSpecification),
  PROVIDES(CameraCalibration),
  PROVIDES(CameraSettings),
  PROVIDES(FieldColors),
  PROVIDES(RobotDimensions),
  PROVIDES(BehaviorParameters),
  PROVIDES(DamageConfigurationBody),
  PROVIDES(DamageConfigurationHead),
  PROVIDES(HeadLimits),
});

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static thread_local CognitionConfigurationDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  std::unique_ptr<BallSpecification> theBallSpecification;
  std::unique_ptr<FieldDimensions> theFieldDimensions;
  std::unique_ptr<FieldColors> theFieldColors;
  std::unique_ptr<IntersectionRelations> theIntersectionRelations;
  std::unique_ptr<CameraCalibration> theCameraCalibration;
  std::unique_ptr<CameraSettings> theCameraSettings;
  std::unique_ptr<RobotDimensions> theRobotDimensions;
  std::unique_ptr<BehaviorParameters> theBehaviorParameters;
  std::unique_ptr<DamageConfigurationBody> theDamageConfigurationBody;
  std::unique_ptr<DamageConfigurationHead> theDamageConfigurationHead;
  std::unique_ptr<HeadLimits> theHeadLimits;

  void update(BallSpecification& ballSpecification) override {update(ballSpecification, theBallSpecification);}
  void update(CameraCalibration& cameraCalibration) override;
  void update(CameraSettings& cameraSettings) override {update(cameraSettings, theCameraSettings);}
  void update(FieldDimensions& fieldDimensions) override;
  void update(FieldColors& fieldColors) override;
  void update(IntersectionRelations& intersectionRelations) override {update(intersectionRelations, theIntersectionRelations);}
  void update(RobotDimensions& robotDimensions) override {update(robotDimensions, theRobotDimensions);}
  void update(BehaviorParameters& behaviorParameters) override {update(behaviorParameters, theBehaviorParameters);}
  void update(DamageConfigurationBody& damageConfigurationBody) override {update(damageConfigurationBody, theDamageConfigurationBody);}
  void update(DamageConfigurationHead& damageConfigurationHead) override {update(damageConfigurationHead, theDamageConfigurationHead);}
  void update(HeadLimits& headLimits) override {update(headLimits, theHeadLimits);}

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
  CognitionConfigurationDataProvider();
  ~CognitionConfigurationDataProvider();

  /**
   * Called from a MessageQueue to distribute messages
   * @param message The message that can be read
   * @return true if the message was handled
   */
  static bool handleMessage(InMessage& message);
};
