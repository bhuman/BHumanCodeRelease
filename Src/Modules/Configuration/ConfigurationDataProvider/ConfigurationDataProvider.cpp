/**
 * @file ConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author Thomas Röfer
 */

#include "ConfigurationDataProvider.h"
#include "Tools/Framework/ModuleContainer.h"
#include "Tools/Settings.h"

thread_local ConfigurationDataProvider* ConfigurationDataProvider::theInstance = nullptr;

ConfigurationDataProvider::ConfigurationDataProvider()
{
  theInstance = this;

  read(theCameraSettings);

  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();
  theIntersectionRelations = std::make_unique<IntersectionRelations>(*theFieldDimensions);

  read(theBallSpecification);
  read(theCameraCalibration);
  read(theDamageConfigurationBody);
  read(theDamageConfigurationHead);
  read(theFootOffset);
  read(theGlobalOptions);
  read(theHeadLimits);
  read(theIMUCalibration);
  read(theJointCalibration);
  read(theJointLimits);
  read(theKeyframeMotionParameters);
  read(theKickInfo);
  read(theMassCalibration);
  read(theRelativeFieldColorsParameters);
  read(theRobotDimensions);
  read(theStiffnessSettings);
  read(theSetupPoses);
  read(theWalkModifier);
}

ConfigurationDataProvider::~ConfigurationDataProvider()
{
  theInstance = nullptr;
}

void ConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  update(cameraCalibration, theCameraCalibration);
  if(theCameraCalibrationNext.hasNext())
    cameraCalibration = theCameraCalibrationNext.getNext();
}

void ConfigurationDataProvider::update(JointLimits& jointLimits)
{
  update(jointLimits, theJointLimits);
  DEBUG_RESPONSE_ONCE("representation:JointLimits:once") OUTPUT(idJointLimits, bin, jointLimits);
}

void ConfigurationDataProvider::update(KickInfo& kickInfo)
{
  const bool wasEmpty = theKickInfo == nullptr;
  update(kickInfo, theKickInfo);
  if(wasEmpty)
    return;
  for(KickInfo::Kick& kick : kickInfo.kicks)
  {
    // Comment this in, if the velocity needs to be recalculated
    //kick.ballVelocity.min = BallPhysics::velocityForDistance(kick.range.min, ConfigurationDataProviderBase::theBallSpecification.friction);
    //kick.ballVelocity.max = BallPhysics::velocityForDistance(kick.range.max, ConfigurationDataProviderBase::theBallSpecification.friction);

    kick.range.min = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.min, 0.f), ConfigurationDataProviderBase::theBallSpecification.friction)).norm();
    kick.range.max = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.max, 0.f), ConfigurationDataProviderBase::theBallSpecification.friction)).norm();
  }
}

void ConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  update(robotDimensions, theRobotDimensions);
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

MAKE_MODULE(ConfigurationDataProvider, infrastructure);
