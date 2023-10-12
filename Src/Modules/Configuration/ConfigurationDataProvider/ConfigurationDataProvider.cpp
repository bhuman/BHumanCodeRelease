/**
 * @file ConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author Thomas Röfer
 */

#include "ConfigurationDataProvider.h"
#include "Debugging/Debugging.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Framework/Settings.h"

ConfigurationDataProvider::ConfigurationDataProvider()
{
  read(theCameraSettings);

  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();

  read(theBallSpecification);
  read(theBehaviorParameters);
  read(theCameraCalibration);
  read(theCameraIntrinsics);
  read(theDamageConfigurationBody);
  read(theDamageConfigurationHead);
  read(theFootOffset);
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
  read(theStaticJointPoses);
  read(theWalkModifier);
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
  if(theKickInfo) // Not copied to representation yet
  {
    update(kickInfo, theKickInfo);
    for(KickInfo::Kick& kick : kickInfo.kicks)
    {
      const BallSpecification& ballSpecification = theBallSpecification ? *theBallSpecification : ConfigurationDataProviderBase::theBallSpecification;

      // Comment this in, if the velocity needs to be recalculated
      //kick.ballVelocity.min = BallPhysics::velocityForDistance(kick.range.min, ballSpecification.friction);
      //kick.ballVelocity.max = BallPhysics::velocityForDistance(kick.range.max, ballSpecification.friction);

      kick.range.min = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.min, 0.f), ballSpecification.friction)).norm();
      kick.range.max = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.max, 0.f), ballSpecification.friction)).norm();
    }
  }
}

void ConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  update(robotDimensions, theRobotDimensions);
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

MAKE_MODULE(ConfigurationDataProvider);
