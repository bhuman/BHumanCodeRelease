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
  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();

  read(theBallSpecification);
  read(theBehaviorParameters);
  read(theCameraCalibration);
  read(theCameraIntrinsics);
  read(theHeadLimits);
  read(theIMUCalibration);
  read(theJointCalibration);
  read(theJointLimits);
  read(theKickInfo);
  read(theKickLengthPair);
  read(theMassCalibration);
  read(theMotorParameters);
  read(theRelativeFieldColorsParameters);
  read(theRobotDimensions);
  read(theStaticJointPoses);
}

void ConfigurationDataProvider::update(BehaviorParameters& behaviorParameters)
{
  update(behaviorParameters, theBehaviorParameters);
  ASSERT(behaviorParameters.noCommunicationAfterUnpenalizedTime < behaviorParameters.noSkillRequestAfterUnpenalizedTime);
  ASSERT(behaviorParameters.noSkillRequestAfterUnpenalizedTime < behaviorParameters.lookAroundAfterUnpenalizedTime);
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
      //kick.ballVelocity.min = BallPhysics::velocityForDistance(kick.range.min, ballSpecification);
      //kick.ballVelocity.max = BallPhysics::velocityForDistance(kick.range.max, ballSpecification);

      kick.range.min = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.min, 0.f), ballSpecification)).norm();
      kick.range.max = (BallPhysics::getEndPosition(Vector2f(0.f, 0.f), Vector2f(kick.ballVelocity.max, 0.f), ballSpecification)).norm();
    }
  }
}

void ConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  update(robotDimensions, theRobotDimensions);
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

MAKE_MODULE(ConfigurationDataProvider);
