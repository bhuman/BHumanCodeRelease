/**
 * @file MotionConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionConfigurationDataProvider.h"

MAKE_MODULE(MotionConfigurationDataProvider, motionInfrastructure)

MotionConfigurationDataProvider::MotionConfigurationDataProvider()
{
  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();

  read(theDamageConfigurationBody);
  read(theDamageConfigurationHead);
  read(theGlobalOptions);
  read(theHeadLimits);
  read(theIMUCalibration);
  read(theJointCalibration);
  read(theJointLimits);
  read(theMassCalibration);
  read(theRobotDimensions);
  read(theStiffnessSettings);
  theWalkKicks = std::make_unique<WalkKicks>();
  readWalkKicks();
}

void MotionConfigurationDataProvider::update(JointLimits& jointLimits)
{
  update(jointLimits, theJointLimits);
  DEBUG_RESPONSE_ONCE("representation:JointLimits:once") OUTPUT(idJointLimits, bin, jointLimits);
}

void MotionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  update(robotDimensions, theRobotDimensions);
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

void MotionConfigurationDataProvider::update(WalkKicks& walkKicks)
{
  DEBUG_RESPONSE_ONCE("module:MotionConfigurationDataProvider:reloadKicks")
    readWalkKicks();
  update(walkKicks, theWalkKicks);
}

void MotionConfigurationDataProvider::readWalkKicks()
{
  FOREACH_ENUM(WalkKicks::Type, kick)
    if(kick != WalkKicks::none)
    {
      const std::string kickName = TypeRegistry::getEnumName(kick);
      InMapFile stream("WalkKicks/" + kickName + ".cfg");
      ASSERT(stream.exists());
      stream >> theWalkKicks->kicks[kick];
    }
}
