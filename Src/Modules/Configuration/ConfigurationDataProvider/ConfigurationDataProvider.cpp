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
  ModuleContainer::addMessageHandler(handleMessage);

  read(theCameraSettings);

  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();
  theIntersectionRelations = std::make_unique<IntersectionRelations>(*theFieldDimensions);

  read(theBallSpecification);
  read(theCameraCalibration);
  read(theDamageConfigurationBody);
  read(theDamageConfigurationHead);
  read(theFieldColors);
  read(theGetUpPhase);
  read(theGlobalOptions);
  read(theHeadLimits);
  read(theIMUCalibration);
  read(theJointCalibration);
  read(theJointLimits);
  read(theMassCalibration);
  read(theRobotDimensions);
  read(theStiffnessSettings);
  read(theWalk2014Modifier);
}

ConfigurationDataProvider::~ConfigurationDataProvider()
{
  theInstance = nullptr;
}

void ConfigurationDataProvider::update(FieldColors& fieldColors)
{
  update(fieldColors, theFieldColors);
  DEBUG_RESPONSE_ONCE("representation:FieldColors:once")
    OUTPUT(idFieldColors, bin, fieldColors);
}

void ConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  update(cameraCalibration, theCameraCalibration);
  if(theCameraCalibrationNext.hasNext())
    cameraCalibration = theCameraCalibrationNext.getNext();
}

bool ConfigurationDataProvider::handleMessage(InMessage& message)
{
  if(theInstance && message.getMessageID() == idColorCalibration)
  {
    if(!theInstance->theFieldColors)
      theInstance->theFieldColors = std::make_unique<FieldColors>();
    message.bin >> *theInstance->theFieldColors;
    return true;
  }
  else
    return false;
}

void ConfigurationDataProvider::update(JointLimits& jointLimits)
{
  update(jointLimits, theJointLimits);
  DEBUG_RESPONSE_ONCE("representation:JointLimits:once") OUTPUT(idJointLimits, bin, jointLimits);
}

void ConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  update(robotDimensions, theRobotDimensions);
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

MAKE_MODULE(ConfigurationDataProvider, infrastructure)
