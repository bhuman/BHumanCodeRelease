/**
* @file MotionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "MotionConfigurationDataProvider.h"

MotionConfigurationDataProvider::MotionConfigurationDataProvider() :
  theJointCalibration(0),
  theSensorCalibration(0),
  theRobotDimensions(0),
  theMassCalibration(0),
  theHardnessSettings(0),
  theDamageConfiguration(0)
{
  readJointCalibration();
  readSensorCalibration();
  readRobotDimensions();
  readMassCalibration();
  readHardnessSettings();
  readDamageConfiguration();
}

MotionConfigurationDataProvider::~MotionConfigurationDataProvider()
{
  if(theJointCalibration)
    delete theJointCalibration;
  if(theSensorCalibration)
    delete theSensorCalibration;
  if(theRobotDimensions)
    delete theRobotDimensions;
  if(theMassCalibration)
    delete theMassCalibration;
  if(theHardnessSettings)
    delete theHardnessSettings;
  if(theDamageConfiguration)
    delete theDamageConfiguration;
}

void MotionConfigurationDataProvider::update(JointCalibration& jointCalibration)
{
  if(theJointCalibration)
  {
    jointCalibration = *theJointCalibration;
    delete theJointCalibration;
    theJointCalibration = 0;
  }
  DEBUG_RESPONSE_ONCE("representation:JointCalibration", OUTPUT(idJointCalibration, bin, jointCalibration););
}

void MotionConfigurationDataProvider::update(SensorCalibration& sensorCalibration)
{
  if(theSensorCalibration)
  {
    sensorCalibration = *theSensorCalibration;
    delete theSensorCalibration;
    theSensorCalibration = 0;
  }
}

void MotionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions", OUTPUT(idRobotDimensions, bin, robotDimensions););
}

void MotionConfigurationDataProvider::update(MassCalibration& massCalibration)
{
  if(theMassCalibration)
  {
    massCalibration = *theMassCalibration;
    delete theMassCalibration;
    theMassCalibration = 0;
  }
}

void MotionConfigurationDataProvider::update(HardnessSettings& hardnessSettings)
{
  if(theHardnessSettings)
  {
    hardnessSettings = *theHardnessSettings;
    delete theHardnessSettings;
    theHardnessSettings = 0;
  }
}

void MotionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration)
{
  if(theDamageConfiguration)
  {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void MotionConfigurationDataProvider::readJointCalibration()
{
  ASSERT(!theJointCalibration);

  InMapFile stream("jointCalibration.cfg");
  if(stream.exists())
  {
    theJointCalibration = new JointCalibration;
    stream >> *theJointCalibration;
  }
}

void MotionConfigurationDataProvider::readSensorCalibration()
{
  ASSERT(!theSensorCalibration);

  InMapFile stream("sensorCalibration.cfg");
  if(stream.exists())
  {
    theSensorCalibration = new SensorCalibration;
    stream >> *theSensorCalibration;
  }
}

void MotionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if(stream.exists())
  {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void MotionConfigurationDataProvider::readMassCalibration()
{
  ASSERT(!theMassCalibration);

  InMapFile stream("massCalibration.cfg");
  if(stream.exists())
  {
    theMassCalibration = new MassCalibration;
    stream >> *theMassCalibration;
  }
}

void MotionConfigurationDataProvider::readHardnessSettings()
{
  ASSERT(!theHardnessSettings);

  InMapFile stream("hardnessSettings.cfg");
  if(stream.exists())
  {
    theHardnessSettings = new HardnessSettings;
    stream >> *theHardnessSettings;
  }
}

void MotionConfigurationDataProvider::readDamageConfiguration()
{
  ASSERT(!theDamageConfiguration);

  InMapFile stream("damageConfiguration.cfg");

  if(stream.exists())
  {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}

MAKE_MODULE(MotionConfigurationDataProvider, Motion Infrastructure)
