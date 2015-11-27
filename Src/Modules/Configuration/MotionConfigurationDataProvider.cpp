/**
 * @file MotionConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionConfigurationDataProvider.h"

MAKE_MODULE(MotionConfigurationDataProvider, motionInfrastructure)

MotionConfigurationDataProvider::MotionConfigurationDataProvider()
{
  readDamageConfigurationBody();
  readDamageConfigurationHead();
  readFieldDimensions();
  readJointCalibration();
  readMassCalibration();
  readMotionSettings();
  readRobotDimensions();
  readStiffnessSettings();
  readUsConfiguration();
}

MotionConfigurationDataProvider::~MotionConfigurationDataProvider()
{
  if(theDamageConfigurationBody)
    delete theDamageConfigurationBody;
  if(theDamageConfigurationHead)
    delete theDamageConfigurationHead;
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theJointCalibration)
    delete theJointCalibration;
  if(theMassCalibration)
    delete theMassCalibration;
  if(theMotionSettings)
    delete theMotionSettings;
  if(theRobotDimensions)
    delete theRobotDimensions;
  if(theStiffnessSettings)
    delete theStiffnessSettings;
  if(theUsConfiguration)
    delete theUsConfiguration;
}

void MotionConfigurationDataProvider::update(DamageConfigurationBody& damageConfigurationBody)
{
  if(theDamageConfigurationBody)
  {
    damageConfigurationBody = *theDamageConfigurationBody;
    delete theDamageConfigurationBody;
    theDamageConfigurationBody = 0;
  }
}

void MotionConfigurationDataProvider::update(DamageConfigurationHead& damageConfigurationHead)
{
  if(theDamageConfigurationHead)
  {
    damageConfigurationHead = *theDamageConfigurationHead;
    delete theDamageConfigurationHead;
    theDamageConfigurationHead = 0;
  }
}

void MotionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = nullptr;
  }
}

void MotionConfigurationDataProvider::update(JointCalibration& jointCalibration)
{
  if(theJointCalibration)
  {
    jointCalibration = *theJointCalibration;
    delete theJointCalibration;
    theJointCalibration = nullptr;
  }
  DEBUG_RESPONSE_ONCE("representation:JointCalibration:once") OUTPUT(idJointCalibration, bin, jointCalibration);
}

void MotionConfigurationDataProvider::update(MassCalibration& massCalibration)
{
  if(theMassCalibration)
  {
    massCalibration = *theMassCalibration;
    delete theMassCalibration;
    theMassCalibration = nullptr;
  }
}

void MotionConfigurationDataProvider::update(MotionSettings& motionSettings)
{
  if(theMotionSettings)
  {
    motionSettings = *theMotionSettings;
    delete theMotionSettings;
    theMotionSettings = nullptr;
  }
}

void MotionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = nullptr;
  }
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

void MotionConfigurationDataProvider::update(StiffnessSettings& stiffnessSettings)
{
  if(theStiffnessSettings)
  {
    stiffnessSettings = *theStiffnessSettings;
    delete theStiffnessSettings;
    theStiffnessSettings = nullptr;
  }
}

void MotionConfigurationDataProvider::update(UsConfiguration& usConfiguration)
{
  if(theUsConfiguration)
  {
    usConfiguration = *theUsConfiguration;
    delete theUsConfiguration;
    theUsConfiguration = nullptr;
  }
}

void MotionConfigurationDataProvider::readDamageConfigurationBody()
{
  ASSERT(!theDamageConfigurationBody);

  InMapFile stream("damageConfigurationBody.cfg");

  if(stream.exists())
  {
    theDamageConfigurationBody = new DamageConfigurationBody;
    stream >> *theDamageConfigurationBody;
  }
}

void MotionConfigurationDataProvider::readDamageConfigurationHead()
{
  ASSERT(!theDamageConfigurationHead);

  InMapFile stream("damageConfigurationHead.cfg");

  if(stream.exists())
  {
    theDamageConfigurationHead = new DamageConfigurationHead;
    stream >> *theDamageConfigurationHead;
  }
}

void MotionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
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

void MotionConfigurationDataProvider::readMotionSettings()
{
  ASSERT(!theMotionSettings);

  InMapFile stream("motionSettings.cfg");
  if(stream.exists())
  {
    theMotionSettings = new MotionSettings;
    stream >> *theMotionSettings;
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

void MotionConfigurationDataProvider::readStiffnessSettings()
{
  ASSERT(!theStiffnessSettings);

  InMapFile stream("stiffnessSettings.cfg");
  if(stream.exists())
  {
    theStiffnessSettings = new StiffnessSettings;
    stream >> *theStiffnessSettings;
  }
}

void MotionConfigurationDataProvider::readUsConfiguration()
{
  ASSERT(!theUsConfiguration);

  InMapFile stream("usConfiguration.cfg");
  if(stream.exists())
  {
    theUsConfiguration = new UsConfiguration;
    stream >> *theUsConfiguration;
  }
}
