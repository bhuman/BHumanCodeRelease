/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Platform/File.h"

PROCESS_LOCAL CognitionConfigurationDataProvider* CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider()
{
  theInstance = this;

  readFieldDimensions();
  readCameraCalibration();
  readColorCalibration();
  readRobotDimensions();
  readDamageConfigurationBody();
  readDamageConfigurationHead();
  readHeadLimits();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theColorCalibration)
    delete theColorCalibration;
  if(theRobotDimensions)
    delete theRobotDimensions;
  if(theDamageConfigurationBody)
    delete theDamageConfigurationBody;
  if(theDamageConfigurationHead)
    delete theDamageConfigurationHead;
  if(theHeadLimits)
    delete theHeadLimits;
  theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
  fieldDimensions.drawPolygons(theOwnTeamInfo.teamColor);
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if(theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
  else
  {
    if(theCameraCalibrationNext.hasNext())
    {
      cameraCalibration = const_cast<CameraCalibrationNext&>(theCameraCalibrationNext).getNext();
    }
  }
}

void CognitionConfigurationDataProvider::update(ColorTable& colorTable)
{
  if(theColorCalibration)
  {
    colorTable.fromColorCalibration(*theColorCalibration, colorCalibration);
    delete theColorCalibration;
    theColorCalibration = 0;
  }

  DEBUG_RESPONSE_ONCE("representation:ColorCalibration:once")
  {
    OUTPUT(idColorCalibration, bin, colorCalibration);
  }
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfigurationBody& damageConfigurationBody)
{
  if(theDamageConfigurationBody)
  {
    damageConfigurationBody = *theDamageConfigurationBody;
    delete theDamageConfigurationBody;
    theDamageConfigurationBody = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfigurationHead& damageConfigurationHead)
{
  if(theDamageConfigurationHead)
  {
    damageConfigurationHead = *theDamageConfigurationHead;
    delete theDamageConfigurationHead;
    theDamageConfigurationHead = 0;
  }
}

void CognitionConfigurationDataProvider::update(HeadLimits& headLimits)
{
  if(theHeadLimits)
  {
    headLimits = *theHeadLimits;
    delete theHeadLimits;
    theHeadLimits = 0;
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);

  InMapFile stream("cameraCalibration.cfg");
  if(stream.exists())
  {
    theCameraCalibration = new CameraCalibration;
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readColorCalibration()
{
  ASSERT(!theColorCalibration);

  InMapFile stream("colorCalibration.cfg");
  if(stream.exists())
  {
    theColorCalibration = new ColorCalibration;
    stream >> *theColorCalibration;
  }
}

void CognitionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if(stream.exists())
  {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void CognitionConfigurationDataProvider::readDamageConfigurationBody()
{
  ASSERT(!theDamageConfigurationBody);

  InMapFile stream("damageConfigurationBody.cfg");
  if(stream.exists())
  {
    theDamageConfigurationBody = new DamageConfigurationBody;
    stream >> *theDamageConfigurationBody;
  }
}

void CognitionConfigurationDataProvider::readDamageConfigurationHead()
{
  ASSERT(!theDamageConfigurationHead);

  InMapFile stream("damageConfigurationHead.cfg");
  if(stream.exists())
  {
    theDamageConfigurationHead = new DamageConfigurationHead;
    stream >> *theDamageConfigurationHead;
  }
}

void CognitionConfigurationDataProvider::readHeadLimits()
{
  ASSERT(!theHeadLimits);

  InMapFile stream("headLimits.cfg");
  if(stream.exists())
  {
    theHeadLimits = new HeadLimits;
    stream >> *theHeadLimits;
  }
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  if(theInstance && message.getMessageID() == idColorCalibration)
  {
    if(!theInstance->theColorCalibration)
      theInstance->theColorCalibration = new ColorCalibration;
    message.bin >> *theInstance->theColorCalibration;
    return true;
  }
  else
    return false;
}

MAKE_MODULE(CognitionConfigurationDataProvider, cognitionInfrastructure)
