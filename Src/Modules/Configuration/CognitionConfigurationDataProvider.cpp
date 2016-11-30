/**
 * @file CognitionConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include <iostream>

thread_local CognitionConfigurationDataProvider* CognitionConfigurationDataProvider::theInstance = nullptr;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider()
{
  theInstance = this;

  readFieldColors();
  readFieldDimensions();
  readCameraCalibration();
  readCameraSettings();
  readRobotDimensions();
  readBehavior2015Parameters();
  readDamageConfigurationBody();
  readDamageConfigurationHead();
  readHeadLimits();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldColors)
    delete theFieldColors;
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theIntersectionRelations)
    delete theIntersectionRelations;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theCameraSettings)
    delete theCameraSettings;
  if(theRobotDimensions)
    delete theRobotDimensions;
  if(theBehavior2015Parameters)
    delete theBehavior2015Parameters;
  if(theDamageConfigurationBody)
    delete theDamageConfigurationBody;
  if(theDamageConfigurationHead)
    delete theDamageConfigurationHead;
  if(theHeadLimits)
    delete theHeadLimits;

  theInstance = nullptr;
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

void CognitionConfigurationDataProvider::update(FieldColors& fieldColors)
{
  if(theFieldColors)
  {
    fieldColors = *theFieldColors;
    delete theFieldColors;
    theFieldColors = 0;
  }
  DEBUG_RESPONSE_ONCE("representation:FieldColors:once")
    OUTPUT(idFieldColors, bin, fieldColors);
}

void CognitionConfigurationDataProvider::update(IntersectionRelations& intersectionRelations)
{
  if(theIntersectionRelations)
  {
    intersectionRelations = *theIntersectionRelations;
    delete theIntersectionRelations;
    theIntersectionRelations = 0;
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

void CognitionConfigurationDataProvider::update(CameraSettings& cameraSettings)
{
  if(theCameraSettings)
  {
    cameraSettings = *theCameraSettings;
    delete theCameraSettings;
    theCameraSettings = 0;
  }
}

void CognitionConfigurationDataProvider::update(Behavior2015Parameters& behavior2015Parameters)
{
  if(theBehavior2015Parameters)
  {
    behavior2015Parameters = *theBehavior2015Parameters;
    delete theBehavior2015Parameters;
    theBehavior2015Parameters = 0;
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
  ASSERT(!theIntersectionRelations);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();

  theIntersectionRelations = new IntersectionRelations(*theFieldDimensions);
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

void CognitionConfigurationDataProvider::readCameraSettings()
{
  ASSERT(!theCameraSettings);

  InMapFile stream("cameraSettings" + std::string(RobotInfo::getName(theRobotInfo.headVersion)) + ".cfg");
  if(stream.exists())
  {
    theCameraSettings = new CameraSettings;
    stream >> *theCameraSettings;
    theCameraSettings->upper.enforceBounds();
    theCameraSettings->lower.enforceBounds();
  }
}

void CognitionConfigurationDataProvider::readFieldColors()
{
  ASSERT(!theFieldColors);

  const std::string name = "fieldColorsCalibration" + std::string(RobotInfo::getName(theRobotInfo.headVersion)) + ".cfg";
  InMapFile stream(name);
  ASSERT(stream.exists());
  theFieldColors = new FieldColors;
  stream >> *theFieldColors;
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

void CognitionConfigurationDataProvider::readBehavior2015Parameters()
{
  ASSERT(!theBehavior2015Parameters);

  InMapFile stream("BehaviorControl2015/behavior2015Parameters.cfg");
  if(stream.exists())
  {
    theBehavior2015Parameters = new Behavior2015Parameters;
    stream >> *theBehavior2015Parameters;
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
    if(!theInstance->theFieldColors)
      theInstance->theFieldColors = new FieldColors;
    message.bin >> *theInstance->theFieldColors;
    return true;
  }
  else
    return false;
}

MAKE_MODULE(CognitionConfigurationDataProvider, cognitionInfrastructure)
