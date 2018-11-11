/**
 * @file CognitionConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CognitionConfigurationDataProvider.h"

thread_local CognitionConfigurationDataProvider* CognitionConfigurationDataProvider::theInstance = nullptr;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider()
{
  theInstance = this;

  read(theCameraSettings, ("cameraSettings" + std::string(TypeRegistry::getEnumName(theRobotInfo.headVersion)) + ".cfg").c_str());

  theFieldDimensions = std::make_unique<FieldDimensions>();
  theFieldDimensions->load();
  theIntersectionRelations = std::make_unique<IntersectionRelations>(*theFieldDimensions);

  read(theBallSpecification);
  read(theFieldColors, ("fieldColorsCalibration" + std::string(TypeRegistry::getEnumName(theRobotInfo.headVersion)) + ".cfg").c_str());
  read(theCameraCalibration);
  read(theRobotDimensions);
  read(theBehaviorParameters);
  read(theDamageConfigurationBody);
  read(theDamageConfigurationHead);
  read(theHeadLimits);
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  theInstance = nullptr;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  update(fieldDimensions, theFieldDimensions);
  fieldDimensions.drawPolygons(theOwnTeamInfo.teamColor);
}

void CognitionConfigurationDataProvider::update(FieldColors& fieldColors)
{
  update(fieldColors, theFieldColors);
  DEBUG_RESPONSE_ONCE("representation:FieldColors:once")
    OUTPUT(idFieldColors, bin, fieldColors);
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  update(cameraCalibration, theCameraCalibration);
  if(theCameraCalibrationNext.hasNext())
    cameraCalibration = theCameraCalibrationNext.getNext();
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
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

MAKE_MODULE(CognitionConfigurationDataProvider, cognitionInfrastructure)
