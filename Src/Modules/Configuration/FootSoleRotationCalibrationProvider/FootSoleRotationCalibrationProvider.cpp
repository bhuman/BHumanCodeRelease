/*
 * @file FootSoleRotationCalibrationProvider.cpp
 * @author Philip Reichenberg
 */

#include "FootSoleRotationCalibrationProvider.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <cmath>

MAKE_MODULE(FootSoleRotationCalibrationProvider, sensing);

FootSoleRotationCalibrationProvider::FootSoleRotationCalibrationProvider()
{
  InMapFile stream("footSoleRotationCalibration.cfg");
  ASSERT(stream.exists());
  stream >> const_cast<FootSoleRotationCalibration&>(theFootSoleRotationCalibration);
}

void FootSoleRotationCalibrationProvider::update(FootSoleRotationCalibration& footSoleRotationCalibration)
{
  bool update = false;
  MODIFY_ONCE("module:FootSoleRotationCalibrationProvider:forceUpdate", update);
  if(update || theCalibrationRequest.serialNumberFootSoleRotationCalibration > serialNumbeFootSoleRotationCalibration)
  {
    serialNumbeFootSoleRotationCalibration = theCalibrationRequest.serialNumberFootSoleRotationCalibration;
    const Angle leftRot = theRobotModel.soleLeft.rotation.getYAngle();
    const Angle rightRot = theRobotModel.soleRight.rotation.getYAngle();
    // Both feet are rotated different, which might let the torso tilt even more
    if(std::abs(leftRot - rightRot) > maxLeftRightFootRotationDifference)
    {
      ANNOTATION("module:FootSoleRotationCalibrationProvider", "Feet are not parallel");
      OUTPUT_TEXT("Feet are not parallel");
      return;
    }
    // At least one foot has not full ground contact. This would result in an additional false offset of over 1 degree
    if(!(theFootSupport.footPressure[Legs::left].forwardPressure == theFootSupport.footPressure[Legs::left].hasPressure && theFootSupport.footPressure[Legs::left].backwardPressure == theFootSupport.footPressure[Legs::left].hasPressure &&
         theFootSupport.footPressure[Legs::right].forwardPressure == theFootSupport.footPressure[Legs::right].hasPressure && theFootSupport.footPressure[Legs::right].backwardPressure == theFootSupport.footPressure[Legs::right].hasPressure &&
         theFootSupport.footPressure[Legs::left].hasPressure == theFootSupport.footPressure[Legs::right].hasPressure &&
         theFootSupport.footPressure[Legs::left].hasPressure == theFootSupport.timeSinceLastUpdate))
    {
      ANNOTATION("module:FootSoleRotationCalibrationProvider", "Underground is uneven");
      OUTPUT_TEXT("Underground is uneven");
      return;
    }

    // use average of all valid measurements
    usedMeasurements++;
    if(usedMeasurements == 0)
      footSoleRotationCalibration.yRotationOffset = 0_deg;
    else
      footSoleRotationCalibration.yRotationOffset *= static_cast<float>(usedMeasurements) - 1.f;
    const Angle measurendOffset = theInertialSensorData.angle.y() + (leftRot + rightRot) / 2.f;
    footSoleRotationCalibration.yRotationOffset += measurendOffset;
    footSoleRotationCalibration.yRotationOffset /= static_cast<float>(usedMeasurements);
    footSoleRotationCalibration.isCalibrated = true;

    ANNOTATION("module:FootSoleRotationCalibrationProvider", "Measured sole rotation offset: " << measurendOffset);

    //save the calibration
    std::string name = "footSoleRotationCalibration.cfg";
    for(std::string& fullName : File::getFullNames(name))
    {
      File path(fullName, "r", false);
      if(path.exists())
      {
        name = std::move(fullName);
        break;
      }
    }
    OutMapFile stream(name, true);
    ASSERT(stream.exists());
    stream << footSoleRotationCalibration;
  }

  // Too many false positives. TODO
  /*if(footSoleRotationCalibration.isCalibrated &&
     theGyroState.timestamp > 0 &&
     theMotionInfo.executedPhase == MotionPhase::stand &&  // is the robot standing?
     std::abs(theGyroState.mean.x()) < maxGyroMean &&
     std::abs(theGyroState.mean.y()) < maxGyroMean &&
     std::abs(theGyroState.mean.z()) < maxGyroMean &&
     std::abs(theInertialSensorData.angle.y() - theInertialData.angle.y()) < maxTorsoDifference &&
     std::abs(theInertialSensorData.angle.x() - theInertialData.angle.x()) < maxTorsoDifference)
  {
    const Angle leftRot = theRobotModel.soleLeft.rotation.getYAngle();
    const Angle rightRot = theRobotModel.soleRight.rotation.getYAngle();

    // Both feet are rotated different, which might let the torso tilt even more
    if(std::abs(leftRot - rightRot) > maxLeftRightFootRotationDifference)
      return;

    if(!(theFootSupport.footPressure[Legs::left].forwardPressure == theFootSupport.footPressure[Legs::left].hasPressure && theFootSupport.footPressure[Legs::left].backwardPressure == theFootSupport.footPressure[Legs::left].hasPressure &&
         theFootSupport.footPressure[Legs::right].forwardPressure == theFootSupport.footPressure[Legs::right].hasPressure && theFootSupport.footPressure[Legs::right].backwardPressure == theFootSupport.footPressure[Legs::right].hasPressure &&
         theFootSupport.footPressure[Legs::left].hasPressure == theFootSupport.footPressure[Legs::right].hasPressure))
      return;

    const Angle currentOffset = theInertialSensorData.angle.y() + (leftRot + rightRot) / 2.f;
    if(std::abs(currentOffset - footSoleRotationCalibration.yRotationOffset) > maxOffsetDifference)
    {
      footSoleRotationCalibration.isCalibrated = false;
      ANNOTATION("FootSoleRotationCalibration", "Too much rotation offset: " << currentOffset.toDegrees());
      SystemCall::say("Leg body connection is broken");
    }
  }*/
}
