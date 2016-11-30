/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include "Tools/RobotParts/Joints.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(DamageConfigurationHead,
{,
  (bool)(false) audioChannel0Defect,
  (bool)(false) audioChannel1Defect,
});

STREAMABLE(DamageConfigurationBody,
{
  ENUM(BrokenStandUp,
  {,
    allFine, //try left and if not successful right
    onlyNormal,  //left stand foot based
    onlyMirrored, //right stand foot based
  });

  DamageConfigurationBody()
  {
    brokenFsrsOnLeftFoot.fill(false);
    brokenFsrsOnRightFoot.fill(false);
  },
  
  (bool)(false) weakLeftLeg,
  (bool)(false) weakRightLeg,
  (bool)(false) usLDefect,
  (bool)(false) usRDefect,
  (bool)(false) leftFootBumperDefect,
  (bool)(false) rightFootBumperDefect,
  (bool)(false) leftArmContactDefect,
  (bool)(false) rightArmContactDefect,
  (bool)(false) noFieldGenuflect,
  (BrokenStandUp)(allFine) brokenStandUp,
  (bool)(false) useOldStandUpFront,
  (bool)(false) useOldStandUpBack,
  (ENUM_INDEXED_ARRAY(bool, (FsrSensors) FsrSensor)) brokenFsrsOnLeftFoot,
  (ENUM_INDEXED_ARRAY(bool, (FsrSensors) FsrSensor)) brokenFsrsOnRightFoot,
  ((Joints) stdVectorJoint)(stdVectorJoint()) jointsToEraseStiffness,
});
