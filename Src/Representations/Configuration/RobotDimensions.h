/**
* @file RobotDimensions.h
* Description of the Dimensions of the NAO Robot
* @author Cord Niehaus
*/

#pragma once

#include "Tools/Math/Vector3.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RobotDimensions,
{
public:
  /**
   * forward offset between head tilt joint and current camera
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getXHeadTiltToCamera(bool lowerCamera) const {return lowerCamera ? xHeadTiltToCamera : xHeadTiltToUpperCamera;}

  /**
   * height offset between head tilt joint and current camera
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getZHeadTiltToCamera(bool lowerCamera) const {return lowerCamera ? zHeadTiltToCamera : zHeadTiltToUpperCamera;}

  /**
   * tilt of current camera against head tilt
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getHeadTiltToCameraTilt(bool lowerCamera) const {return lowerCamera ? headTiltToCameraTilt : headTiltToUpperCameraTilt;},

  (float) lengthBetweenLegs,         //!<length between leg joints LL1 and LR1
  (float) upperLegLength,            //!<length between leg joints LL2 and LL3 in z-direction
  (float) lowerLegLength,            //!<length between leg joints LL3 and LL4 in z-direction
  (float) heightLeg5Joint,           //!<height of leg joints LL4 and LR4 of the ground

  (float) zLegJoint1ToHeadPan,       //!<height offset between LL1 and head pan joint
  (float) xHeadTiltToCamera,         //!<forward offset between head tilt joint and lower camera
  (float) zHeadTiltToCamera,         //!<height offset between head tilt joint and lower camera
  (float) headTiltToCameraTilt,      //!<tilt of lower camera against head tilt

  (float) xHeadTiltToUpperCamera,    //!<forward offset between head tilt joint and upper camera
  (float) zHeadTiltToUpperCamera,    //!<height offset between head tilt joint and upper camera
  (float) headTiltToUpperCameraTilt, //!<tilt of upper camera against head tilt

  (Vector3<>) armOffset,             //!<The offset of the first left arm joint relative to the middle between the hip joints
  (float) yElbowShoulder,            //!<The offset between the elbow joint and the shoulder joint in y
  (float) upperArmLength,            //!<The length between the shoulder and the elbow in y-direction
  (float) lowerArmLength,            //!<height off lower arm starting at arm2/arm3
});
