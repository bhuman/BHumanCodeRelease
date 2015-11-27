/**
 * File:   KickState.h
 * Author: arne
 *
 * Created on March 1, 2015, 1:01 PM
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Motion/DynamicMotionPrimitive.h"
#include "Tools/Modeling/PT2RobotModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Motion/ZmpPreviewController3.h"
#include "Representations/MotionControl/DmpKickRequest.h"
#include "Tools/Enum.h"
#include "Tools/Math/Kalman.h"

struct DmpKickState
{
  ENUM(DynKickMode,
  {,
    windUp,
    kick,
    back,
  });

  DmpKickState() : jointModel("PT2RobotModel.cfg")
  {}
  bool leftSupport; /**<True if the left foot is the support foot. False otherwise. */
  PT2RobotModel jointModel; /** Model of the robots current joint state */
  RobotModel robotModel; /**<A RobotModel initialized with the joints from the jointModel */
  Pose3f torsoInSupportSole;
  Vector3f kickSoleInSupportSole;
  Vector3f kickSoleZeroInSupportSole;
  Vector3f comInSole; /**<Current estimated com position */
  std::deque<float> xPreviews;
  std::deque<float> yPreviews;
  ZmpPreviewController3 zmpCtrlX;/**<The controller used to balance the robot */
  ZmpPreviewController3 zmpCtrlY;/**<The controller used to balance the robot */
  DmpKickRequest::DmpKickType kickType; /**<What kind of kick is executed right now. */
  DynKickMode dynKickMode;
  DynamicMotionPrimitive<3> dynDmp;
  DynamicMotionPrimitive<3> backDmp;//used to move back from the end position
  bool balanceReached; /**<Is false initially, will become true once a more or less balanced state has been reached */
  int additionalKickSteps; /**< number of additional steps after the trajectory has ended */
  float kickTime; /**<execution time based on end velocity */
  float kickSpeed;
  Vector3f kickTarget;
  Vector2f zmpInSole;
  Vector2f predZmpInSole; /**<predicted using the motor model */
  Vector2f initialPreviewPos; /**<The current zmp preview */
  int previewSteps;
  int currentPreviewStep;
  DynamicMotionPrimitive<3> imitatedDmp; /**<A dmp that follows a learned trajectory */
  Vector3f dmpStartInDmpZero; /**<The dmp start location in the dmp coordinate system */
  bool movingBack;
};
