/**
* @file LEDHandler.h
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

MODULE(LEDHandler,
{,
  REQUIRES(BallModel),
  REQUIRES(BehaviorLEDRequest),
  REQUIRES(BehaviorStatus),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalPercept),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  REQUIRES(SystemSensorData),
  REQUIRES(TeammateData),
  PROVIDES(LEDRequest),
});

class LEDHandler : public LEDHandlerBase
{
private:
  void update(LEDRequest& ledRequest);

  void setEyeColor(LEDRequest& ledRequest,
                   bool left,
                   BehaviorLEDRequest::EyeColor col,
                   LEDRequest::LEDState s);

  void setRightEar(LEDRequest& ledRequest);
  void setLeftEar(LEDRequest& ledRequest);
  void setLeftEye(LEDRequest& ledRequest);
  void setRightEye(LEDRequest& ledRequest);
  void setHead(LEDRequest& ledRequest);
  void setChestButton(LEDRequest& ledRequest);
};
