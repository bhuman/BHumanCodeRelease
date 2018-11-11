/**
 * @file LEDHandler.h
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author jeff
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/SitCommand.h"
#include "Tools/Module/Module.h"

MODULE(LEDHandler,
{,
  REQUIRES(BallModel),
  REQUIRES(BehaviorStatus),
  REQUIRES(FieldFeatureOverview),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  REQUIRES(SitCommand),
  REQUIRES(SystemSensorData),
  REQUIRES(TeamData),
  PROVIDES(LEDRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(5) chargingLightSlowness,
  }),
});

class LEDHandler : public LEDHandlerBase
{
public:
  ENUM(EyeColor,
  {,
    red,
    green,
    blue,
    white,
    magenta,
    yellow,
    cyan,
  });

private:
  void update(LEDRequest& ledRequest) override;

  void setEyeColor(LEDRequest& ledRequest,
                   bool left,
                   EyeColor col,
                   LEDRequest::LEDState s);

  void setRightEar(LEDRequest& ledRequest);
  void setLeftEar(LEDRequest& ledRequest);
  void setLeftEye(LEDRequest& ledRequest);
  void setRightEye(LEDRequest& ledRequest);
  void setHead(LEDRequest& ledRequest);
  void setChestButton(LEDRequest& ledRequest);

  size_t chargingLED = 0;
  const LEDRequest::LED headLEDCircle[LEDRequest::numOfHeadLEDs] =
  {
    LEDRequest::headLedRearLeft2,
    LEDRequest::headLedRearLeft1,
    LEDRequest::headLedRearLeft0,
    LEDRequest::headLedMiddleLeft0,
    LEDRequest::headLedFrontLeft0,
    LEDRequest::headLedFrontLeft1,
    LEDRequest::headLedFrontRight1,
    LEDRequest::headLedFrontRight0,
    LEDRequest::headLedMiddleRight0,
    LEDRequest::headLedRearRight0,
    LEDRequest::headLedRearRight1,
    LEDRequest::headLedRearRight2
  };
};
