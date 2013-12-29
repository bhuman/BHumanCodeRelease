/**
* @file LEDHandler.cpp
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#include "LEDHandler.h"
#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest)
{
  //reset
  for(int i = 0; i < ledRequest.numOfLEDs; ++i)
    ledRequest.ledStates[i] = LEDRequest::off;

  //update
  setRightEar(ledRequest);
  setLeftEar(ledRequest);
  setRightEye(ledRequest);
  setLeftEye(ledRequest);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest)
{
  //right ear -> battery

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar];

  int onLEDs = std::min((int)(theFilteredSensorData.data[FilteredSensorData::batteryLevel] / 0.1f), 9);

  for(int i = 0; i <= onLEDs; i++)
    ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
  if(theTeamMateData.numOfConnectedTeamMates > 0)
  {
    ledRequest.ledStates[LEDRequest::earsLeft0Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft36Deg] = LEDRequest::on;
  }
  if(theTeamMateData.numOfConnectedTeamMates > 1)
  {
    ledRequest.ledStates[LEDRequest::earsLeft72Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft108Deg] = LEDRequest::on;
  }
  if(theTeamMateData.numOfConnectedTeamMates > 2)
  {
    ledRequest.ledStates[LEDRequest::earsLeft180Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft216Deg] = LEDRequest::on;
  }
  if(theTeamMateData.numOfConnectedTeamMates > 3)
  {
    ledRequest.ledStates[LEDRequest::earsLeft252Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft288Deg] = LEDRequest::on;
  }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest,
                             bool left,
                             BehaviorLEDRequest::EyeColor col,
                             LEDRequest::LEDState s)
{
  LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

  static const int redOffset = 0,
                   greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
                   blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
                   numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;

  LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

  switch(col)
  {
  case BehaviorLEDRequest::defaultColor:
    ASSERT(false);
    break;
  case BehaviorLEDRequest::red:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    break;
  case BehaviorLEDRequest::green:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = s;
    break;
  case BehaviorLEDRequest::blue:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::white:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = s;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::magenta:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = halfState;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::yellow:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = halfState;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    break;
  case BehaviorLEDRequest::cyan:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = halfState;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  default:
    ASSERT(false);
    break;
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest)
{
  //left eye -> groundContact ? ballSeen and GoalSeen : blue

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];


  //no groundContact
  if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::defaultColor)
    //blue
    setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
  //default
  else
  {
    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;

    // TODO: this used to be seeing a whole goal, now it's only at least one post
    bool goalSeen = theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 250;

    if(ballSeen && goalSeen)
      //red
      setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    else if(ballSeen)
      //white
      setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
    else if(goalSeen)
      //green
      setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest)
{
  //right eye -> groundContact ? role : role -> blinking
  //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

  //no groundContact
  if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::defaultColor)
    setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);
  else
  {
    switch(theBehaviorControlOutput.behaviorStatus.role)
    {
    case BehaviorStatus::keeper:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
      break;
    case BehaviorStatus::striker:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
      break;
    case BehaviorStatus::defender:
      setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
      break;
    default:
      ASSERT(false);
    }
  }
}

MAKE_MODULE(LEDHandler, Behavior Control)

