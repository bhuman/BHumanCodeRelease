/**
 * @file LEDHandler.cpp
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author jeff
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "LEDHandler.h"
#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest)
{
  //reset
  FOREACH_ENUM(LEDRequest::LED, led)
    ledRequest.ledStates[led] = LEDRequest::off;

  //update
  if(thePhotoModeGenerator.isActive)
  {
    setPhotoModeLights(ledRequest);
  }
  else
  {
    setBothEyes(ledRequest);
    setChestButton(ledRequest);
    setLeftFoot(ledRequest);
    setRightFoot(ledRequest);
    setRightEar(ledRequest);
    setLeftEar(ledRequest);
    setHead(ledRequest);
  }
}

void LEDHandler::setRightEar(LEDRequest& ledRequest)
{
  //right ear -> battery
  setBatteryLevelInEar(ledRequest, LEDRequest::earsRight0Deg);
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
  if(theLibDemo.isOneVsOneDemoActive)
  {
    setBatteryLevelInEar(ledRequest, LEDRequest::earsLeft0Deg);
    return;
  }

  //left ear -> connected players
  //          + GameController connection lost -> freaky blinking
  if(!theGameState.gameControllerActive)
  {
    ledRequest.ledStates[LEDRequest::earsLeft324Deg] = LEDRequest::blinking;
    ledRequest.ledStates[LEDRequest::earsLeft144Deg] = LEDRequest::blinking;
  }

  if(!theGlobalTeammatesModel.teammates.empty())
  {
    for(int i = LEDRequest::earsLeft0Deg; i <= LEDRequest::earsLeft324Deg; i++)
      ledRequest.ledStates[static_cast<LEDRequest::LED>(i)] = LEDRequest::on;
  }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest,
                             bool left,
                             EyeColor col,
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
    case red:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + redOffset + i] = s;
      break;
    case green:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + greenOffset + i] = s;
      break;
    case blue:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + blueOffset + i] = s;
      break;
    case white:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + redOffset + i] = s;
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + greenOffset + i] = s;
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + blueOffset + i] = s;
      break;
    case magenta:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + redOffset + i] = halfState;
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + blueOffset + i] = s;
      break;
    case yellow:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + greenOffset + i] = halfState;
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + redOffset + i] = s;
      break;
    case cyan:
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + greenOffset + i] = halfState;
      for(int i = 0; i <= numOfLEDsPerColor; i++)
        ledRequest.ledStates[first + blueOffset + i] = s;
      break;
    default:
      FAIL("Unknown color.");
      break;
  }
}

void LEDHandler::setBothEyes(LEDRequest& ledRequest)
{
  // Both eyes become yellow, if the robot does not have ground contact
  if(!theGroundContactState.contact)
  {
    setEyeColor(ledRequest, true, yellow, LEDRequest::on);
    setEyeColor(ledRequest, false, yellow, LEDRequest::on);
    return;
  }
  // Set right eye color depending on the current role.
  // If the ball was seen, the left eye has the same color as the right eye.
  bool ballWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
  if(Role::isActiveRole(theStrategyStatus.role))
  {
    setEyeColor(ledRequest, false, red, LEDRequest::on);
    if(ballWasSeen)
      setEyeColor(ledRequest, true, red, LEDRequest::on);
  }
  else
  {
    switch(theStrategyStatus.position)
    {
      case Tactic::Position::goalkeeper:
      case Tactic::Position::attackingGoalkeeper:
        setEyeColor(ledRequest, false, blue, LEDRequest::on);
        if(ballWasSeen)
          setEyeColor(ledRequest, true, blue, LEDRequest::on);
        break;
      case Tactic::Position::defender:
      case Tactic::Position::defenderL:
      case Tactic::Position::defenderR:
        setEyeColor(ledRequest, false, white, LEDRequest::on);
        if(ballWasSeen)
          setEyeColor(ledRequest, true, white, LEDRequest::on);
        break;
      case Tactic::Position::midfielder:
      case Tactic::Position::midfielderM:
      case Tactic::Position::midfielderL:
      case Tactic::Position::midfielderR:
      case Tactic::Position::sacPasser:
        setEyeColor(ledRequest, false, green, LEDRequest::on);
        if(ballWasSeen)
          setEyeColor(ledRequest, true, green, LEDRequest::on);
        break;
      case Tactic::Position::forward:
      case Tactic::Position::forwardM:
      case Tactic::Position::forwardL:
      case Tactic::Position::forwardR:
        setEyeColor(ledRequest, false, cyan, LEDRequest::on);
        if(ballWasSeen)
          setEyeColor(ledRequest, true, cyan, LEDRequest::on);
        break;
      default:
        ASSERT(theStrategyStatus.position == Tactic::Position::none);
        if(ballWasSeen)
        {
          setEyeColor(ledRequest, false, magenta, LEDRequest::on);
          setEyeColor(ledRequest, true, magenta, LEDRequest::on);
        }
    }
  }
}

void LEDHandler::setHead(LEDRequest& ledRequest)
{
  if(theSystemSensorData.batteryCharging)
  {
    ++chargingLED %= (LEDRequest::numOfHeadLEDs * chargingLightSlowness);
    const LEDRequest::LED currentLED = headLEDCircle[chargingLED / chargingLightSlowness];
    const LEDRequest::LED nextLED = headLEDCircle[(chargingLED / chargingLightSlowness + 1u) % LEDRequest::numOfHeadLEDs];
    ledRequest.ledStates[currentLED] = LEDRequest::on;
    ledRequest.ledStates[nextLED] = LEDRequest::on;
  }
  //  Show if Robot wwants to make a pass or receive one, maybe add different signs for both situations later on
  else if(theBehaviorStatus.passTarget > 0 || theBehaviorStatus.passOrigin > 0)
  {
    for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
      ledRequest.ledStates[i] = LEDRequest::on;
  }
  //Added for Demos, so we know when a robot is in heat and needs a pause
  else if(theLibDemo.isDemoActive && static_cast<int>(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]) > tempForLEDFastBlinking)
  {
    for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
      ledRequest.ledStates[i] = LEDRequest::fastBlinking;
  }
  else if(theLibDemo.isDemoActive && static_cast<int>(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]) > tempForLEDBlinking)
  {
    for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 1))
      ledRequest.ledStates[i] = LEDRequest::blinking;
  }
  else if(theLibDemo.isDemoActive && static_cast<int>(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]) > tempForHalfLEDActive)
  {
    for(LEDRequest::LED i = LEDRequest::firstHeadLED; i <= LEDRequest::lastHeadLED; i = LEDRequest::LED(unsigned(i) + 2))
      ledRequest.ledStates[i] = LEDRequest::on;
  }
}

void LEDHandler::setChestButton(LEDRequest& ledRequest)
{
  if(theGameState.playerState == GameState::unstiff)
    ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::blinking;
  else if(theGameState.playerState == GameState::calibration)
  {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
  }
  else if(theGameState.isPenalized())
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
  else if(theGameState.state == GameState::standby)
  {
    ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
  }
  else if(theGameState.isReady())
    ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
  else if(theGameState.isSet())
  {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::half;
  }
  else if(theGameState.isPlaying())
    ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
}

void LEDHandler::setLeftFoot(LEDRequest& ledRequest)
{
  switch(theGameState.color())
  {
    case GameState::Team::Color::orange:
      ledRequest.ledStates[LEDRequest::footLeftGreen] = LEDRequest::half;
    case GameState::Team::Color::red:
      ledRequest.ledStates[LEDRequest::footLeftRed] = LEDRequest::on;
      break;
    case GameState::Team::Color::white:
      ledRequest.ledStates[LEDRequest::footLeftBlue] = LEDRequest::on;
    case GameState::Team::Color::yellow:
      ledRequest.ledStates[LEDRequest::footLeftRed] = LEDRequest::on;
    case GameState::Team::Color::green:
      ledRequest.ledStates[LEDRequest::footLeftGreen] = LEDRequest::on;
      break;
    case GameState::Team::Color::purple:
      ledRequest.ledStates[LEDRequest::footLeftRed] = LEDRequest::on;
    case GameState::Team::Color::blue:
      ledRequest.ledStates[LEDRequest::footLeftBlue] = LEDRequest::on;
      break;
    case GameState::Team::Color::gray:
      ledRequest.ledStates[LEDRequest::footLeftBlue] = LEDRequest::half;
    case GameState::Team::Color::brown: // more a darker yellow
      ledRequest.ledStates[LEDRequest::footLeftRed] = LEDRequest::half;
      ledRequest.ledStates[LEDRequest::footLeftGreen] = LEDRequest::half;
      break;
  }
}

void LEDHandler::setRightFoot(LEDRequest& ledRequest)
{
  if(theGameState.gameControllerActive && theGameState.isForOwnTeam())
  {
    ledRequest.ledStates[LEDRequest::footRightRed] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::footRightGreen] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::footRightBlue] = LEDRequest::on;
  }
}

void LEDHandler::setPhotoModeLights(LEDRequest& ledRequest)
{
  auto ledState = thePhotoModeGenerator.ledGroup == PhotoModeGenerator::LEDGroup::eyes && thePhotoModeGenerator.selectLEDs ? LEDRequest::blinking : LEDRequest::on;
  // set the eyes
  switch(thePhotoModeGenerator.lamps[PhotoModeGenerator::eyes])
  {
    case PhotoModeGenerator::red:
      setEyeColor(ledRequest, true, LEDHandler::red, ledState);
      setEyeColor(ledRequest, false, LEDHandler::red, ledState);
      break;
    case PhotoModeGenerator::green:
      setEyeColor(ledRequest, true, LEDHandler::green, ledState);
      setEyeColor(ledRequest, false, LEDHandler::green, ledState);
      break;
    case PhotoModeGenerator::blue:
      setEyeColor(ledRequest, true, LEDHandler::blue, ledState);
      setEyeColor(ledRequest, false, LEDHandler::blue, ledState);
      break;
    case PhotoModeGenerator::yellow:
      setEyeColor(ledRequest, true, LEDHandler::yellow, ledState);
      setEyeColor(ledRequest, false, LEDHandler::yellow, ledState);
      break;
    case PhotoModeGenerator::magenta:
      setEyeColor(ledRequest, true, LEDHandler::magenta, ledState);
      setEyeColor(ledRequest, false, LEDHandler::magenta, ledState);
      break;
    case PhotoModeGenerator::cyan:
      setEyeColor(ledRequest, true, LEDHandler::cyan, ledState);
      setEyeColor(ledRequest, false, LEDHandler::cyan, ledState);
      break;
    case PhotoModeGenerator::white:
      setEyeColor(ledRequest, true, LEDHandler::white, ledState);
      setEyeColor(ledRequest, false, LEDHandler::white, ledState);
      break;
    default:
      break;
  }

  ledState = thePhotoModeGenerator.ledGroup == PhotoModeGenerator::LEDGroup::ears && thePhotoModeGenerator.selectLEDs ? LEDRequest::blinking : LEDRequest::on;
  // set the ears
  if(thePhotoModeGenerator.lamps[PhotoModeGenerator::ears] % 2 == 0)
  {
    for(int i = LEDRequest::earsLeft0Deg; i < LEDRequest::chestRed; ++i)
    {
      ledRequest.ledStates[LEDRequest::LED(i)] = ledState;
    }
  }

  // set the chest
  ledState = thePhotoModeGenerator.ledGroup == PhotoModeGenerator::LEDGroup::chest && thePhotoModeGenerator.selectLEDs ? LEDRequest::blinking : LEDRequest::on;
  switch(thePhotoModeGenerator.lamps[PhotoModeGenerator::chest])
  {
    case PhotoModeGenerator::red:
      ledRequest.ledStates[LEDRequest::chestRed] = ledState;
      break;
    case PhotoModeGenerator::green:
      ledRequest.ledStates[LEDRequest::chestGreen] = ledState;
      break;
    case PhotoModeGenerator::blue:
      ledRequest.ledStates[LEDRequest::chestBlue] = ledState;
      break;
    case PhotoModeGenerator::yellow:
      ledRequest.ledStates[LEDRequest::chestRed] = ledState;
      ledRequest.ledStates[LEDRequest::chestGreen] = ledState;
      break;
    case PhotoModeGenerator::magenta:
      ledRequest.ledStates[LEDRequest::chestRed] = ledState;
      ledRequest.ledStates[LEDRequest::chestBlue] = ledState;
      break;
    case PhotoModeGenerator::cyan:
      ledRequest.ledStates[LEDRequest::chestGreen] = ledState;
      ledRequest.ledStates[LEDRequest::chestBlue] = ledState;
      break;
    case PhotoModeGenerator::white:
      ledRequest.ledStates[LEDRequest::chestRed] = ledState;
      ledRequest.ledStates[LEDRequest::chestGreen] = ledState;
      ledRequest.ledStates[LEDRequest::chestBlue] = ledState;
      break;
    default:
      break;
  }

  // provisional because nothing works
  //  setEyeColor(ledRequest, true, LEDHandler::white, LEDRequest::LEDState::on);
  //  setEyeColor(ledRequest, false, LEDHandler::white, LEDRequest::LEDState::on);
  //  for(int i = LEDRequest::earsLeft0Deg; i < LEDRequest::chestRed; ++i)
  //  {
  //    ledRequest.ledStates[LEDRequest::LED(i)] = LEDRequest::on;
  //  }
  //  ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
  //  ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
  //  ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
}

void LEDHandler::setBatteryLevelInEar(LEDRequest& ledRequest, LEDRequest::LED baseLED)
{
  int onLEDs = std::min(static_cast<int>(theSystemSensorData.batteryLevel / 0.1f), 9);

  for(int i = 0; i <= onLEDs; ++i)
    ledRequest.ledStates[baseLED + i] = LEDRequest::on;
}

MAKE_MODULE(LEDHandler);
