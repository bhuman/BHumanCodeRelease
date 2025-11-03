/**
 * @file TestGameController.cpp
 *
 * This file implements a class that embeds the GameController in an
 * automatic testing environment.
 *
 * @author Nico Holsten
 * @author Thomas Röfer
 */

#include "TestGameController.h"
#include "RoboCupCtrl.h"
#include "Platform/Time.h"
#include <MainWindow.h>
#include <QTimer>
#include <SimRobotCore3.h>
#include <SimRobotCore2D.h>

bool TestGameController::startTest(TestParameters testParams)
{
  if(!init(RoboCupCtrl::application->getFilePath(), testParams))
  {
    return false;
  }
  QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simStop()));
  QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simStart()));
  QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simReset()));
  return true;
}

void TestGameController::stopTest()
{
  TestController::stopTest();
  QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simStop()));
}

void TestGameController::setTestSkill(SkillRequest::Type skill)
{
  if(isSituationTestRunning() && runCheck(&TestController::checkRobotInSkill, skill))
  {
    finished();
    return;
  }
}

bool TestGameController::playing()
{
  const bool stateChanged = gameControllerData.state != STATE_PLAYING;
  if(GameController::playing())
  {
    if(stateChanged)
    {
      //we skip the 10s where robots are penalized when in playing in opposite half when in a situation test
      int skipSeconds = getTestParameters().testType == situation ? 10000 : 0;
      timeWhenStateBegan = Time::getCurrentSystemTime() - skipSeconds;
    }
    return true;
  }
  else
    return false;
}

bool TestGameController::finished()
{
  if(gameControllerData.state == STATE_FINISHED)
    return true;

  if(isTestRunning())
  {
    if(isGameTestRunning())
    {
      recordHalftimeData(gameControllerData.firstHalf ? 0 : 1,
                         gameControllerData.teams[0].score, gameControllerData.teams[1].score,
                         gameControllerData.teams[0].messageBudget, gameControllerData.teams[1].messageBudget);

      if(gameControllerData.firstHalf && gameControllerData.secsRemaining == 0)
      {
        GameController::finished();
        initSecondHalf();
        return false;
      }
    }

    TestAction action = finishTestCycle();
    switch(action)
    {
      case reset:
        QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simReset()));
        break;
      case stop:
        QTimer::singleShot(0, static_cast<MainWindow*>(RoboCupCtrl::application), SLOT(simStop()));
        break;
    }
    return true;
  }
  else
    return GameController::finished();
}

bool TestGameController::goal(int side)
{
  if(GameController::goal(side))
  {
    if(isSituationTestRunning() && runCheck(&TestController::checkGoal, side))
      finished();
    return true;
  }
  else
    return false;
}

bool TestGameController::penalty(int robot, Penalty penalty)
{
  if(isSituationTestRunning())
  {
    int robotIndex = robot + 1;
    QString robotName = QString::fromStdString("RoboCup.robots.robot" + std::to_string(robotIndex));
    if(runCheck(&TestController::checkPenalized, robotName))
    {
      finished();
      return true;
    }
  }
  return GameController::penalty(robot, penalty);
}

bool TestGameController::loadTest()
{
  return TestController::loadTest(fieldDimensions);
}

void TestGameController::update()
{
  GameController::update();

  if(!isSituationTestRunning())
  {
    return;
  }

  if(gameControllerData.secsRemaining >= 0 &&
     runCheck(&TestController::checkTime, gameControllerData.secsRemaining))
  {
    finished();
    return;
  }

  if(runCheck(&TestController::checkBudget, gameControllerData.teams[0].messageBudget, gameControllerData.teams[1].messageBudget))
  {
    finished();
    return;
  }

  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);

  if(gameControllerData.state == STATE_PLAYING && runCheck(&TestController::checkBallInZone, ballPos, fieldDimensions))
  {
    finished();
    return;
  }

  for(int i = 0; i < 2; ++i)
  {
    if(lastBallContactRobots[i] != nullptr)
    {
      if(runCheck(&TestController::checkRobotTouch, lastBallContactRobots[i]->getFullName()) ||
         runCheck(&TestController::checkTeamPossession, lastBallContactRobots[i]->getFullName()))
      {
        finished();
        return;
      }
    }
  }

  // Get names of all the robots from the test targets
  std::vector<QString> names = getRobotNames();
  std::vector<Vector2f> pos;
  for(QString& name : names)
  {
    if(!name.isEmpty())
    {
      SimRobot::Object* robot = RoboCupCtrl::application->resolveObject(name, RoboCupCtrl::controller->is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));

      // Save robot's position
      pos.push_back(robot != nullptr
                    ? SimulatedRobot::getPosition(robot)
                    : Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
    }
    else
      pos.push_back(Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
  }
  if(gameControllerData.state == STATE_PLAYING)
  {
    if(runCheck(&TestController::checkRobotInZone, pos, fieldDimensions))
    {
      finished();
      return;
    }
  }
}

void TestGameController::initSecondHalf()
{
  VERIFY(initial());
  VERIFY(setHalf(2));
  VERIFY(kickOff(1));
  VERIFY(ready());
}
