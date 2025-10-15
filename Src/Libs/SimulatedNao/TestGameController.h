/**
 * @file TestGameController.h
 *
 * This file declares a class that embeds the GameController in an
 * automatic testing environment.
 *
 * @author Nico Holsten
 * @author Thomas Röfer
 */

#pragma once

#include "GameController.h"
#include "TestController.h"

class TestGameController : public GameController, public TestController
{
public:

  /**
   * Starts a test with parameters provided.
   * @param testParams The parameters for the test.
   * @return True if the test was started successfully, false otherwise.
   */
  bool startTest(TestParameters testParams);

  /**
   * Stops the current test if it is running.
   */
  void stopTest();

  /**
   * Sets the test skill for a robot.
   * @param skill The skill to set.
   */
  void setTestSkill(SkillRequest::Type skill);

public:
  using GameController::initial;
  bool playing() override;
  bool finished() override;
  bool goal(int size) override;
  bool penalty(int robot, Penalty penalty) override;
  void update();
  bool loadTest();

private:
  void initSecondHalf();
};
