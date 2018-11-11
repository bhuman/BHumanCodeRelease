#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/cmdlib/Context.h"

class Context;

/**
 * A Command which is intended to be executed for one or more robots.
 */
class RobotCommand : public CommandAdapter
{
  bool status;

public:
  RobotCommand() : status(true) {}

  /**
   * Executes the preExecution(), perRobotExecution() and postExecution() in
   * this order.
   * NOTE: do not overwrite it. (btw: It won't have much effect since it isn't virtual.)
   * @param context The context of the command.
   * @param params The parameters of the command.
   * @return false if one of the called methods returned false, true otherwise.
   */
  bool execute(Context& context, const std::vector<std::string>& params) override;

  /**
   * This method is intended to do something which has to be done before the command is
   * specialized for every single robot.
   * @param context The context of the command.
   * @param params The parameters of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool preExecution(Context& context, const std::vector<std::string>& params) { return true; }

  /**
   * This method can be either used to execute code for every robot separately
   * or to construct a RobotTask for the robot, which is executed concurrently
   * after the method returned it.
   * @param context The context of the command.
   * @param robot The robot for which the code should be executed. Is never 0.
   * @return  Should return a pointer to a RobotTask-Object but also can return 0
   *          to indicate that no task should be executed.
   */
  virtual Task* perRobotExecution(Context& context, Robot& robot) = 0;

  /**
   * This method is intended to do cleanup tasks after the command handled every
   * robot separately.
   * @param context The context of the command.
   * @param params The parameters of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool postExecution(Context& context, const std::vector<std::string>& params) { return status; }
};

/**
 * A Task which can be executed concurrent for a specific robot.
 */
class RobotTask : public Task
{
protected:

  /**
   * The pointer to the robot for which the Task should be executed.
   * Should not be 0.
   */
  Robot* robot;

public:

  /**
   * Constructs a new RobotTask with the given context and the given robot
   * @param context The context of the task.
   * @param robot The robot for which the task should be executed.
   */
  RobotTask(Context& context, Robot* robot)
    : Task(context, true),
      robot(robot)
  {}

  std::string getLabel() override { return robot->name; }
};
