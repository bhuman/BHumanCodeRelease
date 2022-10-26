#pragma once

#include "cmdlib/Command.h"
#include "cmdlib/Context.h"
#include "models/Robot.h"

class Context;

class RobotCommandTools
{
public:
  /**
   * Checks whether any robot is selected in the given context.
   * @param context The context to check for selected robots.
   * @return Whether any robot is selected.
   */
  static bool checkSelectedRobots(Context& context);

  /**
   * Checks whether a robot is reachable (via the device from the team selcted in a context).
   * @param context The context in which to check reachability.
   * @param robot The robot to check.
   * @param status Is set to false if the robot is not reachable, otherwise it is not changed.
   * @return Whether the robot is reachable.
   */
  static bool checkReachable(Context& context, Robot* robot, bool& status);
};

/**
 * @class RobotCommand
 * Base class for commands that do something for each selected robot.
 * @tparam Derived The derived class.
 * @tparam Args The type of arguments for this command.
 */
template<typename Derived, typename Args = CommandArgs>
class RobotCommand : public Command<Derived, Args>
{
protected:
  using Command<Derived, Args>::Command;

  /**
   * Executes the preExecution(), perRobotExecution() and postExecution() in
   * this order.
   * @param context The context of the command.
   * @param args The arguments of the command.
   * @return false if one of the called methods returned false, true otherwise.
   */
  bool execute(Context& context, const Args& args) const override final
  {
    if(!RobotCommandTools::checkSelectedRobots(context))
      return false;
    if(!preExecution(context, args))
      return false;
    bool result = true;
    for(Robot* robot : context.getSelectedRobots())
    {
      if(!RobotCommandTools::checkReachable(context, robot, result))
        continue;
      Task* task = perRobotExecution(context, *robot, args);
      if(task)
        context.executeDetached(task);
    }
    result &= context.waitForChildren();
    result &= postExecution(context, args);
    return result;
  }

  /**
   * This method can be either used to execute code for every robot separately
   * or to construct a RobotTask for the robot, which is executed concurrently
   * after the method returned it.
   * @param context The context of the command.
   * @param robot The robot for which the code should be executed.
   * @param args The arguments of the command.
   * @return  Should return a pointer to a RobotTask-Object but also can return 0
   *          to indicate that no task should be executed.
   */
  virtual Task* perRobotExecution(Context& context, Robot& robot, const Args& args) const = 0;

  /**
   * This method is intended to do something which has to be done before the command is
   * specialized for every single robot.
   * @param context The context of the command.
   * @param args The arguments of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool preExecution([[maybe_unused]] Context& context, [[maybe_unused]] const Args& args) const { return true; }

  /**
   * This method is intended to do cleanup tasks after the command handled every
   * robot separately.
   * @param context The context of the command.
   * @param args The arguments of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool postExecution([[maybe_unused]] Context& context, [[maybe_unused]] const Args& args) const { return true; }
};

/**
 * @class RobotCommand
 * Specialization for robot commands with default (i.e. no) arguments.
 * @tparam Derived The derived class.
 */
template<typename Derived>
class RobotCommand<Derived, CommandArgs> : public Command<Derived, CommandArgs>
{
protected:
  using Command<Derived, CommandArgs>::Command;

  /**
   * Executes the preExecution(), perRobotExecution() and postExecution() in
   * this order.
   * @param context The context of the command.
   * @return false if one of the called methods returned false, true otherwise.
   */
  bool execute(Context& context) const override final
  {
    if(!RobotCommandTools::checkSelectedRobots(context))
      return false;
    if(!preExecution(context))
      return false;
    bool result = true;
    for(Robot* robot : context.getSelectedRobots())
    {
      if(!RobotCommandTools::checkReachable(context, robot, result))
        continue;
      Task* task = perRobotExecution(context, *robot);
      if(task)
        context.executeDetached(task);
    }
    result &= context.waitForChildren();
    result &= postExecution(context);
    return result;
  }

  /**
   * This method can be either used to execute code for every robot separately
   * or to construct a RobotTask for the robot, which is executed concurrently
   * after the method returned it.
   * @param context The context of the command.
   * @param robot The robot for which the code should be executed.
   * @return  Should return a pointer to a RobotTask-Object but also can return 0
   *          to indicate that no task should be executed.
   */
  virtual Task* perRobotExecution(Context& context, Robot& robot) const = 0;

  /**
   * This method is intended to do something which has to be done before the command is
   * specialized for every single robot.
   * @param context The context of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool preExecution([[maybe_unused]] Context& context) const { return true; }

  /**
   * This method is intended to do cleanup tasks after the command handled every
   * robot separately.
   * @param context The context of the command.
   * @return Should return false to indicate an error.
   */
  virtual bool postExecution([[maybe_unused]] Context& context) const { return true; }
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
  RobotTask(Context& context, Robot* robot) :
    Task(context, true),
    robot(robot)
  {}

  std::string getLabel() override { return robot->name; }
};
