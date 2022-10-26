#pragma once

#include "Commands.h"
#include <string>

class Context;

/**
 * @struct CommandArgs
 * The base class for all argument containers.
 */
struct CommandArgs
{
  /** Virtual destructor for polymorphism. */
  virtual ~CommandArgs() = default;
};

/**
 * @class CommandBase
 * Do not inherit from this class directly, use the templates \c Command or \c RobotCommand instead.
 */
class CommandBase
{
protected:
  /**
   * Constructor. Initializes the name.
   * @param name The name of the command.
   */
  CommandBase(const std::string& name) :
    name(name)
  {}

public:
  const std::string name; /**< The name of the command. */

  /**
   * This method is called by @link Context::execute in order to execute the
   * command. It is implemented by the @link Command class which casts the
   * generic arguments to the type needed for this command.
   *
   * @param context The @link Context which can be used to print output on the
   *                console or get the current state of the bush.
   * @param args The generic arguments to the command.
   * @return Whether the command ran successfully.
   */
  virtual bool call(Context& context, const CommandArgs* args) const = 0;
};

/**
 * @class RegisteredCommand
 * A command that registers itself at \c Commands. Do not inherit from this class either.
 * @tparam Derived The derived class that should be registered.
 */
template<typename Derived>
class RegisteredCommand : public CommandBase
{
protected:
  /**
   * Constructor. Enforces that \c reg is called at static initialization time.
   * @param name The name of the command.
   */
  RegisteredCommand(const std::string& name) : CommandBase(name)
  {
    // This is needed to force a call to reg.
    static_cast<void>(_register);
  }

private:
  /** Registers a static instance of the command at the command registry. */
  static bool reg()
  {
    static Derived theCommandInstance;
    Commands::getInstance().add(&theCommandInstance);
    return true;
  }

  static bool _register; /**< Needed to force a call to \c reg. */
};

template<typename Derived>
bool RegisteredCommand<Derived>::_register = RegisteredCommand<Derived>::reg();


/**
 * @class Command
 * Base class for commands with arguments.
 * @tparam Derived The derived class.
 * @tparam Args The type of arguments for this command.
 */
template<typename Derived, typename Args = CommandArgs>
class Command : public RegisteredCommand<Derived>
{
protected:
  using RegisteredCommand<Derived>::RegisteredCommand;

  /**
   * The command implementation.
   * @param context The @link Context which can be used to print output on the
   *                console or get the current state of the bush.
   * @param args The arguments to the command.
   * @return Whether the command ran successfully.
   */
  virtual bool execute(Context& context, const Args& args) const = 0;

private:
  bool call(Context& context, const CommandArgs* args) const override
  {
    return execute(context, *static_cast<const Args*>(args));
  }
};

/**
 * @class Command
 * Specialization for commands with default (i.e. no) arguments.
 * @tparam Derived The derived class.
 */
template<typename Derived>
class Command<Derived, CommandArgs> : public RegisteredCommand<Derived>
{
protected:
  using RegisteredCommand<Derived>::RegisteredCommand;

  /**
   * The command implementation.
   * @param context The @link Context which can be used to print output on the
   *                console or get the current state of the bush.
   * @return Whether the command ran successfully.
   */
  virtual bool execute(Context& context) const = 0;

private:
  bool call(Context& context, const CommandArgs*) const override
  {
    return execute(context);
  }
};
