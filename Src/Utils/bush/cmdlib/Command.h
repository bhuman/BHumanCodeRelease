#pragma once

#include <string>
#include <vector>

class Context;

class Command
{
public:

  /** This method should return a string which identifies the command.
   * The returned sting has to be unique in the collection of all commands held
   * by @link Commands and is not allowed to contain whitespaces.
   *
   * @return The name of the command conform to the requirements specified
   *         above.
   */
  virtual std::string getName() const = 0;

  /** This method is used by the @link HelpCmd to describe all known commands.
   * The string, returned by this method should describe command and its
   * parameters.
   *
   * @return The description as it should be displayed by help.
   */
  virtual std::string getDescription() const = 0;

  /** This method is called by @link Commands::execute in order to execute the
   * command. So here you can place the code which should be executed if the
   * command is called.
   *
   * @param context The @link Context which can be used to print output on the
   *                console or get the current state of the bush.
   * @param params  A list of parameters which are delivered by the bush.
   * @return Should return <em>false</em> if an error occured during the
   *         execution or the execution was not successful from another reason.
   *         Otherwise <em>true</em> should be returned.
   */
  virtual bool execute(Context& context, const std::vector<std::string>& params) = 0;

  /** This method should return a list of completion suggestions of the given
   * command line. The returned suggestions have to have the given command line
   * as prefix.
   *
   * @param cmdLine The current command line which should be completed.
   * @return The suggested completions.
   */
  virtual std::vector<std::string> complete(const std::string& cmdLine) const = 0;
};
