#pragma once

#include <map>
#include <string>
#include <vector>

class Command;
class Commands;
class Context;

class Commands
{
  std::map<std::string, Command*> commands;
public:
  static Commands& getInstance();
  bool addCommand(Command* cmd);

  /** Removes the given command.
   * @param cmd The command which should be removed. Must not be <em>0</em> and
   *            @link Command::getName() has to retun a proper string.
   * @return true if there was a command whose name equals the name of the given
   *         command and it could be removed.
   */
  bool removeCommand(const Command* cmd);

  /** Removes the command with the given name.
   * @param name The name of the command which should be removed.
   * @return true if there was a command with the given name which could be
   *         removed.
   */
  bool removeCommand(const std::string& name);

  /** Executes a command line with a given context.
   * Note: Do not call it from any command. Use the execute method of the given
   * Context object instead. (@link Context::execute)
   *
   * @param context The context in which the command line should be executed in. Must
   *                not be <em>0</em>.
   * @param cmdLine The command line which should be executed.
   * @return true if the command was executed sucessfully and false if an error
   *         occured or the command which sould be executed was not found.
   */
  bool execute(Context* context, const std::string& cmdLine);

  Command* operator[](const std::string& cmd);
  std::vector<Command*> getAllCommands();
  std::vector<std::string> getAllCommandNames() const;

  /** Returns possible completions of the given command line.
   *
   * First of all this mehtod completes all available command names if just one
   * word is given in the command line string.
   * If a second word exsists it will be delivered to the completion method of
   * the command whose name equals the first word of the command line string.
   *
   * Note: As required by the completer of gui, the prefix of all suggestions
   *       has to be the current command line. This differs from the prior
   *       implementation of this method.
   *
   * @param cmdLine The current command line as typed by the user.
   * @return A vector of competion suggestions.
   */
  std::vector<std::string> complete(const std::string& cmdLine);
};
