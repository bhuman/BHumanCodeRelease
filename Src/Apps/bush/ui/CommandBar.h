#pragma once

#include <QToolBar>
#include <functional>

struct CommandArgs;
class CommandBase;
class Console;
class QAction;
class TeamSelector;

class CommandBar : public QToolBar
{
  Q_OBJECT

  Console* console;

public:
  CommandBar(Console* console, TeamSelector* teamSelector);

private:
  QAction* addCommand(const QString& name,
                      const CommandBase* command,
                      const std::function<bool(const CommandArgs*&)>& argsGenerator = [](const CommandArgs*&){ return true; });
};
