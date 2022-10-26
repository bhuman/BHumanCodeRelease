#include "CommandBar.h"
#include "cmdlib/Command.h"
#include "cmdlib/Commands.h"
#include "cmds/CompileCmd.h"
#include "cmds/DeployCmd.h"
#include "cmds/RestartCmd.h"
#include "models/Team.h"
#include "ui/Console.h"
#include "ui/TeamSelector.h"
#include <QAction>
#include <QKeySequence>
#include <QMessageBox>
#include <QString>

CommandBar::CommandBar(Console* console, TeamSelector* teamSelector) :
  QToolBar("Commands"),
  console(console)
{
  setObjectName("Commands");
  setMovable(false);

  QAction* deploy = addCommand("deploy", Commands::getInstance().get("deploy"), [teamSelector](const CommandArgs*& args)
                               {
                                 args = new DeployArgs(QString::fromStdString(teamSelector->getSelectedTeam() ? teamSelector->getSelectedTeam()->buildConfig : ""));
                                 return true;
                               });
  deploy->setShortcut(QKeySequence(static_cast<int>(Qt::CTRL) + static_cast<int>(Qt::Key_D)));
  deploy->setShortcutContext(Qt::ShortcutContext::ApplicationShortcut);
  deploy->setToolTip("Deploy the selected robots with the given settings");

  QAction* downloadLogs = addCommand("download logs", Commands::getInstance().get("downloadLogs"));
  downloadLogs->setShortcut(QKeySequence(static_cast<int>(Qt::CTRL) + static_cast<int>(Qt::Key_L)));
  downloadLogs->setShortcutContext(Qt::ShortcutContext::ApplicationShortcut);
  downloadLogs->setToolTip("Download logs from the selected robots");

  QAction* deleteLogs = addCommand("delete logs", Commands::getInstance().get("deleteLogs"), [](const CommandArgs*&)
                                   {
                                     QMessageBox msgBox;
                                     msgBox.setWindowTitle("Delete Log Files");
                                     msgBox.setText("All log files on the selected robots will be lost.\nThis action cannot be undone!\n\nAre you sure?");
                                     msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
                                     msgBox.setDefaultButton(QMessageBox::Cancel);
                                     msgBox.setIcon(QMessageBox::Warning);

                                     return msgBox.exec() == QMessageBox::Yes;
                                   });
  deleteLogs->setToolTip("Delete logs from the selected robots");

  QAction* downloadCalibration = addCommand("download calibration", Commands::getInstance().get("downloadCalibration"));
  downloadCalibration->setToolTip("Download calibration files from the selected robots");

  QAction* restart = addCommand("restart", Commands::getInstance().get("restart"), [](const CommandArgs*& args)
                                {
                                  args = new RestartArgs(RestartArgs::bhuman);
                                  return true;
                                });
  restart->setToolTip("Restart the bhuman service on the selected robots");

  QAction* ssh = addCommand("ssh", Commands::getInstance().get("ssh"));
  ssh->setToolTip("Open a remote shell on the selected robots");

  addSeparator();

  QAction* reboot = addCommand("reboot", Commands::getInstance().get("restart"), [](const CommandArgs*& args)
                               {
                                 QMessageBox msgBox;
                                 msgBox.setWindowTitle("Reboot");
                                 msgBox.setText("The selected robots will be rebooted.\n\nAre you sure?");
                                 msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
                                 msgBox.setDefaultButton(QMessageBox::Cancel);
                                 msgBox.setIcon(QMessageBox::Warning);

                                 return msgBox.exec() == QMessageBox::Yes && (args = new RestartArgs(RestartArgs::robot), true);
                               });
  reboot->setToolTip("Reboot the selected robots");

  QAction* shutdown = addCommand("shutdown", Commands::getInstance().get("shutdown"), [](const CommandArgs*&)
                                 {
                                   QMessageBox msgBox;
                                   msgBox.setWindowTitle("Shutdown");
                                   msgBox.setText("The selected robots will be shut down.\n\nAre you sure?");
                                   msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
                                   msgBox.setDefaultButton(QMessageBox::Cancel);
                                   msgBox.setIcon(QMessageBox::Warning);

                                   return msgBox.exec() == QMessageBox::Yes;
                                 });
  shutdown->setToolTip("Shut down the selected robots");

  addSeparator();

  QAction* compile = addCommand("compile", Commands::getInstance().get("compile"), [teamSelector](const CommandArgs*& args)
                                {
                                  args = new CompileArgs(QString::fromStdString(teamSelector->getSelectedTeam() ? teamSelector->getSelectedTeam()->buildConfig : "Develop"), "Nao");
                                  return true;
                                });
  compile->setToolTip("Compile the bhuman executable in the selected build configuration");

  QAction* simulator = addCommand("simulator", Commands::getInstance().get("sim"));
  simulator->setToolTip("Open SimRobot (in Develop) and connect to the selected robot");
}

QAction* CommandBar::addCommand(const QString& name,
                                const CommandBase* command,
                                const std::function<bool(const CommandArgs*&)>& argsGenerator)
{
  QAction* a = new QAction(name, this);
  addAction(a);
  connect(a, &QAction::triggered, this, [this, command, argsGenerator]
          {
            const CommandArgs* args = nullptr;
            if(!argsGenerator(args))
              delete args;
            else
              console->fireCommand(command, args);
          });
  return a;
}
