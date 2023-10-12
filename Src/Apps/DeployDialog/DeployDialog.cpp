/**
 * @file DeployDialog.h
 *
 * This file implements a class that  represents the main window of the deploy dialog.
 * It also hosts all processes that either ping the robots or poll status updates from them.
 *
 * @author Thomas Röfer
 */

#include "DeployDialog.h"
#include <regex>
#include <set>
#include <QApplication>
#include <QDir>
#include <QMenu>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QVBoxLayout>
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#ifdef MACOS
#include <Carbon/Carbon.h>
#endif

DeployDialog::DeployDialog(int argc, char* argv[])
{
  setWindowTitle("Deploy");
  setWindowIcon(QPixmap(":Icons/DeployDialog.png"));

  QSettings settings("B-Human", "DeployDialog");
  QStringList substitutes = settings.value("substitutes", QStringList()).toStringList();

  gotoConfigDirectory(argc ? argv[0] : nullptr);

  // Read known robots.
  for(const QString& dir : QDir("Robots").entryList(QDir::Dirs, QDir::Name))
  {
    InMapFile stream("Robots/" + dir.toStdString() + "/network.cfg");
    if(stream.exists())
    {
      Robot robot;
      stream >> robot;
      robots[robot.name] = robot;
    }
  }

  // Read deploy presets.
  InMapFile presetsStream("teams.cfg");
  if(presetsStream.exists())
    presetsStream >> presets;

  // Create dialog contents.
  QHBoxLayout* clientAreaLayout = new QHBoxLayout(this);
  clientAreaLayout->setContentsMargins(0, 0, 0, 0);
  clientAreaLayout->setSpacing(0);
  clientAreaLayout->addWidget(createRobotsTable(substitutes));
  settingsArea = new SettingsArea(presets, this, table, settings);
  clientAreaLayout->addWidget(settingsArea);

  // Start background processes.
  createProcesses();

  show();
}

DeployDialog::~DeployDialog()
{
  // End background processes in reverse order.
  for(auto process = processes.rbegin(); process != processes.rend(); ++process)
  {
    (*process)->close();
    delete *process;
  }

  delete table;
}

void DeployDialog::gotoConfigDirectory([[maybe_unused]] const char* argv0)
{
#ifndef WINDOWS
  std::string path = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0).toStdString();
  QDir::setCurrent(path.substr(0, path.find_last_of("/\\")).c_str());
#endif
  QDir::setCurrent(QString(File::getBHDir()) + "/Config");
}

RobotsTable* DeployDialog::createRobotsTable(const QStringList& substitutes)
{
  // Determine the number of rows required for all the presets to be able
  // to assign the player numbers. In general, this should work even if
  // the team sizes are different for different presets.
  // It is also determined, which robots are already in use, i.e. assigned
  // in a preset.
  int numOfPlayers = 0;
  std::set<std::string> used;
  for(const Presets::Preset* team : presets.teams)
  {
    numOfPlayers += static_cast<int>(team->players.size());
    for(size_t i = 0; i < team->players.size(); ++i)
      if(team->players[i] != "_")
        used.insert(team->players[i]);
  }

  // Create the table with a rows for the player numbers of all presets and
  // the substitutes that are not assigned to any preset yet.
  table = new RobotsTable(static_cast<int>(robots.size() - used.size()) + numOfPlayers);

  // Since the teams.cfg can also be edited manually, it is made sure that each robot
  // is at most assigned to a single preset. The set \c unused contains the robots
  // that were not assigned yet, which is all of them at the beginning.
  std::set<std::string> unused;
  for(const auto& [name, _] : robots)
    unused.insert(name);

  // Create the table rows with player numbers for all presets.
  int row = 0;
  for(Presets::Preset* team : presets.teams)
  {
    int playerNumber = 1;
    for(size_t i = 0; i < team->players.size(); ++i)
    {
      std::string& robot = team->players[i];
      if(robot != "_" && unused.contains(robot))
      {
        table->setItem(row, 0, new QTableWidgetItem(robot.c_str()));
        const std::string ip = robots[robot].lan;
        table->setItem(row, 1, new QTableWidgetItem(ip.substr(ip.rfind('.') + 1).c_str()));
        table->item(row, 1)->setTextAlignment(Qt::AlignCenter);
        unused.erase(robot);
      }
      else
        robot = "_";
      table->setVerticalHeaderItem(row++, new QTableWidgetItem(QString::number(playerNumber++)));
    }
  }

  // The substitute robots should be added in the same sequence as they were stored
  // in the settings. However, some of them might have been assigned to presets,
  // because the teams.cfg was manually edited. Skip the latter.
  for(const QString& robot : substitutes)
    if(unused.contains(robot.toStdString()))
    {
      table->setVerticalHeaderItem(row, new QTableWidgetItem(""));
      table->setItem(row, 0, new QTableWidgetItem(robot));
      table->setItem(row, 1, new QTableWidgetItem(robots[robot.toStdString()].lan.substr(robots[robot.toStdString()].lan.rfind('.') + 1).c_str()));
      table->item(row++, 1)->setTextAlignment(Qt::AlignCenter);
      unused.erase(robot.toStdString());
    }

  // There also might be robots that were not listed as substitutes, but now they
  // are ones, because they were removed from presets by manually editing the
  // teams.cfg. Add them last.
  for(const std::string& robot : unused)
  {
    table->setVerticalHeaderItem(row, new QTableWidgetItem(""));
    table->setItem(row, 0, new QTableWidgetItem(robot.c_str()));
    table->setItem(row, 1, new QTableWidgetItem(robots[robot].lan.substr(robots[robot].lan.rfind('.') + 1).c_str()));
    table->item(row++, 1)->setTextAlignment(Qt::AlignCenter);
  }

  table->resizeColumnsToContents();

  // The is a single process that runs commands resulting from selecting popup
  // menu items.
  QProcess* process = new QProcess(this);
  processes.push_back(process);

  // Create a popup for a robot if it is connected.
  connect(table, &QTableWidget::customContextMenuRequested, [this, process](const QPoint& point)
  {
    const QModelIndex index = table->indexAt(point);

    // Is the pop menu requested while the mouse pointer is over a valid table cell?
    if(index.isValid() && table->item(index.row(), 0) && table->item(index.row(), 0)->text() != ""
       && process->state() == QProcess::NotRunning)
    {
      // Is the robot connected?
      const Robot& robot = robots[table->item(index.row(), 0)->text().toStdString()];
      std::string ip = table->getBetterIP(robot);
      if(ip.empty())
        return;

      // Create popup menu.
      QMenu menu("Robot", this);

      QAction* login = new QAction("&Login", this);
      connect(login, &QAction::triggered, [process, ip]
      {
        process->startCommand(localCommand("../Make/Common/robotTerminal " + ip).c_str());
      });
      menu.addAction(login);

      QAction* simRobot = new QAction("Connect Sim&Robot", this);
      connect(simRobot, &QAction::triggered, [process, ip]
      {
        process->startCommand(localCommand("../Make/Common/robotConnection " + ip).c_str());
      });
      menu.addAction(simRobot);

      // "Show Crash Log" is only enabled if there is one.
      QAction* crashLog = new QAction("Show &Crash Log", this);
      crashLog->setEnabled(table->item(index.row(), table->columnCount() - 1) && table->item(index.row(), table->columnCount() - 1)->text() == "yes");
      connect(crashLog, &QAction::triggered, [process, ip]
      {
        process->startCommand(localCommand("../Make/Common/robotTerminal " + ip + " \"\"\"cat bhdump.log; read\"\"\"").c_str());
      });
      menu.addAction(crashLog);

      menu.addSeparator();

      // "Download Calibration" is only enabled if there is one.
      QAction* downloadCalibration = new QAction("&Download Calibration", this);
      downloadCalibration->setEnabled(table->item(index.row(), 8) && table->item(index.row(), 8)->text() == "yes");
      connect(downloadCalibration, &QAction::triggered, [process, ip]
      {
        process->startCommand(localCommand("../Make/Common/downloadCalibration " + ip).c_str());
      });
      menu.addAction(downloadCalibration);

      menu.addSeparator();

      // Choose "Restart" or "Start" based on whether bhuman is currently running.
      QAction* restart = new QAction(table->item(index.row(), 9) && table->item(index.row(), 9)->text() == "run" ? "Res&tart" : "S&tart", this);
      connect(restart, &QAction::triggered, [process, ip]
      {
        process->startCommand(remoteCommand(ip, "systemctl --user restart bhuman.service </dev/null >/dev/null 2>&1 &").c_str());
      });
      menu.addAction(restart);

      // "Stop" is only enabled if bhuman is currently running.
      QAction* stop = new QAction("Sto&p", this);
      stop->setEnabled(table->item(index.row(), 9) && table->item(index.row(), 9)->text() == "run");
      connect(stop, &QAction::triggered, [process, ip]
      {
        process->startCommand(remoteCommand(ip, "systemctl --user stop bhuman.service </dev/null >/dev/null 2>&1 &").c_str());
      });
      menu.addAction(stop);

      QAction* shutdown = new QAction("&Shutdown", this);
      connect(shutdown, &QAction::triggered, [process, ip]
      {
        process->startCommand(remoteCommand(ip, "sudo systemctl poweroff </dev/null >/dev/null 2>&1 &").c_str());
      });
      menu.addAction(shutdown);

      QAction* reboot = new QAction("Re&boot", this);
      connect(reboot, &QAction::triggered, [process, ip]
      {
        process->startCommand(remoteCommand(ip, "sudo systemctl reboot </dev/null >/dev/null 2>&1 &").c_str());
      });
      menu.addAction(reboot);

      menu.exec(table->mapToGlobal(point));

      // Rows should not be selected, because the coloring for warnings would not
      // be visible anymore otherwise.
      table->clearSelection();
    }
  });

  return table;
}

void DeployDialog::createProcesses()
{
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert("LC_ALL", "C");
  for(auto& [name, robot] : robots)
  {
    // One process for status information per robot. It will be started by one of the ping
    // processes of the same robot.
    QProcess* statusProcess = new QProcess(this);
    connect(statusProcess, &QProcess::readyReadStandardOutput, this, [this, name = name]
    {
      const QStringList output = QString(dynamic_cast<QProcess*>(sender())->readAllStandardOutput()).trimmed().split(' ');
      if(output.size() != 9)
        for(int column = 4; column < table->columnCount(); ++column)
          table->setCell(name, column, "");
      else
      {
        int column = 4;
        const bool usb = output[4] == "yes";
        const bool active = output[7] == "active";
        const bool inactive = output[7] == "inactive";
        const int battery = active ? static_cast<int>(output[0].toFloat() * 100.f + 0.5f) : 100;
        const bool charging = !active || static_cast<short>(output[1].trimmed().toFloat()) & 0b10000000;
        const int temperature = active ? output[2].toInt() : 0;
        const int internalLogs = output[3].toInt() - (active && !usb ? 1 : 0); // Don't count log currently written
        const bool usbReadable= output[5] != "no";
        const int usbLogs = usbReadable ? output[5].toInt() - (active ? 1 : 0) : 0; // Don't count log currently written
        const bool calibration = output[6] == "yes";
        const bool dump = output[8] == "yes";

        table->setCell(name, column++, active ? std::to_string(battery) : "",
                       !charging ? Qt::red : battery < batteryWarnThreshold ? warnColor : QColor(),
                       !charging ? "Not charging" : battery < batteryWarnThreshold ? "Charge still enough?" : "");
        table->setCell(name, column++, temperature ? output[2].toStdString() : "",
                       temperature < temperatureWarnThreshold ? QColor() : temperature < temperatureCriticalThreshold ? warnColor : Qt::red,
                       temperature < temperatureWarnThreshold ? "" : temperature < temperatureCriticalThreshold ? "Hot" : "Critically hot");
        table->setCell(name, column++, std::to_string(internalLogs), internalLogs ? warnColor : QColor(),
                       internalLogs ? "Logs present on internal drive" : "");
        table->setCell(name, column++, usb && !usbReadable ? "fail" : !usb ? "no" : std::to_string(usbLogs),
                       !usbReadable ? Qt::red : usbLogs ? warnColor : QColor(),
                       usb && !usbReadable ? "USB drive not readable" : !usb ? "USB drive missing" : usbLogs ? "Logs present on USB drive" : "");
        table->setCell(name, column++, output[6].toStdString(), calibration ? warnColor : QColor(), calibration ? "New calibration present" : "");
        table->setCell(name, column++, active ? "run" : inactive ? "stop" : "fail",
                       active ? QColor() : inactive ? warnColor : Qt::red,
                       active ? "" : inactive ? "bhuman not running" : "bhuman probably crashed (" + output[7].toStdString() + ")");
        table->setCell(name, column++, output[8].toStdString(), dump ? Qt::red : QColor(),
                       dump ? "Crash log present" : "");
      }
    });
    processes.push_back(statusProcess);

    // Two ping processes per robot (lan, wlan).
    for(const std::string& ip : {robot.lan, robot.wlan})
    {
      QProcess* pingProcess = new QProcess(this);
      connect(pingProcess, &QProcess::readyReadStandardOutput, this,
              [this, &robot = robot, column = ip == robot.lan ? 2 : 3, statusProcess]
      {
        const QStringList output = QString(dynamic_cast<QProcess*>(sender())->readAllStandardOutput())
          .split(" ", Qt::SkipEmptyParts)
          .filter(QRegularExpression("time=\\d+(\\.\\d+)?"));
        if(output.empty())
          table->setCell(robot.name, column, "");
        else
        {
          QStringList keyAndValue = output[0].split(QRegularExpression("[<=]"), Qt::SkipEmptyParts);
          if(keyAndValue.size() < 2)
            table->setCell(robot.name, column, "");
          else
          {
            const int ping = static_cast<int>(keyAndValue[1].replace("ms", "").toDouble());
            table->setCell(robot.name, column, std::to_string(ping), ping < pingWarnThreshold ? QColor()
                           : ping < pingCriticalThreshold ? warnColor : Qt::red,
                           ping < pingWarnThreshold ? "" : ping < pingCriticalThreshold ? "Ping is slow" : "Ping is very slow");
          }
        }

        // If the robot can be pinged, get status information if enough time has passed.
        const std::string ip = table->getBetterIP(robot);
        if(ip.empty())
          for(int column = 4; column < table->columnCount(); ++column)
            table->setCell(robot.name, column, "");
        else if(Time::getRealTimeSince(robot.lastStatusUpdate) > statusUpdateTime && statusProcess->state() == QProcess::NotRunning)
        {
          statusProcess->startCommand(remoteCommand(ip, "( cat /var/volatile/tmp/internalState.txt; "
                                                          "echo -n ' '; "
                                                          "ls /home/nao/logs | wc -l; "
                                                          "echo -n ' '; "
                                                          "mount | grep media/usb >/dev/null && echo yes || echo no; "
                                                          "echo -n ' '; "
                                                          "ls >/dev/null 2>&1 /media/usb/logs && ls /media/usb/logs | wc -l || echo no; "
                                                          "echo -n ' '; "
                                                          "ls /home/nao/Config/settings.cfg"
                                                            " /home/nao/Config/Robots/*/*/cameraCalibration.cfg"
                                                            " /home/nao/Config/Robots/*/Body/imuCalibration.cfg 2>/dev/null"
                                                            " | xargs ls -t | head -1 | grep settings.cfg >/dev/null && echo no || echo yes; "
                                                          "echo -n ' '; "
                                                          "systemctl --user is-failed bhuman.service; "
                                                          "echo -n ' '; "
                                                          "[ -e /home/nao/bhdump.log ] && echo yes || echo no ) | tr -d '\\n'").c_str());
          robot.lastStatusUpdate = Time::getRealSystemTime();
        }
      });
      pingProcess->start("bash", QStringList() << "-c" << QString("ping ")
#ifndef MACOS
        + "-O "
#endif
        + ip.c_str());
      processes.push_back(pingProcess);
    }
  }
}

std::string DeployDialog::remoteCommand(const std::string& ip, std::string command)
{
#ifdef WINDOWS
  static std::regex regex("[\\^\\&\\|\\<\\>]");
  command = std::regex_replace(command, regex, "^$&");
#endif
  return
#ifdef WINDOWS
    "cmd /c "
#endif
    "bash -c \"../Make/Common/robotSSH nao@" + ip + " \"\"\"" + command + "\"\"\"\"";
}

std::string DeployDialog::localCommand(const std::string& command)
{
  return
#ifdef WINDOWS
    "cmd /c "
#endif
    "bash -c \"" + command + "\"";
}

void DeployDialog::done(int reason)
{
  QSettings settings("B-Human", "DeployDialog");
  bool save = reason == QDialog::Accepted;

  if(save)
    settingsArea->writeOutput(robots);
  else
  {
    Presets originalPresets;
    InMapFile presetsStream("teams.cfg");
    if(presetsStream.exists())
    {
      presetsStream >> originalPresets;

      // Determine whether the original players would result in same selection as the current ones.
      std::set<std::string> unused;
      for(const auto& [name, _] : robots)
        unused.insert(name);
      for(Presets::Preset* team : originalPresets.teams)
        for(size_t i = 0; i < team->players.size(); ++i)
        {
          std::string& robot = team->players[i];
          if(robot != "_" && unused.contains(robot))
            unused.erase(robot);
          else
            robot = "_";
        }
    }

    // Determine whether the original substitutes would result in same order as the current ones.
    QStringList substitutes = table->getSubstitutes();
    QStringList originalSubstitutes = settings.value("substitutes", QStringList()).toStringList();
    originalSubstitutes.removeIf([&](const QString& substitute) {return !substitutes.contains(substitute);});
    for(const auto& [name, _] : robots)
      if(!originalSubstitutes.contains(name.c_str()) && substitutes.contains(name.c_str()))
        originalSubstitutes.append(name.c_str());

    // If the settings changed, ask the user whether they should be saved.
    if(presets != originalPresets || settingsArea->modified(settings) || substitutes != originalSubstitutes)
    {
      QMessageBox msgBox;
      msgBox.setText("The settings have been modified.");
      msgBox.setInformativeText("Do you want to save your changes?");
      msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
      msgBox.setDefaultButton(QMessageBox::Save);
      switch(msgBox.exec())
      {
        case QMessageBox::Cancel:
          return;
        case QMessageBox::Save:
          save = true;
      }
    }
  }

  // Save settings if requested to do so.
  if(save)
  {
    settings.setValue("substitutes", table->getSubstitutes());
    OutMapFile stream("teams.cfg");
    stream << presets;

    // If "Close" is not checked, prevent closing the dialog by not calling the
    // overridden method if "Deploy/Write/Download/Delete" was clicked.
    if(!settingsArea->save(settings) && reason == QDialog::Accepted)
      return;
  }

  QDialog::done(reason);
}

int main(int argc, char* argv[])
{
#ifdef MACOS
  ProcessSerialNumber psn = { 0, kCurrentProcess };
  TransformProcessType(&psn, argc > 1 && std::string(argv[1]) == "--fullscreen" ? kProcessTransformToUIElementApplication : kProcessTransformToForegroundApplication);
#endif
  QApplication app(argc, argv);
#ifdef WINDOWS
  app.setStyle("fusion");
#endif
  DeployDialog deployDialog(argc, argv);
  app.connect(&app, &QApplication::lastWindowClosed, &app, &QApplication::quit);
  app.exec();
}
