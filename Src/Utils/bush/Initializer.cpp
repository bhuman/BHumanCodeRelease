#include "Utils/bush/Initializer.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/agents/StatusAgent.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/ui/Console.h"

#include <iostream>
#include <cstdlib>

#include <QApplication>
#include "Tools/FunctionList.h"
#include "Utils/bush/ui/MainWindow.h"

Initializer::Initializer(int& argc, char** argv) : logLevel(WARN), app(0)
{
  log(TRACE, "Initializer: Initialization started.");

  FunctionList::execute();
#ifdef WINDOWS
  ProcessRunner r("taskkill /F /IM ping.exe");
#else // Linux, MACOS
  ProcessRunner r("ps axco pid,command | grep \" ping$\" | awk '{ print $1; }' | xargs kill");
#endif
  r.run();
  if(r.error())
    log(WARN, "Initializer: Could not kill any ping processes.");
  else
    log(TRACE, "Initializer: Killed all active ping processes.");

  log(TRACE, "Initializer: changing working directory to...");
  goToConfigDirectory(argv[0]);

  app = new QApplication(argc, argv);
#ifdef MACOS
  app->setStyle("macintosh");
#endif
  app->setApplicationName("B-Human User Shell (bush)");
  app->setCursorFlashTime(0);
  Icons::getInstance().init();
  log(TRACE, "Initializer: Created Qt application.");

  Session::getInstance();
  log(TRACE, "Initializer: Session instance created.");

  Session::getInstance().logLevel = WARN;
  log(TRACE, "Initializer: Set log level to " + toString(Session::getInstance().logLevel) + ".");

  Session::getInstance().pingAgent = new PingAgent;
  log(TRACE, "Initializer: Ping agent started.");

  Session::getInstance().statusAgent = new StatusAgent(Session::getInstance().pingAgent);
  log(TRACE, "Initializer: Power agent started.");

  Robot::initRobotsByName(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Robots loaded.");

  Session::getInstance().pingAgent->connect(&Session::getInstance(), SIGNAL(robotsChanged()), SLOT(robotsChanged()));
  Session::getInstance().pingAgent->initializeProcesses(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Registered robots at ping agent.");

  Session::getInstance().statusAgent->initialize(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Registered robots at power agent.");

  mainWindow = new MainWindow;
  mainWindow->show();
  log(TRACE, "Initializer: Created main window.");

  log(TRACE, "Initializer: Finished initialization.");
}

Initializer::~Initializer()
{
  log(TRACE, "Initializer: Shutdown started.");

  delete mainWindow;
  delete app;
  log(TRACE, "Initializer: Deleted GUI.");

  Session::getInstance().console = 0;
  log(TRACE, "Initializer: Removed console.");

  delete Session::getInstance().pingAgent;
  Session::getInstance().pingAgent = 0;
  log(TRACE, "Initializer: Removed ping angent.");

  log(TRACE, "Initializer: Finished shutdown.");
}

int Initializer::start()
{
  return app->exec();
}

void Initializer::log(LogLevel logLevel, const std::string& message)
{
  if(logLevel >= this->logLevel)
  {
    std::cout << message << std::endl;
  }
}
