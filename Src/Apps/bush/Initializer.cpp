#include "Initializer.h"
#include "Session.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include "ui/MainWindow.h"
#include "Platform/BHAssert.h"
#include "Streaming/FunctionList.h"

#include <QApplication>

Initializer::Initializer(int& argc, char** argv) :
  session(Session::getInstance())
{
  FunctionList::execute();
  {
#ifdef WINDOWS
    ProcessRunner r("taskkill /F /IM ping.exe");
#else // Linux, MACOS
    ProcessRunner r("bash", {"-c", "ps axco pid,command | grep \" ping$\" | awk '{ print $1; }' | xargs kill"});
#endif
    r.run();
  }

  goToConfigDirectory(argv[0]);

  app = new QApplication(argc, argv);
  app->setApplicationName("B-Human User Shell (bush)");
  app->setCursorFlashTime(0);
#ifdef WINDOWS
  app->setStyle("fusion");
#endif

  {
#ifdef WINDOWS
    ProcessRunner r("cmd /c bash -c \"cp ../Install/Keys/id_rsa_nao /tmp/id_rsa_nao\"");
#else
    ProcessRunner r("cp ../Install/Keys/id_rsa_nao /tmp/id_rsa_nao");
#endif
    r.run();
    ASSERT(!r.error());
  }

  session.initialize();

  mainWindow = new MainWindow;
  mainWindow->show();
}

Initializer::~Initializer()
{
  delete mainWindow;
  delete app;

  session.destroy();
}

int Initializer::start()
{
  return app->exec();
}
