/**
* @file SimRobot/Main.cpp
* Implementation of the main function of SimRobot
* @author Colin Graf
*/

#include <QApplication>

#ifdef WINDOWS
#include <crtdbg.h>
#endif
#include "MainWindow.h"

#ifdef MACOS
#include <QFileOpenEvent>

static MainWindow* mainWindow;

class SimRobotApp : public QApplication
{
public:
  SimRobotApp(int &argc, char **argv)
  : QApplication(argc, argv) {}

protected:
  bool event(QEvent *ev)
  {
    if(ev->type() == QEvent::FileOpen)
    {
      mainWindow->openFile(static_cast<QFileOpenEvent*>(ev)->file());
      return true;
    }
    else
      return QApplication::event(ev);
  }
};

#define QApplication SimRobotApp

const char* _fromQString(const QString& string)
{
  static char buffer[1000];
  strncpy(buffer, string.toUtf8().constData(), sizeof(buffer));
  buffer[sizeof(buffer) - 1] = 0;
  return buffer;
}

#ifdef NDEBUG
static void ignoreMessageOutput(QtMsgType, const QMessageLogContext&, const QString&) {}
#endif

#endif

int main(int argc, char *argv[])
{
#ifdef WINDOWS
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
  //_CrtSetBreakAlloc(18969); // Use to track down memory leaks
#endif

  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);

#ifdef WINDOWS
  app.setStyle("fusion");
#endif
#ifdef MACOS
#ifdef NDEBUG
  qInstallMessageHandler(ignoreMessageOutput);
#endif
  ::mainWindow = &mainWindow;
  app.setStyle("macintosh");
#endif
  app.setApplicationName("SimRobot");

  // open file from commandline
  for(int i = 1; i < argc; i++)
    if(*argv[i] != '-')
    {
#ifdef MACOS
      if(strcmp(argv[i], "YES"))
      {
        mainWindow.setWindowOpacity(0);
        mainWindow.show();
        mainWindow.openFile(argv[i]);
        mainWindow.setWindowOpacity(1);
      }
#else
      mainWindow.openFile(argv[i]);
#endif
      break;
    }

  mainWindow.show();
  int result = app.exec();
#ifdef LINUX
  exit(result); // fixes a mysterious segfault
#endif
  return result;
}
