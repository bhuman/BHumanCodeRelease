/**
* @file SimRobot/Main.cpp
* Implementation of the main function of SimRobot
* @author Colin Graf
*/

#include <QApplication>
#include <QTextCodec>

#ifdef WINDOWS
#include "qtdotnetstyle.h"
#include <crtdbg.h>
#endif
#include "MainWindow.h"

#ifdef OSX
#include <QFileOpenEvent>
#include "MacFullscreen.h"

MainWindow* mainWindow;

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

#endif

int main(int argc, char *argv[])
{
  QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
  QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));
#ifdef WINDOWS
  QApplication::setStyle(new QtDotNetStyle(QtDotNetStyle::Standard));
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
  //_CrtSetBreakAlloc(91417); // Use to track down memory leaks
#endif
  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);
#ifdef OSX
  ::mainWindow = &mainWindow;
  app.setStyle("macintosh");
  MacFullscreen::enable(&mainWindow);
#endif
  app.setApplicationName("SimRobot");

  // open file from commandline
  for(int i = 1; i < argc; i++)
    if(*argv[i] != '-')
    {
#ifdef OSX
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

  int result = 0;
#ifdef FIX_WIN32_WINDOWS7_BLOCKING_BUG
  if(QSysInfo::windowsVersion() <= QSysInfo::WV_WINDOWS7)
    while(mainWindow.isVisible())
    {
      app.sendPostedEvents();
      app.processEvents(mainWindow.timerId ? QEventLoop::AllEvents : QEventLoop::WaitForMoreEvents);
      if(mainWindow.timerId)
        mainWindow.timerEvent(0);
    }
  else
#endif
    result = app.exec();
#ifdef LINUX
  exit(result); // fixes a mysterious segfault
#endif
  return result;
}
