/**
* @file SimRobot/Main.cpp
* Implementation of the main function of SimRobot
* @author Colin Graf
*/

#include <QApplication>
#include <QTextCodec>

#ifdef _WIN32
#include "qtdotnetstyle.h"
#endif
#include "MainWindow.h"

#ifdef MACOSX
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QFileOpenEvent>
#include "MacFullscreen.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

class SimRobotApp : public QApplication
{
public:
  SimRobotApp(int &argc, char **argv)
  : QApplication(argc, argv) {}

  MainWindow* mainWindow;

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
#endif

int main(int argc, char *argv[])
{
  QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
#ifdef _WIN32
  QApplication::setStyle(new QtDotNetStyle(QtDotNetStyle::Standard));
#endif
  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);
#ifdef MACOSX
#ifdef MAC_OS_X_VERSION_10_9
  QFont::insertSubstitution(".Lucida Grande UI", "Lucida Grande");
#endif
  app.mainWindow = &mainWindow;
  app.setStyle("macintosh");
  MacFullscreen::enable(&mainWindow);
#endif
  app.setApplicationName("SimRobot");

  // open file from commandline
  for(int i = 1; i < argc; i++)
    if(*argv[i] != '-')
    {
#ifdef MACOSX
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
