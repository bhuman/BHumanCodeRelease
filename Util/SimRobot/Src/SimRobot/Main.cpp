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

/** The address of the main window object used by the following class. */
MainWindow* mainWindow;

/**
 * A helper for opening files when they were launched from the Finder.
 * macOS triggers an event for them rather than passing them as a command line
 * parameter. This class handles that event.
 */
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

/** Use the new class rather than the default one. */
#define QApplication SimRobotApp

/**
 * A helper for diplaying QStrings in Xcode that converts UTF16 strings to UTF8
 * character arrays that Xcode can display.
 * @param string The string to convert.
 * @return The address of the converted string. Note that it points to a static
 *         buffer that will be reused in the next call.
 */
const char* _fromQString(const QString& string)
{
  static char buffer[1000];
  strncpy(buffer, string.toUtf8().constData(), sizeof(buffer));
  buffer[sizeof(buffer) - 1] = 0;
  return buffer;
}
#endif // MACOS

int main(int argc, char *argv[])
{
#ifdef WINDOWS
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
  //_CrtSetBreakAlloc(18969); // Use to track down memory leaks
#endif
#if QT_VERSION >= QT_VERSION_CHECK(5, 6, 0)
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif
  QApplication app(argc, argv);
  MainWindow mainWindow(argc, argv);

#ifdef WINDOWS
  app.setStyle("fusion");
#elif defined MACOS
  app.setStyle("macintosh");
  ::mainWindow = &mainWindow;
#endif

  app.setApplicationName("SimRobot");

#ifdef MACOS
  mainWindow.show();
#endif

  // open file from commandline
  for(int i = 1; i < argc; i++)
    if(*argv[i] != '-' && strcmp(argv[i], "YES"))
    {
      mainWindow.openFile(argv[i]);
      break;
    }

#ifndef MACOS
  mainWindow.show();
#endif

  return app.exec();
}
