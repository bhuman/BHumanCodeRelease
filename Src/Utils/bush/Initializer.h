#pragma once

#include "Utils/bush/Session.h"

class QApplication;
class MainWindow;

class Initializer
{
  LogLevel logLevel;
  QApplication* app;
  MainWindow* mainWindow;
public:
  Initializer(int& argc, char** argv);
  ~Initializer();
  int start();
  void log(LogLevel logLevel, const std::string& message);
};
