#pragma once

class MainWindow;
class QApplication;
class Session;

class Initializer
{
  QApplication* app = nullptr;
  MainWindow* mainWindow = nullptr;
  Session& session;
public:
  Initializer(int& argc, char** argv);
  ~Initializer();
  int start();
};
