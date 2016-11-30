#pragma once

#include <QMainWindow>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
private:
  void closeEvent(QCloseEvent* event);
  void readSettings();
};
