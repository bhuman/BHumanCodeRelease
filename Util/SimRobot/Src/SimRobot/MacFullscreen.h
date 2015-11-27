#pragma once

class MainWindow;

class MacFullscreen
{
private:
  static bool available;

public:
  static void enable(MainWindow* window);
  static bool isActive(MainWindow* window);
  static void setActive(MainWindow* window, bool active);
};
