#pragma once
#include <QDesktopWidget>

class SizeManager
{
private:
  QDesktopWidget desktop;
public:
  int screenNumber = desktop.numScreens();
  int widgetWidth = desktop.width() / screenNumber;
  int widgetHeight = desktop.height();
  int robotBlockWidth = static_cast<int>(widgetWidth * 0.75);
  int robotBlockHeight = widgetHeight;
  int robotViewWidth = robotBlockWidth / 6;
  int robotViewHeight = robotBlockHeight / 4;
  int statusWidgetWidth = static_cast<int>(robotViewWidth * 0.55);
  int statusWidgetHeight = robotBlockHeight / 7;
  int statusBarWidth = statusWidgetWidth * 66 / 100;
  int statusBarHeight = statusWidgetHeight / 4;
};
