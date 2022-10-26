#pragma once

#include <QGuiApplication>
#include <QScreen>

class SizeManager
{
public:
  int screenNumber = static_cast<int>(QGuiApplication::screens().size());
  int widgetWidth = QGuiApplication::primaryScreen()->virtualGeometry().width() / screenNumber;
  int widgetHeight = QGuiApplication::primaryScreen()->virtualGeometry().height();
  int robotBlockWidth = static_cast<int>(widgetWidth * 0.75f);
  int robotBlockHeight = widgetHeight;
  int robotViewWidth = robotBlockWidth / 6;
  int robotViewHeight = robotBlockHeight / 4;
  int statusWidgetWidth = static_cast<int>(robotViewWidth * 0.55f);
  int statusWidgetHeight = robotBlockHeight / 7;
  int statusBarWidth = statusWidgetWidth * 66 / 100;
  int statusBarHeight = statusWidgetHeight / 4;
};
