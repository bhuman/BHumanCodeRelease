/**
* @file Controller/Views/KickView/TabWidget.h
*
* Declaration of class TabWidget
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#pragma once

#include <QTabWidget>
#include <QTabBar>
#include <QObject>

class TabBar : public QTabBar
{
  Q_OBJECT
public:
  TabBar(QWidget* parent = 0);

private:
  QPoint m_dragStartPos;
  int m_dragCurrentIndex;
protected:
  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void dragEnterEvent(QDragEnterEvent* event);
  void dropEvent(QDropEvent* event);
signals:
  void tabMoveRequested(int fromIndex, int toIndex);
};

class TabWidget : public QTabWidget
{
public:
  TabWidget(QWidget* parent = 0);
  QTabBar* getTabBar() { return tb;}
private:
  QTabBar* tb;
};
