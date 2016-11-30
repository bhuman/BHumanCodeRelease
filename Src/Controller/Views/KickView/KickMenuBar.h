/**
 * @file Controller/Views/KickView/KickMenuBar.h
 *
 * Declaration of class KickMenuBar
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include <QMenuBar>
#include <QWidget>

class KickMenuBar : public QMenuBar
{
public:
  QAction* xy_plane;
  QAction* xz_plane;
  QAction* yz_plane;

  QMenu* phaseMenu;
  QMenu* dragPlaneMenu;

  QActionGroup* dragPlaneActionGroup;

  KickMenuBar();

private:
  void createActions();
  void createMenus();
};
